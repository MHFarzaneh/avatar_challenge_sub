import rclpy
from rclpy.node import Node
import yaml

from scipy.interpolate import splprep, splev

import numpy as np
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
from xarm_msgs.srv import PlanSingleStraight, PlanJoint
from moveit_msgs.srv import GetPositionIK
from xarm_msgs.srv import  PlanExec

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf_transformations import quaternion_matrix

# Function to transform 2D point to 3D using start pose position and orientation
# This assumes the 2D point is in the plane defined by the start pose orientation
# and that the Z coordinate is 0 in the local frame.
def transform_2d_point_to_3d(p2d, start_pose_pos, start_pose_quat):
    point_shape = np.array([p2d[0], p2d[1], 0, 1])
    T = quaternion_matrix(start_pose_quat)
    T[0:3, 3] = start_pose_pos
    point_global = T @ point_shape
    return point_global[0:3]

# Function to generate points for an arc segment
# center: [x, y] coordinates of the arc center
# radius: radius of the arc
# start_angle: starting angle in radians
# end_angle: ending angle in radians
# clockwise: whether the arc is clockwise or counter-clockwise
# resolution: number of points to generate along the arc
# Returns a list of [x, y] points along the arc
def generate_arc_points(center, radius, start_angle, end_angle, clockwise=False, resolution=20):
    if clockwise and end_angle > start_angle:
        end_angle -= 2 * np.pi
    elif not clockwise and end_angle < start_angle:
        end_angle += 2 * np.pi

    angles = np.linspace(start_angle, end_angle, resolution)
    return [[center[0] + radius * np.cos(a), center[1] + radius * np.sin(a)] for a in angles]

# Function to blend points for smooth transitions
# p1, p2, p3: points in 2D space to blend between
# blend_radius: radius for the blending transition
# Returns a list of blended points including start and end points   
def blend_points(p1, p2, p3, blend_radius=0.01):
    # Shorten p2 to create a transition zone
    v1 = np.array(p2) - np.array(p1)
    v2 = np.array(p3) - np.array(p2)

    v1_unit = v1 / np.linalg.norm(v1)
    v2_unit = v2 / np.linalg.norm(v2)

    p2_start = p2 - v1_unit * blend_radius
    p2_end = p2 + v2_unit * blend_radius

    return [p2_start.tolist(), p2_end.tolist()]

# Function to generate B-spline points from control points
# control_points: list of [x, y] control points
# resolution: number of points to generate along the spline
# Returns a list of [x, y] points along the B-spline curve
def generate_bspline_points(control_points, resolution=30):
    control_points = np.array(control_points).T  # (2, N)
    tck, _ = splprep(control_points, s=0)
    u = np.linspace(0, 1, resolution)
    spline = splev(u, tck)
    return list(zip(spline[0], spline[1]))

# Function to generate a 3D pose from a 2D point and start pose
# p2d: 2D point [x, y]
# start_pos: start pose position [x, y, z]
# quat: orientation quaternion [x, y, z, w]
# Returns a 3D pose message
class ShapeTracerNode(Node):
    def __init__(self):
        super().__init__('shape_tracer_node')

        # Create publisher for visualization markers
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.shape_marker_id = 0

        # Load YAML file path param
        self.declare_parameter('shapes_file', 'shapes/shapes.yaml')
        full_yaml_path = self.get_parameter('shapes_file').get_parameter_value().string_value

        # Load shapes data from YAML file
        with open(full_yaml_path, 'r') as f:
            self.shapes_data = yaml.safe_load(f)['shapes']

        self.get_logger().info(f"Loaded {len(self.shapes_data)} shapes")

        # Create clients for planning services
        self.exec_client = self.create_client(PlanExec, 'xarm_exec_plan')

        # Create clients for services
        self.joint_plan_client = self.create_client(PlanJoint, '/xarm_joint_plan')
        while not self.joint_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /xarm_joint_plan service...')

        # Create straight plan service client
        self.straight_plan_client = self.create_client(PlanSingleStraight, '/xarm_straight_plan')
        while not self.straight_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /xarm_straight_plan service...')

        # Create IK service client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /compute_ik service...')

        self.exec_client.wait_for_service()

        self.trace_shapes()

    # Main function to trace shapes
    # Iterates through each shape, processes segments, and publishes markers
    # Uses IK to move to the first point, then traces remaining
    def trace_shapes(self):
        exec_req = PlanExec.Request()
        exec_req.wait = True

        # Iterate through each shape in the loaded data
        for shape_idx, shape in enumerate(self.shapes_data):
            name = shape.get('name', f'shape_{shape_idx}')
            self.get_logger().info(f"Tracing shape '{name}'")

            start_pos = shape['start_pose']['position']
            rpy = shape['start_pose']['orientation_rpy']
            quat = quaternion_from_euler(*rpy)

            segments = shape.get('segments', []) 
            if not segments:
                vertices_2d = shape['vertices']
                segments = [{"type": "segment", "points": vertices_2d}]

            all_points_2d = []
            
            # Process each segment in the shape
            # Handles segment, arc, B-spline, and blend types
            for seg in segments:
                if seg["type"] == "segment":
                    all_points_2d.extend(seg["points"])

                elif seg["type"] == "arc":
                    arc_points = generate_arc_points(
                        center=seg["center"],
                        radius=seg["radius"],
                        start_angle=seg["start_angle"],
                        end_angle=seg["end_angle"],
                        clockwise=seg.get("clockwise", False),
                        resolution=seg.get("resolution", 20)
                    )
                    all_points_2d.extend(arc_points)

                elif seg["type"] == "bspline":
                    bspline_pts = generate_bspline_points(seg["control_points"], resolution=seg.get("resolution", 30))
                    all_points_2d.extend(bspline_pts)

                elif seg["type"] == "blend":
                    # Expect points to be [p1, p2, p3]
                    p1, p2, p3 = seg["points"]
                    blended = blend_points(p1, p2, p3, blend_radius=seg.get("blend_radius", 0.01))
                    all_points_2d.extend([p1] + blended + [p3])  # Include start and end

                else:
                    self.get_logger().warn(f"Unknown segment type '{seg['type']}' in shape '{name}'")

            points_3d = [transform_2d_point_to_3d(p2d, start_pos, quat) for p2d in all_points_2d]
            self.publish_line_strip_marker(name, points_3d)

            # Check if we have valid 3D points to trace
            if not points_3d:
                self.get_logger().warn(f"No valid points found for shape '{name}'")
                continue

            # Move to first point using joint plan
            joint_positions = self.call_ik_service(points_3d[0], quat)
            if joint_positions is None:
                self.get_logger().error("IK failed for first point. Aborting shape trace.")
                continue

            # Prepare the execution request
            self.get_logger().info(f"Moving to first point of '{name}' using joint plan")
            req_joint = PlanJoint.Request()
            req_joint.target = joint_positions
            future_joint = self.joint_plan_client.call_async(req_joint)
            rclpy.spin_until_future_complete(self, future_joint)
            if future_joint.result() is None or not future_joint.result().success:
                self.get_logger().error(f"Joint plan failed for first point of '{name}'. Skipping shape.")
                continue

            exec_future = self.exec_client.call_async(exec_req)
            rclpy.spin_until_future_complete(self, exec_future)

            # Trace remaining points with straight plans
            for idx, point_3d in enumerate(points_3d[1:], start=1):
                pose_msg = Pose()
                pose_msg.position.x = point_3d[0]
                pose_msg.position.y = point_3d[1]
                pose_msg.position.z = point_3d[2]
                pose_msg.orientation.x = quat[0]
                pose_msg.orientation.y = quat[1]
                pose_msg.orientation.z = quat[2]
                pose_msg.orientation.w = quat[3]

                self.get_logger().info(f"Straight planning to point {idx} of '{name}'")

                # Create a request for the straight plan
                # This assumes the straight plan service expects a Pose message
                # Adjust if the service requires a different format
                req_straight = PlanSingleStraight.Request()
                req_straight.target = pose_msg
                future_straight = self.straight_plan_client.call_async(req_straight)
                rclpy.spin_until_future_complete(self, future_straight)

                # Check if the straight plan was successful 
                if future_straight.result() is None or not future_straight.result().success:
                    self.get_logger().error(f"Straight plan failed at point {idx} of '{name}'. Aborting shape trace.")
                    break

                exec_future = self.exec_client.call_async(exec_req)
                rclpy.spin_until_future_complete(self, exec_future)

            self.get_logger().info(f"Finished tracing shape '{name}'")

        self.get_logger().info("All shapes traced.")


    # Function to call the IK service with a given Cartesian pose
    # Returns joint positions if successful, else None
    # Uses the xarm7 group and expects the joint order to be [joint2, joint3, joint4, joint5, joint6, joint1, joint7]
    # The IK service is expected to return joint positions that can be used for planning
    # The position is a 3D point and orientation_quat is a quaternion representing the desired orientation
    # The function handles the service call and checks for success or failure
    # If the service call fails or returns an error code, it logs an error message and returns None
    # If successful, it logs the joint positions and returns them for further processing
    # This function is used to compute the joint positions needed to reach a specific Cartesian pose
    # It is essential for moving the robot arm to the desired position and orientation in 3D space
    # The IK service is a critical part of the motion planning process, allowing the robot to determine how to move its joints
    # to achieve a specific end-effector pose in the workspace.
    # The function uses the GetPositionIK service from the moveit_msgs package to compute the inverse kinematics
    # for the xarm7 robot arm, which is a common task in robotic applications where precise positioning is required.
    def call_ik_service(self, position, orientation_quat):
        """
        Call /compute_ik service with given Cartesian pose.
        Returns list of joint positions if success, else None.
        """
        req = GetPositionIK.Request()
        req.ik_request.group_name = "xarm7"

        # Use correct joint order expected by the IK solver
        req.ik_request.robot_state.joint_state.name = [
            "joint2", "joint3", "joint4", "joint5", "joint6", "joint1", "joint7"
        ]
        req.ik_request.robot_state.joint_state.position = [0.0] * 7

        # Create a PoseStamped message for the IK request
        # This includes the position and orientation in the robot's base frame
        # The position is a 3D point and the orientation is given as a quaternion
        # The PoseStamped message is used to specify the desired end-effector pose in the robot's coordinate frame
        from geometry_msgs.msg import PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "link_base"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = position[0]
        pose_stamped.pose.position.y = position[1]
        pose_stamped.pose.position.z = position[2]
        pose_stamped.pose.orientation.x = orientation_quat[0]
        pose_stamped.pose.orientation.y = orientation_quat[1]
        pose_stamped.pose.orientation.z = orientation_quat[2]
        pose_stamped.pose.orientation.w = orientation_quat[3]

        req.ik_request.pose_stamped = pose_stamped

        req.ik_request.timeout.sec = 1
        req.ik_request.timeout.nanosec = 0

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res is None:
            self.get_logger().error("IK service call failed (no response).")
            return None
        if res.error_code.val != 1:
            self.get_logger().warn(f"IK failed with error code {res.error_code.val}")
            return None

        joint_positions = res.solution.joint_state.position
        self.get_logger().error(f"IK Success! Joint positions: {joint_positions}")

        return joint_positions


    # Function to publish a line strip marker for visualizing shapes        
    def publish_line_strip_marker(self, shape_name, points_3d, color=(0.0, 1.0, 0.0)):
        marker = Marker()
        marker.header.frame_id = "link_base"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "shape_lines"
        marker.id = self.shape_marker_id
        self.shape_marker_id += 1
    
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
    
        marker.scale.x = 0.01  # Line width
    
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
    
        for pt in points_3d:
            p = Point()
            p.x, p.y, p.z = pt
            marker.points.append(p)
    
        # Optionally close the shape by linking last to first point
        if len(points_3d) > 2:
            p = Point()
            p.x, p.y, p.z = points_3d[0]
            marker.points.append(p)
    
        self.marker_pub.publish(marker)
        self.get_logger().info(f"Published shape lines for '{shape_name}'")


def main(args=None):
    rclpy.init(args=args)
    node = ShapeTracerNode()

    # Instead of shutting down after done, just spin to keep node alive
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # allow Ctrl-C to exit cleanly
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


''' It would have been a better choice to go with CPP version, since Moveit_commander in python no loner supported by ROS2
Moreover, in order o visualize the path, cpp version provides "visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);"
I tried to use the move_group in CPP, however, I faced error related to SRDF and robot description missing. It would be helpful to
launch xarm_description node to check if that will publish the missing topics.
'''