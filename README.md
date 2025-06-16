How to run:
In one terminal
ros2 launch avatar_challenge start.launch.py
in another
ros2 run avatar_challenge shape_tracer_node --ros-args -p shapes_file:=/absolute/path/to/shapes.yaml

start.launch.py is modified to start 'xarm_planner' as well.
yaml file includes 3 types: arc.bspline and blend
In the shape_tracer node for each shape, first '/compute_ik' service is called to find a valid joint solution for the first point, the command to move calling 'xarm_exec_plan';
then for the rest, calling 'xarm_straight_plan'.

In hindsight, it might have been a better choice to use CPP, since moveit_commander is not supported in ROS2 python.
Moreover, in order to visualize the path, cpp version provides "visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);"
I tried to use the move_group in CPP, however, I faced errors related to SRDF and robot description missing. It would be helpful to
launch xarm_description node to check if that will publish the missing topics.
