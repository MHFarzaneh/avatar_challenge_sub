from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xarm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("xarm_moveit_config"),
                    "launch",
                    "xarm7_moveit_fake.launch.py",
                ]
            )
        ),
    )

    return LaunchDescription(
        [
            xarm_moveit_launch,
        ]
    )

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

'''def generate_launch_description():
    dof = LaunchConfiguration('dof')
    robot_type = LaunchConfiguration('robot_type', default='xarm')

    xarm_planner_node_test = Node(
        name='test_xarm_planner_client_pose',
        package='xarm_planner',
        executable='test_xarm_planner_client_pose',
        output='screen',
        parameters=[
            {
                'robot_type': 'xarm7',
                'dof': 7
            },
        ],
    )
    return LaunchDescription([
        xarm_planner_node_test
    ])'''


def generate_launch_description():
    xarm_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("xarm_planner"),
                    "launch",
                    "xarm7_planner_fake.launch.py",
                ]
            )
        ),
    )

    return LaunchDescription(
        [
            xarm_moveit_launch,
        ]
    )
