import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, IfElseSubstitution, EqualsSubstitution, TextSubstitution

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="fake",
        description="Robot IP address to connect to. If left unspecified, a fake robot instance will be created.",
        )

    robot_ip = LaunchConfiguration('robot_ip', default='fake')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')

    movefile = IfElseSubstitution(EqualsSubstitution(
            robot_ip, TextSubstitution(text='fake')),
            "lite6_moveit_fake.launch.py",
            "lite6_moveit_realmove.launch.py",
            )

    lite6_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch',  movefile])),
        launch_arguments={
            'robot_ip': robot_ip,
            'hw_ns': hw_ns,
        }.items(),
    )

    return LaunchDescription(
        [
            robot_ip_arg,
            lite6_launch,
        ]
    )
