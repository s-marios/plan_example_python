"""
A launch file for running the motion planning python api tutorial
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from ros2launch.api.api import print_arguments_of_launch_description

import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_ip = LaunchConfiguration('robot_ip')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')

    # robot moveit realmove launch
    # xarm_moveit_config/launch/_robot_moveit_realmove.launch.py
    lite6_real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', 'lite6_moveit_realmove.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
            'hw_ns': hw_ns,
        }.items(),
    )





    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="lite6", package_name="plan_example_python"
            )
        #TODO
        .robot_description(file_path="config/lite6.urdf")
        .robot_description_semantic(file_path="config/lite6.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        #.robot_description_kinematics(file_path="config/kinematics.yaml")
        #we also pulled in all other files (limits, planning.yaml etc) into config
        .moveit_cpp(
            file_path=get_package_share_directory("plan_example_python")
            + "/config/motion_planning_python_api_tutorial.yaml"
            )
        .to_moveit_configs()
        )

    example_file = DeclareLaunchArgument(
        "example_file",
        # default_value="motion_planning_python_api_tutorial.py",
        default_value="plan",
        description="Python API tutorial file name",
        )

    moveit_py_node = Node(
        name="moveit_py",
        package="plan_example_python",
        executable=LaunchConfiguration("example_file"),
        # executable='plan',
        output="both",
        parameters=[moveit_config.to_dict()],
        )

    return LaunchDescription(
        [
            example_file,
            moveit_py_node,
            lite6_real_launch,
            ]
        )
