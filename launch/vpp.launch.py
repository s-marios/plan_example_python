
"""
A launch file for running the motion planning python api tutorial
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('plan_example_python'), 'launch',  'common.launch.py'])),
    )

    spawn_exec_file = DeclareLaunchArgument(
        "object_spawner_exec",
        default_value="object_spawner",
        description="Spawn virtual objects for the robot to pick up",
        choices=["object_spawner", "random_object_spawner"]
        )

    obj_spawn_node = Node(
        name="obj_spawn",
        package="plan_example_python",
        executable=LaunchConfiguration("object_spawner_exec"),
        output="both",
        )


    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="lite6", package_name="plan_example_python"
            )
        .robot_description(file_path="config/lite6.urdf")
        .robot_description_semantic(file_path="config/lite6.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("plan_example_python")
            + "/config/motion_planning_python_api_tutorial.yaml"
            )
        .to_moveit_configs()
        )

    pick_place_node = Node(
        name="pick_place_moveit_py",
        package="plan_example_python",
        executable="pick_place",
        output="both",
        parameters=[moveit_config.to_dict()],
        )

    return LaunchDescription(
        [
            common_launch,
            spawn_exec_file,
            obj_spawn_node,
            pick_place_node,
        ]
    )
