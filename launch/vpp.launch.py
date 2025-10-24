
"""
A launch file for running the motion planning python api tutorial
"""

import os

from typing import Optional, List

from launch import LaunchDescription, LaunchDescriptionEntity, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory

def prepare_node_config(context: LaunchContext, *args, **kwargs) -> Optional[List[LaunchDescriptionEntity]]:
    urdf="config/lite6_ng.urdf"
    srdf="config/lite6_ng.srdf"
    add_vacuum_gripper_string = LaunchConfiguration("add_vacuum_gripper", default="false").perform(context).lower()
    if add_vacuum_gripper_string == "true":
        urdf="config/lite6_vg.urdf"
        srdf="config/lite6_vg.srdf"

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="lite6", package_name="plan_example_python"
            )
        .robot_description(file_path=urdf)
        .robot_description_semantic(file_path=srdf)
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
        parameters=[
            moveit_config.to_dict(),
            {
                "robot_ip" : LaunchConfiguration("robot_ip"),
                "add_vacuum_gripper" : LaunchConfiguration("add_vacuum_gripper", default=False )
            },
        ],
        )

    return [pick_place_node]

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

    return LaunchDescription(
        [
            common_launch,
            spawn_exec_file,
            obj_spawn_node,
            OpaqueFunction(function=prepare_node_config),
        ]
    )
