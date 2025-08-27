"""
A launch file for generating the lite6 srdf
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution



import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    #NOTE adjust the mappings to your liking

    # Assuming your xacro file is in a ROS package
    package_name = 'xarm_moveit_config'
    xacro_file_path = os.path.join(get_package_share_directory(package_name), 'srdf', 'xarm.srdf.xacro')
    srdf_config = xacro.process_file(xacro_file_path, mappings={'robot_type': 'lite', 'dof' : '6'})
    robot_desc_xml_string = srdf_config.toxml()
    print(robot_desc_xml_string)
    out = open("lite6.srdf",'w')
    out.write(robot_desc_xml_string)
    out.close()

    return LaunchDescription([])
