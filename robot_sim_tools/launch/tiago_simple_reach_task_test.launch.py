import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro

from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch_param_builder import load_xacro
from pathlib import Path

#####################################
# Helpers functions
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


#####################################
def generate_launch_description():

    # Robot task UI Params
    rtu_yaml = load_yaml(
        "task_ui", "config/tiago_robot_task.yaml"
    )
    rtu_params = {"robot_task_ui_params": rtu_yaml}

    # markers
    task_marker = Node(
        package='task_ui',
        executable='markers_get_robot_base_node',
        output='screen',
        parameters=[
            {"group": "arm_torso"},
            {"robot_name": "tiago"},
            rtu_params
        ]
    )    

    # Robot to task
#    app_robot_to_task = Node(
#        package='reachability_description',
#        executable='app_robot_to_task_2',
#        output='screen',
#        parameters=[
#            reachability_params,
#            {"robot_description": urdf_config},
#            {"robot_description_semantic" : srdf_config},
#            {"chain_group_name": "arm_torso"},
#            {"robot_name": "tiago"}
#        ]
#    )    




    return LaunchDescription(
        [
          task_marker,
#          app_robot_to_task
        ]

    )
