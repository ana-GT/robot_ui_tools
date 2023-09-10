import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_pal.substitutions import LoadFile
from ament_index_python.packages import get_package_share_directory


from tiago_description.tiago_launch_utils import (get_tiago_hw_arguments,
                                                  generate_robot_description_action)

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

    robot_description_semantic_config = load_file(
        "robots_config", "config/tiago/tiago_pal-hey5.srdf"
    )
    

    parameters = {'robot_description': generate_robot_description_action(),
                  'robot_description_semantic': robot_description_semantic_config,
                  'group': "arm"}

    
    tiago_args = get_tiago_hw_arguments(
        laser_model=True,
        arm=True,
        end_effector=True,
        ft_sensor=True,
        camera_model=True,
        default_laser_model="sick-571")

    task_ui_markers = Node(package='task_ui',
                        executable='task_ui_markers_node',
                        output='screen',
                        parameters=[parameters])

    return LaunchDescription([*tiago_args, task_ui_markers])
