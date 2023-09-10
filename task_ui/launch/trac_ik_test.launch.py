from launch import LaunchDescription
from launch_ros.actions import Node

from tiago_description.tiago_launch_utils import (get_tiago_hw_arguments,
                                                  generate_robot_description_action)


def generate_launch_description():
    parameters = {'robot_description': generate_robot_description_action()}

    tiago_args = get_tiago_hw_arguments(
        laser_model=True,
        arm=True,
        end_effector=True,
        ft_sensor=True,
        camera_model=True,
        default_laser_model="sick-571")

    trac_ik_test = Node(package='task_ui',
                        executable='trac_ik_test',
                        output='screen',
                        parameters=[parameters])

    return LaunchDescription([*tiago_args, trac_ik_test])
