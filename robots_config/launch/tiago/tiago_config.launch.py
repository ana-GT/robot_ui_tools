import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_pal.include_utils import include_launch_py_description


def generate_launch_description():

    robot_state_publisher = include_launch_py_description(
        'tiago_description', ['launch', 'robot_state_publisher.launch.py'])

    sources_list = {
        "source_list": ["/move_group/fake_controller_joint_states"]
    }
    
    
    joint_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[sources_list],
        output='screen')

    rviz_base = os.path.join(get_package_share_directory("robots_config"), "rviz")
    rviz_full_config = os.path.join(rviz_base, "tiago.rviz")
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_full_config],
        output='screen')

    return LaunchDescription([
        robot_state_publisher,
        joint_pub,
        start_rviz_cmd
    ])
    
    
    
