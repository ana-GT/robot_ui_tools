
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    fetch_urdf_file = os.path.join(get_package_share_directory('fetch_description'), 'robots',
                                     'fetch.urdf')
    robot_description = open(fetch_urdf_file).read()

    rviz_file = os.path.join(get_package_share_directory('robots_config'), 'rviz',
                             'fetch.rviz')

    rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
    )

    zeros_yaml = os.path.join(get_package_share_directory('robots_config'), 'config',
                             'fetch', 'zeros.yaml')
    jsp = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
             zeros_yaml,
             { "rate": 10, 
               "source_list": ["joint_state_command"]
             }],
            output='screen'
    )
    
    move_base = Node(
            package='reachability_demos',
            executable='simulate_robot_base_motion',
            name='simulate_robot_base_motion',
            output='screen',
            parameters = [{'ref_frame': 'world', 
                      'robot_frame': 'base_link',
                      'init_x': 0.0,
                      'init_y': 0.0,
                      'init_z': 0.0,
                      'init_roll': 0.0,
                      'init_pitch': 0.0,
                      'init_yaw': 0.0
                      }]
    )

    
    rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file]
    )


    return LaunchDescription([
      rsp,
      jsp,
      move_base,
      rviz
    ])
