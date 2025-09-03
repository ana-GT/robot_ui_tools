
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import xacro

def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(name="rviz", default_value="True"),
    ]

    # Urdf
    rc_dir = get_package_share_directory("robots_config")
    urdf_string = xacro.process_file(os.path.join(rc_dir, 'robots/fetch/fetch.urdf.xacro'))
    robot_description = {"robot_description": urdf_string.toxml()}

    # Robot state publisher
    rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
    )

    # Joint state publisher
    zeros_yaml = os.path.join(rc_dir, 'config/fetch/zeros.yaml')
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

    # Move base simulation
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

    # Rviz
    rviz_file = os.path.join(rc_dir, 'rviz/fetch.rviz')    
    rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(LaunchConfiguration("rviz")),
             output="screen"
    )


    return LaunchDescription(launch_args + [
      rsp,
      jsp,
      move_base,
      rviz
    ])
