import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_param_builder import load_xacro

from launch_pal.include_utils import include_launch_py_description

from launch.substitutions import Command, PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument

from pathlib import Path

def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(name="rviz", default_value="True"),
    ]

    # URDF
    robot_config = get_package_share_directory("robots_config")
    xacro_file_path = Path(os.path.join(robot_config, "robots/tiago/tiago.urdf.xacro"))

    xacro_input_args = {
        "arm_type": "tiago-arm",
        "camera_model": "orbbec-astra",
        "end_effector": "pal-gripper",
        "ft_sensor": "schunk-ft",
        "laser_model": "sick-571",
        "wrist_model": "wrist-2010",
        "base_type": "pmb2",
        "has_screen": False
    }
    urdf_config = load_xacro(xacro_file_path, xacro_input_args)    
    robot_description = {'robot_description': urdf_config}

    # Robot state publisher
    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               output='both',
               parameters=[robot_description])

    # Joint state publisher
    zeros_yaml = os.path.join(robot_config, 'config/tiago/zeros.yaml')
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[zeros_yaml],
        output='screen')

    # Rviz
    rviz_full_config = os.path.join(robot_config, "rviz/tiago.rviz")
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_full_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')))

    # Move base simulation
    move_base = Node(
        package='reachability_demos',
        executable='simulate_robot_base_motion',
        name='simulate_robot_base_motion',
        output='screen',
        parameters=[{
        'ref_frame': 'world',
        'robot_frame': 'base_footprint',
        'init_x': 0.0, 'init_y': 0.0, 'init_z': 0.0, 
        'init_roll': 0.0, 'init_pitch': 0.0, 'init_yaw': 0.0}]
    )
    

    return LaunchDescription(
        launch_args + [
        rsp,
        jsp,
        move_base,
        rviz
    ])
    
    
    
