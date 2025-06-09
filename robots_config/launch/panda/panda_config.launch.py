import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(name="rviz", default_value="True"),
    ]

    # Urdf
    rc_dir = get_package_share_directory("robots_config")
    urdf_string = xacro.process_file(
        os.path.join(rc_dir, "robots/panda/panda.urdf.xacro"),
        mappings ={'hand': 'true'}
    )    
    robot_description = {"robot_description": urdf_string.toxml()}

    # Robot state publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Joint State publisher
    panda_zero_joints = {
      "zeros.fr3_joint4": -1.5708,
      "zeros.fr3_joint6": 1.5708 	
    }
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[panda_zero_joints],
        output='screen')

    # Rviz
    rviz_config = os.path.join(rc_dir, "rviz/panda.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
        robot_description
        ]
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base"],
    )

    return LaunchDescription(
        launch_args +
        [rsp, jsp, rviz, static_tf]
    )
