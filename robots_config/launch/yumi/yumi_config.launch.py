import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

##########################################
def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(name="rviz", default_value="True"),
    ]

    # Urdf  
    rc_dir = get_package_share_directory("robots_config")
    
    robot_description_config = xacro.process_file(
        os.path.join(rc_dir, "robots/yumi/yumi.urdf.xacro",
        ),
        in_order = False,
        mappings = {'arms_interface': 'VelocityJointInterface', 
                    'grippers_interface': 'EffortJointInterface',
                    'yumi_setup' : 'default'}
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Robot state publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Joint State publisher
    yumi_zero_joints = { 
        "zeros": {
          "yumi_joint_1_l": 0, 
          "yumi_joint_2_l": -1.57, 
          "yumi_joint_7_l": 1.57, 
          "yumi_joint_3_l": 0.7, 
          "yumi_joint_4_l": -1.57, 
          "yumi_joint_5_l": 1.3, 
          "yumi_joint_6_l": 1.57, 
          "yumi_joint_1_r": 0, 
          "yumi_joint_2_r": -1.57, 
          "yumi_joint_7_r": -1.57, 
          "yumi_joint_3_r": 0.7, 
          "yumi_joint_4_r": -1.57, 
          "yumi_joint_5_r": -1.3, 
          "yumi_joint_6_r": -1.57        
       }
    }

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[yumi_zero_joints],
        output='screen')

    # Rviz
    rviz_full_config = os.path.join(rc_dir, "rviz/yumi.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
    )


    return LaunchDescription(
        launch_args +
        [rviz, rsp, jsp]
    )
