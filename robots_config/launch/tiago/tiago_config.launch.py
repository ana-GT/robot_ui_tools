import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_pal.include_utils import include_launch_py_description

from launch.substitutions import Command, PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage

from tiago_description.tiago_launch_utils import (get_tiago_hw_arguments,
						    TiagoXacroConfigSubstitution)


def generate_launch_description():

    urdf_config = Command(
        [
            ExecutableInPackage(package='xacro', executable="xacro"),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('robots_config'),
                 'robots', 'tiago', 'tiago.urdf.xacro']),
            TiagoXacroConfigSubstitution()
        ])


    parameters = {'robot_description': urdf_config}


    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               output='both',
               parameters=[{'robot_description': urdf_config}])

    tiago_args = get_tiago_hw_arguments(
        laser_model=True,
        arm=True,
        end_effector=True,
        ft_sensor=True,
        camera_model=True,
        default_end_effector='pal-gripper',
        default_laser_model="sick-571")

    joint_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
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
        *tiago_args,
        rsp,
        joint_pub,
        start_rviz_cmd
    ])
    
    
    
