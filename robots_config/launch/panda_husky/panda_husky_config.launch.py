
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("robots_config"),
            "robots", "panda_husky",
            "panda_husky.urdf.xacro",
        ),
        mappings ={'hand': 'true'}
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    srdf_file = os.path.join(get_package_share_directory('robots_config'),'config',
                                              'panda_husky',
                                              'panda_husky.srdf.xacro')
    srdf_config = Command(
        [FindExecutable(name='xacro'), ' ', srdf_file, ' hand:=true']
    )
    robot_description_semantic = {
        'robot_description_semantic': srdf_config
    }

    panda_zero_joints = {
      "zeros.fr3_joint4": -1.5708,
      "zeros.fr3_joint6": 1.5708 	
    }


    rviz_file = os.path.join(get_package_share_directory('robots_config'), 'rviz',
                             'panda_husky.rviz')

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[panda_zero_joints],
            output='screen'
        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file])
    ])
