
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

#    mars_rover_models_path = get_package_share_directory('simulation')
#    urdf_model_path = os.path.join(mars_rover_models_path, 'models', 'curiosity_path',
#        'urdf', 'curiosity_mars_rover.xacro.urdf')
    mars_rover_models_path = get_package_share_directory('robots_config')
    urdf_model_path = os.path.join(mars_rover_models_path, 'robots', 'curiosity',
        'curiosity_base_link.urdf.xacro')

    doc = xacro.process_file(urdf_model_path)
    robot_description = {'robot_description': doc.toxml()}

    rviz_file = os.path.join(get_package_share_directory('robots_config'), 'rviz',
                             'curiosity.rviz')

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
            output='screen'
        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file])
    ])