
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

  launch_args = [
        DeclareLaunchArgument(name="rviz", default_value="True"),
  ]

  rc_dir = get_package_share_directory("robots_config")
  robot_description_config = xacro.process_file(
        os.path.join(
            rc_dir,
            "robots", "panda_husky",
            "panda_husky.urdf.xacro",
        ),
        mappings ={'hand': 'true'}
  )
  
  robot_description = {"robot_description": robot_description_config.toxml()}

  srdf_file = os.path.join(rc_dir,'config', 'panda_husky', 'panda_husky.srdf.xacro')
  srdf_config = Command(
        [FindExecutable(name='xacro'), ' ', srdf_file, ' hand:=true']
  )
  robot_description_semantic = {'robot_description_semantic': srdf_config}

  panda_zero_joints = {
      "zeros.fr3_joint4": -1.5708,
      "zeros.fr3_joint6": 1.5708 	
  }


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

  rviz_file = os.path.join(get_package_share_directory('robots_config'), 'rviz',
                             'panda_husky.rviz')

  rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        )
        
  jsp = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[panda_zero_joints],
            output='screen')
            
  rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file])   

  return LaunchDescription(launch_args + [
      rsp,
      jsp,
      move_base,
      rviz
  ])    
