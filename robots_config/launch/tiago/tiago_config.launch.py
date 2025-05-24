import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_param_builder import load_xacro

from launch_pal.include_utils import include_launch_py_description

from launch.substitutions import Command, PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from pathlib import Path

def generate_launch_description():

    xacro_file_path = Path(
        os.path.join(
            get_package_share_directory("robots_config"),
            "robots", 'tiago',
            "tiago.urdf.xacro",
        )
    )

    xacro_input_args = {
        "arm_type": "tiago-arm",
        "camera_model": "orbbec-astra",
        "end_effector": "pal-gripper",
        "ft_sensor": "schunk-ft",
        "laser_model": "sick-571",
        "wrist_model": "wrist-2010",
        "base_type": "pmb2",
        "has_screen": False,
#        "use_sim_time": False,
#        "is_public_sim": True,
#        "namespace": read_launch_argument("namespace", context),
    }
    urdf_config = load_xacro(xacro_file_path, xacro_input_args)
    
    
    parameters = {'robot_description': urdf_config}

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               output='both',
               parameters=[{'robot_description': urdf_config}])

    zeros_yaml = os.path.join(get_package_share_directory('robots_config'), 'config',
                             'tiago', 'zeros.yaml')
    joint_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[zeros_yaml],
        output='screen')

    rviz_base = os.path.join(get_package_share_directory("robots_config"), "rviz")
    rviz_full_config = os.path.join(rviz_base, "tiago.rviz")
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_full_config],
        output='screen')

    move_base = Node(
        package='robot_sim_tools',
        executable='simulate_robot_base_motion',
        name='simulate_robot_base_motion',
        parameters=[{
        'ref_frame': 'world',
        'robot_frame': 'base_footprint',
        'init_x': 0.0, 'init_y': 0.0, 'init_z': 0.0, 
        'init_roll': 0.0, 'init_pitch': 0.0, 'init_yaw': 0.5}],
        output='screen')
    

    return LaunchDescription([
        rsp,
        joint_pub,
        start_rviz_cmd,
        move_base
    ])
    
    
    
