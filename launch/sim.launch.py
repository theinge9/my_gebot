import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = os.path.join(get_package_share_directory('my_gebot'))
    rviz_config_path = os.path.join(pkg_name, 'rviz/urdf.rviz')
    world_path = os.path.join(pkg_name, 'worlds', 'test1.world')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_name, 'launch', 'rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'
                ]), launch_arguments={'world': world_path}.items()
    )
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'gebot'],
                        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    control_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tricycle_controller'],
    )
    
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='rvizconfig', default_value=rviz_config_path, 
            description='Absolute path to rviz config file'),
        rsp,
        gazebo,
        spawn_entity,
        rviz_node,
        control_drive_spawner,
        joint_state_broadcaster
    ])