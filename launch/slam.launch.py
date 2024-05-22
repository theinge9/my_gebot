import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = os.path.join(get_package_share_directory('my_gebot'))
    rviz_config_path = os.path.join(pkg_name, 'rviz', 'slam.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_params_file = LaunchConfiguration('slam_params_file')
    
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_name, 'config', 'slam.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation/Gazebo clock'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Launch RViz when true'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,

        slam_params_file_arg,
        slam_node,

        rviz2_node
    ])