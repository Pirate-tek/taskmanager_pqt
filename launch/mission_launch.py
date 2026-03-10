import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Directories
    pkg_gazebo = get_package_share_directory('dynominion_gazebo')
    pkg_navigation = get_package_share_directory('dynominion_navigation')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Simulation Launch
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'dynominion_gazebo.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Navigation Launch
    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'dynominion_nav_bringup.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    from launch.actions import TimerAction
    from launch_ros.actions import Node

    # Delay Navigation Launch
    delayed_navigation = TimerAction(
        period=15.0,
        actions=[launch_navigation]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        launch_gazebo,
        delayed_navigation
    ])
