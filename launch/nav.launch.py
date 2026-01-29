import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    uwnre_nav_dir = get_package_share_directory('uwnre_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')

    # Declare the launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(uwnre_nav_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # This pulls in the standard Nav2 lifecycle manager and servers
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false'
        }.items()
    )

    # RE-MAPPING: This is where the "piping" happens
    # We remap the generic 'cmd_vel' to the LUCI-specific topic
    # And ensure odom is coming from your base controller
    return LaunchDescription([
        declare_params_file_cmd,
        bringup_cmd
    ])