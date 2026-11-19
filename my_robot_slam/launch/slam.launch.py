"""Start slam_toolbox in online async mapping mode with this package's config.

Thin wrapper around slam_toolbox's own online_async_launch.py, so the robot
always uses config/mapper_params_online_async.yaml from my_robot_slam.
Works on ROS 2 Humble and Jazzy (only arguments common to both are passed).

SLAM needs a sensor_msgs/LaserScan on /scan and an odom -> base_footprint TF;
it waits until both are available.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Build the launch description for SLAM.

    Declares the launch arguments and includes slam_toolbox's
    online_async_launch.py with the chosen parameter file.

    Launch arguments:
        use_sim_time (str): 'true' to use the /clock topic (simulation or
            bag playback), 'false' for the real robot. Default 'false'.
        slam_params_file (str): absolute path to a slam_toolbox YAML.
            Default: this package's installed
            config/mapper_params_online_async.yaml. Lets you try tuned
            parameters without rebuilding.

    Returns:
        LaunchDescription: the argument declarations plus the included
            slam_toolbox launch file.

    Raises:
        PackageNotFoundError: if slam_toolbox or my_robot_slam is not
            installed or the workspace is not sourced.
    """
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    default_params_file = os.path.join(
        get_package_share_directory('my_robot_slam'),
        'config', 'mapper_params_online_async.yaml')
    slam_toolbox_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch', 'online_async_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (/clock) time if true'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_params_file,
            description='Full path to the slam_toolbox parameter file'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time': use_sim_time,
            }.items()),
    ])
