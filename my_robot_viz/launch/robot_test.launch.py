"""Choose which test tool to start: the LiDAR viewer, the odometry test bench, or both.

    ros2 launch my_robot_viz robot_test.launch.py test:=lidar
    ros2 launch my_robot_viz robot_test.launch.py test:=odom
    ros2 launch my_robot_viz robot_test.launch.py test:=both

The LiDAR viewer is started through lidar_viz.launch.py (unchanged) on
lidar_port (default 8080). The odometry test bench (odom_test_server) runs on
odom_port (default 8081), so both can run at the same time.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the launch description for the selected test tool(s).

    Launch arguments:
        test (str): 'lidar', 'odom' or 'both'. Default 'both'.
        LiDAR (passed to lidar_viz.launch.py):
            start_driver (str): 'true' to also start the LD19 driver. Default 'false'.
            serial_port (str): LD19 serial device. Default '/dev/ttyUSB0'.
            scan_topic (str): LaserScan topic. Default '/scan'.
            lidar_port (str): LiDAR viewer web port, 1-65535. Default '8080'.
        Odometry (odom_test_server parameters):
            odom_topic (str): Odometry topic. Default '/odom'.
            cmd_vel_topic (str): Twist topic for test moves. Default '/cmd_vel'.
            allow_motion (str): 'true' to let tests drive the robot. Default 'false'.
            fake_odom (str): 'true' to simulate odometry from the commands.
                Default 'false'.
            wheel_radius (str): wheel radius in metres. Default '0.0325'.
            wheel_separation (str): left-right wheel distance in metres. Default '0.14'.
            odom_port (str): odometry test web port, 1-65535. Default '8081'.

    Returns:
        LaunchDescription: argument declarations, the conditional include of
            lidar_viz.launch.py and the conditional odom_test_server node.
    """
    test = LaunchConfiguration('test')
    arg = LaunchConfiguration

    def selected(names):
        # True when the 'test' argument is one of names, evaluated at launch time.
        return IfCondition(PythonExpression(["'", test, "' in ", repr(names)]))

    declared = [
        DeclareLaunchArgument('test', default_value='both', choices=['lidar', 'odom', 'both'],
                              description='Which test tool to start'),
        DeclareLaunchArgument('start_driver', default_value='false',
                              description='LiDAR: also start the LD19 driver'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='LiDAR: serial device of the LD19'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='LiDAR: LaserScan topic'),
        DeclareLaunchArgument('lidar_port', default_value='8080',
                              description='LiDAR: viewer web port'),
        DeclareLaunchArgument('odom_topic', default_value='/odom',
                              description='Odometry: nav_msgs/Odometry topic'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel',
                              description='Odometry: geometry_msgs/Twist topic for test moves'),
        DeclareLaunchArgument('allow_motion', default_value='false',
                              description='Odometry: let test moves drive the robot'),
        DeclareLaunchArgument('fake_odom', default_value='false',
                              description='Odometry: simulate odometry from the commands'),
        DeclareLaunchArgument('wheel_radius', default_value='0.0325',
                              description='Odometry: wheel radius in metres'),
        DeclareLaunchArgument('wheel_separation', default_value='0.14',
                              description='Odometry: left-right wheel distance in metres'),
        DeclareLaunchArgument('odom_port', default_value='8081',
                              description='Odometry: test page web port'),
    ]

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('my_robot_viz'), 'launch', 'lidar_viz.launch.py'])),
        launch_arguments={
            'start_driver': arg('start_driver'),
            'serial_port': arg('serial_port'),
            'scan_topic': arg('scan_topic'),
            'web_port': arg('lidar_port'),
        }.items(),
        condition=selected(('lidar', 'both')))

    odom = Node(
        package='my_robot_viz',
        executable='odom_test_server',
        name='odom_test_server',
        output='screen',
        condition=selected(('odom', 'both')),
        parameters=[{
            'odom_topic': arg('odom_topic'),
            'cmd_vel_topic': arg('cmd_vel_topic'),
            'allow_motion': ParameterValue(arg('allow_motion'), value_type=bool),
            'fake_odom': ParameterValue(arg('fake_odom'), value_type=bool),
            'wheel_radius': ParameterValue(arg('wheel_radius'), value_type=float),
            'wheel_separation': ParameterValue(arg('wheel_separation'), value_type=float),
            'port': ParameterValue(arg('odom_port'), value_type=int),
        }])

    return LaunchDescription(declared + [lidar, odom])
