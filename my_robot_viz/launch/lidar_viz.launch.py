"""Start the web LiDAR viewer, and optionally the LD19 driver.

Open http://<pi-ip>:<web_port>/ in a browser on the same network.

The LD19 driver (ldlidar_stl_ros2) is not available from apt; clone it into
the workspace first (see README.md). Its own ld19.launch.py hard-codes the
serial port, so the driver node is started here directly with the same
parameters, which lets serial_port be overridden. The driver's
base_link -> base_laser static transform is not started here; the viewer
does not need TF, and the robot's TF tree comes in a later phase.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the launch description for the viewer (and optional driver).

    Launch arguments:
        start_driver (str): 'true' to also start the LD19 driver node.
            Default 'false' (use when /scan is already published).
        serial_port (str): LD19 serial device. Default '/dev/ttyUSB0'.
        scan_topic (str): LaserScan topic used by driver and viewer.
            Default '/scan'.
        web_port (str): TCP port of the viewer web page, 1-65535.
            Default '8080'.

    Returns:
        LaunchDescription: argument declarations, the optional LD19 driver
            node and the scan_web_server node.
    """
    start_driver = LaunchConfiguration('start_driver')
    serial_port = LaunchConfiguration('serial_port')
    scan_topic = LaunchConfiguration('scan_topic')
    web_port = LaunchConfiguration('web_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_driver', default_value='false',
            description='Also start the LD19 driver (ldlidar_stl_ros2) if true'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0',
            description='Serial device of the LD19'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='LaserScan topic to publish (driver) and show (viewer)'),
        DeclareLaunchArgument(
            'web_port', default_value='8080',
            description='TCP port of the viewer web page'),

        # Parameters copied from ldlidar_stl_ros2's launch/ld19.launch.py.
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='LD19',
            output='screen',
            condition=IfCondition(start_driver),
            parameters=[{
                'product_name': 'LDLiDAR_LD19',
                'topic_name': scan_topic,
                'frame_id': 'base_laser',
                'port_name': serial_port,
                'port_baudrate': 230400,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
                'angle_crop_min': 135.0,
                'angle_crop_max': 225.0,
            }]),

        Node(
            package='my_robot_viz',
            executable='scan_web_server',
            name='scan_web_server',
            output='screen',
            parameters=[{
                'scan_topic': scan_topic,
                'port': ParameterValue(web_port, value_type=int),
            }]),
    ])
