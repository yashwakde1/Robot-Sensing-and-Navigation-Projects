from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    port = LaunchConfiguration('port')

    # Argument to set the serial port
    gps_puck_port_launch_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/pts/3'  # The default value can be changed based on your actual setup
    )
    
    GPS_data = Node(
        package="gps_driver",
        # Set a proper namespace
        namespace="gps_namespace",
        executable="driver",
        name="GPS_Driver",
        parameters=[{'port': port}],  # Use port as a parameter, not as a namespace
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        gps_puck_port_launch_arg,
        GPS_data,
    ])