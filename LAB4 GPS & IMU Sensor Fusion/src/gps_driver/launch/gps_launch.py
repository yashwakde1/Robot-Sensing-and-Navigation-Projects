from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    port = LaunchConfiguration('port')

    #gps_puck_port_launch_arg = DeclareLaunchArgument(
    #	'port',
    #	default_value = '/dev/ttyUSB0'
    #)
    
    GPS_data = Node(
        package="gps_driver",
        #namespace = port,
        executable="driver",
        name="GPS_Driver",
        parameters = [{'port': port}],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        #gps_puck_port_launch_arg,
        GPS_data,
    ])
