from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    imu_port = LaunchConfiguration('imu_port')
    gps_port = LaunchConfiguration('gps_port')
    
    IMU_data = Node(
        package="imu_driver",
        executable="driver",
        name="IMU_Driver",
        parameters = [{'port': imu_port}],
        output='screen',
        emulate_tty=True
    )
    GPS_data = Node(
        package="gps_driver",
        #namespace = port,
        executable="driver",
        name="GPS_Driver",
        parameters = [{'port': gps_port}],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        #gps_puck_port_launch_arg,
        IMU_data, GPS_data,
    ])
