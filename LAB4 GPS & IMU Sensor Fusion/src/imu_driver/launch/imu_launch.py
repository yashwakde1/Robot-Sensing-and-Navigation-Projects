from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    port = LaunchConfiguration('port')
    
    IMU_data = Node(
        package="imu_driver",
        executable="driver",
        name="IMU_Driver",
        parameters = [{'port': port}],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        #gps_puck_port_launch_arg,
        IMU_data,
    ])
