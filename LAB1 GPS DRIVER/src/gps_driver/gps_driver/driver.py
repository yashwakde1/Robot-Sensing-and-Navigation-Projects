#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from math import sin, pi
from std_msgs.msg import Float64
from std_msgs.msg import Header
from std_msgs.msg import Float64
from rclpy.parameter import Parameter
from msg_package.msg import GPSmsg
import utm
from builtin_interfaces.msg import Time
from datetime import datetime, timedelta

def convert_to_decimal_degrees_latitude(degrees_minutes, direction):
    if not degrees_minutes or len(degrees_minutes) < 4:
        return 0.0
    degrees = float(degrees_minutes[:2])
    minutes = float(degrees_minutes[2:])
    decimal_degrees = degrees + (minutes / 60.0)
    if direction == 'S':
        decimal_degrees *= -1
    return decimal_degrees

def convert_to_decimal_degrees_longitude(degrees_minutes, direction):
    if not degrees_minutes or len(degrees_minutes) < 5:
        return 0.0
    degrees = float(degrees_minutes[:3])
    minutes = float(degrees_minutes[3:])
    decimal_degrees = degrees + (minutes / 60.0)
    if direction == 'W':
       decimal_degrees *= -1
    return decimal_degrees

class GPSPublisher(Node):
    def __init__(self, serial_port):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(GPSmsg, '/gps', 10)
        self.serial_port = serial_port
        self.serial_connection = self.open_serial_connection(self.serial_port)
        self.timer = self.create_timer(0.1, self.publish_gps_data)

    def open_serial_connection(self, serial_port):
        try:
            ser = serial.Serial(serial_port, baudrate=4800, timeout=1)
            self.get_logger().info(f'Successfully connected to {serial_port}')
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {serial_port}: {e}')
            return None

    def publish_gps_data(self):
        if self.serial_connection is None:
            return

        line = self.serial_connection.readline().decode('ascii', errors='replace').strip()
        

        if line.startswith('$GPGGA'):
            print("\n")
            print(line)

            temp_line = line.split(',')
            print(temp_line)
            gps_time_str = temp_line[1]
            gps_hour = int(gps_time_str[0:2])
            gps_minute = int(gps_time_str[2:4])
            gps_second = float(gps_time_str[4:])
            now = datetime.utcnow()
            gps_time = datetime(now.year, now.month, now.day, gps_hour, gps_minute, int(gps_second), int((gps_second % 1) * 1e6))

            time_since_epoch = (gps_time - datetime(1970, 1, 1, 0, 0, 0))  # UTC epoch

            gps_ros_time = Time()
            gps_ros_time.sec = int(time_since_epoch.total_seconds())
            gps_ros_time.nanosec = int((time_since_epoch.total_seconds() % 1) * 1e9)

            print(gps_ros_time)
            
            gps_msg = GPSmsg()
            data = line.split(',')

            if len(data) < 15:
                self.get_logger().warn('Not enough data fields in GPS line.')
                return

            latitude_str = data[2]
            latitude_dir = data[3]
            longitude_str = data[4]
            longitude_dir = data[5]
            
            # Check for valid altitude data and handle cases where it's missing
            try:
                altitude = float(data[9]) if data[9] else None
            except ValueError:
                self.get_logger().warn('Invalid altitude value. Skipping this message.')
                altitude = None

            latitude = convert_to_decimal_degrees_latitude(latitude_str, latitude_dir)
            longitude = convert_to_decimal_degrees_longitude(longitude_str, longitude_dir)

            # Convert lat/long to UTM coordinates
            utm_data = utm.from_latlon(latitude, longitude)
            utm_easting = utm_data[0]
            utm_northing = utm_data[1]
            utm_zone = utm_data[2]
            utm_letter = utm_data[3]

            gps_msg.header = Header()
            gps_msg.header.frame_id = 'GPS1_Frame'
            gps_msg.header.stamp = gps_ros_time#self.get_clock().now().to_msg()
           
           

            gps_msg.latitude = latitude
            gps_msg.longitude = longitude
            gps_msg.altitude = altitude if altitude is not None else 0.0  # Set a default value if altitude is missing
            gps_msg.utm_easting = utm_easting
            gps_msg.utm_northing = utm_northing
            gps_msg.zone = utm_zone
            gps_msg.letter = utm_letter  # Convert the UTM letter to ASCII integer

            self.publisher_.publish(gps_msg)
            self.get_logger().info(f'Published GPS data: {gps_msg}')

def main(args=None):
    rclpy.init(args=args)

    # Create node
    node = GPSPublisher('/dev/pts/3')  # Use a default value for testing
    
    # # Retrieve serial port parameter from launch file
    # serial_port = node.get_parameter('port').get_parameter_value().string_value
    
    # if serial_port:
    #     node.serial_port = serial_port  # Set the serial port passed from the launch file

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('GPS publisher interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
