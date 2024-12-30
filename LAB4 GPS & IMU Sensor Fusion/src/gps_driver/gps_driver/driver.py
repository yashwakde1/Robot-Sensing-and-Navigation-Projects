#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import typing
import serial
from math import sin, pi
from std_msgs.msg import Float64
from std_msgs.msg import Header
from std_msgs.msg import Float64
from rclpy.parameter import Parameter
from gps_msg.msg import GPSmsg
import utm
from builtin_interfaces.msg import Time
from datetime import datetime, timedelta

def GPS_to_UTM(latitude,longitude):

    utm_conv = utm.from_latlon(latitude,longitude)

    return utm_conv

def lat_long_conversion(input_degrees, direction, lt_lg):
    if (lt_lg == "lt" and (not input_degrees or len(input_degrees) < 4)) or (lt_lg == "lG" and (not input_degrees or len(input_degrees) < 5)):
        return 0.0
    
    if lt_lg == "lt":
        degrees = float(input_degrees[:2])
        minutes = float(input_degrees[2:])
        decimal_degrees = degrees + (minutes / 60.0)
        if direction == 'S':
            decimal_degrees *= -1
    else:
        degrees = float(input_degrees[:3])
        minutes = float(input_degrees[3:])
        decimal_degrees = degrees + (minutes / 60.0)
        if direction == 'W':
            decimal_degrees *= -1

    return decimal_degrees


class GPS_test_node(Node):
    def __init__(self):
        super().__init__('GPS_test_node')
        
        self.declare_parameter('port', rclpy.Parameter.Type.STRING)
        self.get_logger().info('Hi from GPS Driver.')
        
        port_name = self.get_parameter('port').value #.get_parameter_value().string_value
        self.get_logger().info(str(port_name))

        self.declare_parameter('baudrate', 4800)
        
        serial_baud = self.get_parameter('baudrate').value
        

        self.port = serial.Serial(port_name, serial_baud, timeout=3.0) # main line to read serial port
        self.serial_port = port_name

        #self.serial_port = self.port
        #self.serial_connection = self.open_serial_connection(self.serial_port)
        
        self.get_logger().debug("Using GPS sensor on port " + port_name + " at " + str(serial_baud))
        self.get_logger().debug("Initializing sensor.....")
        self.get_logger().info("Using GPS sensor on port " + port_name + " at " + str(serial_baud))
        self.get_logger().info("Initializing sensor.....")

        #self.port.write('*0100EW*0100PR=' + str(sampling_count) + '\r\n'.encode())
        
        #self.port.readline()


        self.publisher_ = self.create_publisher(
            msg_type=GPSmsg, 
            topic='/gps',
            qos_profile=10)
        
        timer_period = 0.1  # seconds or t = 1/f, we need 10 Hz here

        self.get_logger().debug("Initialization complete")
        self.get_logger().info("Initialization complete")
        self.get_logger().info("Publishing GPS Data.")

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
         
    def timer_callback(self):
        #if self.serial_connection is None:
        #    return
        
        global Lat, Long, Alt, Gps_time, gps_ros_time
        msg = String()
        gps_ros_time = self.get_clock().now().to_msg()
        
        Gps_time = '0'

        #line = "$GPGGA,001925.012,42.360081,N,-71.058884,W,1,8,1.05,2.1,M,-33.8,M,,0000*44" 
        #line = self.port.readline().decode().strip()
        
        #line = self.port.readline().decode('utf-8').strip()

        #line = self.serial_connection.readline().decode('ascii', errors='replace').strip()
        line = self.port.readline().decode('ascii', errors='replace').strip()
        temp_line = line
        temp_line = temp_line.replace('$GPGGA,','')
        
        #if line.startswith("$GPGGA"):
        #    print("Great work!!!!!!!!!!!!!")
        #print(line)
        


        #if line == '':
        #    self.get_logger().warn("GPS: No data")

        if line.startswith("$GPGGA"):
            #print("\n")
            #print(line)

            temp_line_data = line.split(',')
            #print(temp_line_data)
            gps_time_str = temp_line_data[1]
            gps_hour = int(gps_time_str[0:2])
            gps_minute = int(gps_time_str[2:4])
            gps_second = float(gps_time_str[4:])
            now = datetime.utcnow()
            gps_time = datetime(now.year, now.month, now.day, gps_hour, gps_minute, int(gps_second), int((gps_second % 1) * 1e6))

            time_since_epoch = (gps_time - datetime(1970, 1, 1, 0, 0, 0))  # UTC epoch

            gps_ros_time = Time()
            gps_ros_time.sec = int(time_since_epoch.total_seconds())
            gps_ros_time.nanosec = int((time_since_epoch.total_seconds() % 1) * 1e9)

            #print(gps_ros_time)

            coma_pos = temp_line.find(',')
            Gps_time = temp_line[:coma_pos].strip()
            #print(Gps_time)
            temp_line = temp_line[coma_pos+1:].strip()

            gps_line_data= line.split(',')

            if len(gps_line_data) < 15:
                self.get_logger().warn('Not enough data fields in GPS line.')
                return
            
            Lat = gps_line_data[2]
            lat_nw = gps_line_data[3]
            Lat = lat_long_conversion(Lat,lat_nw,'lt')
            Long = gps_line_data[4]
            long_ew = gps_line_data[5]
            Long = lat_long_conversion(Long,long_ew,'lg')
            Alt = float(gps_line_data[9])

            GPS_msg = GPSmsg()
            GPS_msg.header = Header()
            GPS_msg.header.stamp = self.get_clock().now().to_msg() #gps_ros_time #self.get_clock().now().to_msg()
            GPS_msg.header.frame_id = "GPS1_Frame"

            utm_con = GPS_to_UTM(Lat,Long)
            
            GPS_msg.latitude = Lat
            GPS_msg.longitude = Long
            GPS_msg.altitude = Alt
            GPS_msg.utm_easting = utm_con[0]
            GPS_msg.utm_northing = utm_con[1]
            GPS_msg.zone = utm_con[2]
            GPS_msg.letter = str(utm_con[3])
            GPS_msg.raw_data = line
            print("\n")
            print(GPS_msg)

            #GPS_msg.name = Alt
            self.publisher_.publish(GPS_msg)
            self.i += 1

        else:
            pass
            return
            '''
            #print("NO $GPGGA line found on the data stream..!!")
            try:
                Lat = Lat
                Long = Long
                Alt = Alt
                GPS_msg = GPSmsg()
                GPS_msg.header = Header()
                GPS_msg.header.stamp = gps_ros_time #self.get_clock().now().to_msg()
                GPS_msg.header.frame_id = "GPS1_Frame"

                utm_con = GPS_to_UTM(Lat,Long)
                
                GPS_msg.latitude = Lat
                GPS_msg.longitude = Long
                GPS_msg.altitude = Alt
                GPS_msg.utm_easting = utm_con[0]
                GPS_msg.utm_northing = utm_con[1]
                GPS_msg.zone = utm_con[2]
                GPS_msg.letter = str(utm_con[3])
                GPS_msg.raw_data = "NO $GPGGA line found on the data stream..!!"
                print("\n")
                print(GPS_msg)

                #GPS_msg.name = Alt
                self.publisher_.publish(GPS_msg)
                self.i += 1
            except:
                return
            '''    
                
            

        

def main(args=None):
    #port = args
    rclpy.init(args=args)
    
    gps_test_node = GPS_test_node()
    
    print('Hi from GPS Driver.')
    #node.get_logger().info('Hi from GPS Driver.')
    
    #serial_port = '/dev/ttyUSB0'  # Replace with your GPS module's serial port
    #gps_publisher = GPSPublisher(serial_port)
    
    try:
        rclpy.spin(gps_test_node)
    except KeyboardInterrupt:
        gps_test_node.get_logger().info('GPS publisher interrupted.')
    finally:
        gps_test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
