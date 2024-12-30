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
from imu_msg.msg import IMUmsg
import utm
from builtin_interfaces.msg import Time
from datetime import datetime, timedelta
from geometry_msgs.msg import Quaternion
import math
from sensor_msgs.msg import Imu, MagneticField
import time



class IMU_node(Node):      

    def __init__(self):
        super().__init__('IMU_node')
        
        self.declare_parameter('port', rclpy.Parameter.Type.STRING)
        self.get_logger().info('Hi from IMU Driver.')
        
        port_name = self.get_parameter('port').value #.get_parameter_value().string_value
        self.get_logger().info(str(port_name))

        self.declare_parameter('baudrate', 115200)
        
        serial_baud = self.get_parameter('baudrate').value
        

        self.port = serial.Serial(port_name, serial_baud, timeout=3.0) # main line to read serial port
        self.serial_port = port_name

        #self.serial_port = self.port
        #self.serial_connection = self.open_serial_connection(self.serial_port)
        
        self.get_logger().debug("Using IMU sensor on port " + port_name + " at " + str(serial_baud))
        self.get_logger().debug("Initializing sensor.....")
        self.get_logger().info("Using IMU sensor on port " + port_name + " at " + str(serial_baud))
        self.get_logger().info("Initializing sensor.....")

        #self.port.write('*0100EW*0100PR=' + str(sampling_count) + '\r\n'.encode())
        
        #self.port.readline()


        self.publisher_ = self.create_publisher(
            msg_type=IMUmsg, 
            topic='/imu',
            qos_profile=10)
        
        timer_period = 0.01 #0.025  # seconds or t = 1/f, we need 40 Hz here

        '''
        data_string = "$VNWRG,07,40*59"#"$VNRRG,5*46" #"$VNWRG,07,40*59"  
        data_string_pause = "$VNASY,0*XX"
        data_string_play = "$VNASY,1*XX"

        data_bytes = data_string_pause.encode('ascii')
        self.port.write(data_bytes)
        print(f"Sent: {data_string_pause}")
        received_data = self.port.readline()  # Read data from serial (up to newline character)
        time.sleep(1)
        if received_data:
            decoded_data = received_data.decode('ascii').strip()
            print(f"Received: {decoded_data}")

        else:
            print("No data received.")
        time.sleep(1)
        received_data = self.port.readline()  # Read data from serial (up to newline character)
        if received_data:
            decoded_data = received_data.decode('ascii').strip()
            print(f"Received: {decoded_data}")

        else:
            print("No data received.")
        time.sleep(1)

        data_bytes = data_string.encode('ascii')
        self.port.write(data_bytes)
        print(f"Sent: {data_string}")
        time.sleep(1)
        received_data = self.port.readline()  # Read data from serial (up to newline character)
        if received_data:
            decoded_data = received_data.decode('ascii').strip()
            print(f"Received: {decoded_data}")

        else:
            print("No data received.")


        time.sleep(1)
        """Read data from the serial port."""
        received_data = self.port.readline()  # Read data from serial (up to newline character)
        if received_data:
            decoded_data = received_data.decode('ascii').strip()
            print(f"Received: {decoded_data}")

        else:
            print("No data received.")
        time.sleep(3)

        data_bytes = data_string_play.encode('ascii')
        self.port.write(data_bytes)
        print(f"Sent: {data_string_play}")
        decoded_data = received_data.decode('ascii').strip()
        print(f"Received: {decoded_data}")
        time.sleep(7)
	    '''

        self.get_logger().debug("Initialization complete")
        self.get_logger().info("Initialization complete")
        self.get_logger().info("Publishing IMU Data.")

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        
    def euler_to_quaternion(self, roll, pitch, yaw):
        """ Convert Euler angles to quaternion. """
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
    
    

    def timer_callback(self):


        #if self.serial_connection is None:
        #    return

        #line = "$VNYMR,-165.980,-037.300,+001.245,+00.2893,+00.0748,+00.7405,-05.970,-00.145,-07.838,+00.001519,+00.000095,+00.000844*64" 
        #                 YAW     PITCH     ROLL     MagX     MagY     MagZ    accelX  accelY  accelZ   GyroX       GyroY       GyroZ

        #line = self.port.readline().decode().strip()
        #line = self.port.readline().decode('utf-8').strip()
        #line = self.serial_connection.readline().decode('ascii', errors='replace').strip()

        line = self.port.readline().decode('ascii', errors='replace').strip()

        
        #if line.startswith("$VNYMR"):
        #    print("Great work!!!!!!!!!!!!!")
        #print(line)

        #if line == '':
        #    self.get_logger().warn("IMU: No data")

        if line.startswith("$VNYMR"):
            #print("\n")
            #print(line)
            
            imu_line_data= line.split(',')

            if len(imu_line_data) < 13:
                self.get_logger().warn('Not enough data fields in IMU "$VNYMR" line.')
                return
            try:
                Yaw = float(imu_line_data[1])
                Pitch = float(imu_line_data[2])
                Roll = float(imu_line_data[3])
                MagX = float(imu_line_data[4]) * 10**(-4)
                MagY = float(imu_line_data[5]) * 10**(-4)
                MagZ = float(imu_line_data[6]) * 10**(-4)
                AccelX = float(imu_line_data[7])
                AccelY = float(imu_line_data[8])
                AccelZ = float(imu_line_data[9])
                GyroX = float(imu_line_data[10])
                GyroY = float(imu_line_data[11])
                GyroZ = float(imu_line_data[12].split('*')[0])
                #print(GyroZ)


                IMU_msg = IMUmsg()
                IMU_msg.header = Header()
                IMU_msg.header.stamp = self.get_clock().now().to_msg()
                IMU_msg.header.frame_id = "IMU1_Frame"

                IMU_msg.imu.header.stamp = self.get_clock().now().to_msg()
                IMU_msg.imu.header.frame_id = "IMU1_Frame"

                IMU_msg.mag_field.header.stamp = self.get_clock().now().to_msg()
                IMU_msg.mag_field.header.frame_id = "IMU1_Frame"

                IMU_msg.imu.linear_acceleration.x = AccelX
                IMU_msg.imu.linear_acceleration.y = AccelY
                IMU_msg.imu.linear_acceleration.z = AccelZ

                IMU_msg.imu.angular_velocity.x = GyroX
                IMU_msg.imu.angular_velocity.y = GyroY
                IMU_msg.imu.angular_velocity.z = GyroZ

                q = self.euler_to_quaternion(Roll, Pitch, Yaw)
                IMU_msg.imu.orientation = q

                IMU_msg.mag_field.magnetic_field.x = MagX
                IMU_msg.mag_field.magnetic_field.y = MagY
                IMU_msg.mag_field.magnetic_field.z = MagZ

                IMU_msg.raw_imu = line

                #print("\n")
                print(IMU_msg)

                self.publisher_.publish(IMU_msg)
                self.i += 1

                       
            except ValueError as e:
                self.get_logger().error(f"Error parsing IMU data: {e}")
        else:
            self.get_logger().warn("Received non-IMU string data.")        
                
                
        

def main(args=None):

    rclpy.init(args=args)
    
    imu_test_node = IMU_node()
    
    print('Hi from IMU Driver.')
    

    try:
        rclpy.spin(imu_test_node)
    except KeyboardInterrupt:
        imu_test_node.get_logger().info('IMU publisher interrupted.')
    finally:
        imu_test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
