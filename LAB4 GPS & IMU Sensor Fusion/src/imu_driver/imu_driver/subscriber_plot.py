#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from imu_msg.msg import IMUmsg
import matplotlib.pyplot as plt
import numpy as np
import math
import csv  # Import the CSV module

class IMUDataSubscriber(Node):
    def __init__(self):
        super().__init__('imu_data_subscriber')
        
        # Create subscriber to /imu topic
        self.subscription = self.create_subscription(
            IMUmsg,
            '/imu',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Initialize storage for plotting
        self.euler_data = []
        self.angular_velocity_data = []
        self.linear_acceleration_data = []
        self.magnetometer_data = []  # Add magnetometer data storage
        self.time_stamps = []

        # Set up matplotlib with 4 subplots (adding magnetometer plot)
        self.fig, self.axs = plt.subplots(4, 1, figsize=(10, 10))

    def quaternion_to_euler(self, x, y, z, w):
        # Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

        return roll, pitch, yaw

    def listener_callback(self, imu_msg):
        # Extract data from the message
        timestamp = self.get_clock().now().nanoseconds * 1e-9  # Convert nanoseconds to seconds
        self.time_stamps.append(timestamp)

        # Convert quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(
            imu_msg.imu.orientation.x,
            imu_msg.imu.orientation.y,
            imu_msg.imu.orientation.z,
            imu_msg.imu.orientation.w
        )

        # Store the Euler angles (roll, pitch, yaw)
        self.euler_data.append([roll, pitch, yaw])

        # Store angular velocity
        self.angular_velocity_data.append([
            imu_msg.imu.angular_velocity.x,
            imu_msg.imu.angular_velocity.y,
            imu_msg.imu.angular_velocity.z
        ])

        # Store linear acceleration
        self.linear_acceleration_data.append([
            imu_msg.imu.linear_acceleration.x,
            imu_msg.imu.linear_acceleration.y,
            imu_msg.imu.linear_acceleration.z
        ])

        # Store magnetometer data
        self.magnetometer_data.append([
            imu_msg.mag_field.magnetic_field.x * 10**-4,
            imu_msg.mag_field.magnetic_field.y * 10**-4,
            imu_msg.mag_field.magnetic_field.z * 10**-4
        ])

    def update_plots(self):
        time = np.array(self.time_stamps) - self.time_stamps[0]  # Time starts from zero

        # Plot Euler Angles (Roll, Pitch, Yaw)
        euler_angles = np.array(self.euler_data)
        euler_mean = np.mean(euler_angles, axis=0)
        euler_std = np.std(euler_angles, axis=0)

        self.axs[0].plot(time, np.rad2deg(euler_angles[:, 0]), label='Roll (deg)')
        self.axs[0].plot(time, np.rad2deg(euler_angles[:, 1]), label='Pitch (deg)')
        self.axs[0].plot(time, np.rad2deg(euler_angles[:, 2]), label='Yaw (deg)')
        self.axs[0].axhline(np.rad2deg(euler_mean[0]), color='r', linestyle='--', label='Mean Roll')
        self.axs[0].axhline(np.rad2deg(euler_mean[1]), color='g', linestyle='--', label='Mean Pitch')
        self.axs[0].axhline(np.rad2deg(euler_mean[2]), color='b', linestyle='--', label='Mean Yaw')
        self.axs[0].fill_between(time, np.rad2deg(euler_mean[0] - euler_std[0]), np.rad2deg(euler_mean[0] + euler_std[0]), color='r', alpha=0.2)
        self.axs[0].fill_between(time, np.rad2deg(euler_mean[1] - euler_std[1]), np.rad2deg(euler_mean[1] + euler_std[1]), color='g', alpha=0.2)
        self.axs[0].fill_between(time, np.rad2deg(euler_mean[2] - euler_std[2]), np.rad2deg(euler_mean[2] + euler_std[2]), color='b', alpha=0.2)
        self.axs[0].set_title('IMU Orientation (Euler Angles)')
        self.axs[0].set_xlabel('Time [s]')
        self.axs[0].set_ylabel('Angle [deg]')
        self.axs[0].legend()

        # Plot Angular Velocity
        angular_velocity = np.array(self.angular_velocity_data)
        angular_mean = np.mean(angular_velocity, axis=0)
        angular_std = np.std(angular_velocity, axis=0)

        self.axs[1].plot(time, angular_velocity[:, 0], label='Angular Velocity X')
        self.axs[1].plot(time, angular_velocity[:, 1], label='Angular Velocity Y')
        self.axs[1].plot(time, angular_velocity[:, 2], label='Angular Velocity Z')
        self.axs[1].axhline(angular_mean[0], color='r', linestyle='--', label='Mean X')
        self.axs[1].axhline(angular_mean[1], color='g', linestyle='--', label='Mean Y')
        self.axs[1].axhline(angular_mean[2], color='b', linestyle='--', label='Mean Z')
        self.axs[1].fill_between(time, angular_mean[0] - angular_std[0], angular_mean[0] + angular_std[0], color='r', alpha=0.2)
        self.axs[1].fill_between(time, angular_mean[1] - angular_std[1], angular_mean[1] + angular_std[1], color='g', alpha=0.2)
        self.axs[1].fill_between(time, angular_mean[2] - angular_std[2], angular_mean[2] + angular_std[2], color='b', alpha=0.2)
        self.axs[1].set_title('IMU Angular Velocity')
        self.axs[1].set_xlabel('Time [s]')
        self.axs[1].set_ylabel('Angular Velocity [rad/s]')
        self.axs[1].legend()

        # Plot Linear Acceleration
        linear_acceleration = np.array(self.linear_acceleration_data)
        acc_mean = np.mean(linear_acceleration, axis=0)
        acc_std = np.std(linear_acceleration, axis=0)

        self.axs[2].plot(time, linear_acceleration[:, 0], label='Acceleration X')
        self.axs[2].plot(time, linear_acceleration[:, 1], label='Acceleration Y')
        self.axs[2].plot(time, linear_acceleration[:, 2], label='Acceleration Z')
        self.axs[2].axhline(acc_mean[0], color='r', linestyle='--', label='Mean X')
        self.axs[2].axhline(acc_mean[1], color='g', linestyle='--', label='Mean Y')
        self.axs[2].axhline(acc_mean[2], color='b', linestyle='--', label='Mean Z')
        self.axs[2].fill_between(time, acc_mean[0] - acc_std[0], acc_mean[0] + acc_std[0], color='r', alpha=0.2)
        self.axs[2].fill_between(time, acc_mean[1] - acc_std[1], acc_mean[1] + acc_std[1], color='g', alpha=0.2)
        self.axs[2].fill_between(time, acc_mean[2] - acc_std[2], acc_mean[2] + acc_std[2], color='b', alpha=0.2)
        self.axs[2].set_title('IMU Linear Acceleration')
        self.axs[2].set_xlabel('Time [s]')
        self.axs[2].set_ylabel('Acceleration [m/s^2]')
        self.axs[2].legend()

        # Plot Magnetometer Data
        magnetometer = np.array(self.magnetometer_data)
        mag_mean = np.mean(magnetometer, axis=0)
        mag_std = np.std(magnetometer, axis=0)

        self.axs[3].plot(time, magnetometer[:, 0], label='Magnetometer X')
        self.axs[3].plot(time, magnetometer[:, 1], label='Magnetometer Y')
        self.axs[3].plot(time, magnetometer[:, 2], label='Magnetometer Z')
        self.axs[3].axhline(mag_mean[0], color='r', linestyle='--', label='Mean X')
        self.axs[3].axhline(mag_mean[1], color='g', linestyle='--', label='Mean Y')
        self.axs[3].axhline(mag_mean[2], color='b', linestyle='--', label='Mean Z')
        self.axs[3].fill_between(time, mag_mean[0] - mag_std[0], mag_mean[0] + mag_std[0], color='r', alpha=0.2)
        self.axs[3].fill_between(time, mag_mean[1] - mag_std[1], mag_mean[1] + mag_std[1], color='g', alpha=0.2)
        self.axs[3].fill_between(time, mag_mean[2] - mag_std[2], mag_mean[2] + mag_std[2], color='b', alpha=0.2)
        self.axs[3].set_title('IMU Magnetometer Data')
        self.axs[3].set_xlabel('Time [s]')
        self.axs[3].set_ylabel('Magnetometer [T]')
        self.axs[3].legend()

        plt.tight_layout()
        plt.show()

    def save_to_csv(self, filename='imu_data.csv'):
        # Save IMU data and statistics to CSV
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time', 'Roll', 'Pitch', 'Yaw', 'Angular Velocity X', 'Angular Velocity Y', 'Angular Velocity Z', 'Acceleration X', 'Acceleration Y', 'Acceleration Z', 'Magnetometer X', 'Magnetometer Y', 'Magnetometer Z'])
            for i in range(len(self.time_stamps)):
                writer.writerow([
                    self.time_stamps[i],
                    self.euler_data[i][0],
                    self.euler_data[i][1],
                    self.euler_data[i][2],
                    self.angular_velocity_data[i][0],
                    self.angular_velocity_data[i][1],
                    self.angular_velocity_data[i][2],
                    self.linear_acceleration_data[i][0],
                    self.linear_acceleration_data[i][1],
                    self.linear_acceleration_data[i][2],
                    self.magnetometer_data[i][0],
                    self.magnetometer_data[i][1],
                    self.magnetometer_data[i][2]
                ])
            # Write means and standard deviations
            writer.writerow(['Mean', np.mean(self.euler_data, axis=0)[0], np.mean(self.euler_data, axis=0)[1], np.mean(self.euler_data, axis=0)[2],
                             np.mean(self.angular_velocity_data, axis=0)[0], np.mean(self.angular_velocity_data, axis=0)[1], np.mean(self.angular_velocity_data, axis=0)[2],
                             np.mean(self.linear_acceleration_data, axis=0)[0], np.mean(self.linear_acceleration_data, axis=0)[1], np.mean(self.linear_acceleration_data, axis=0)[2],
                             np.mean(self.magnetometer_data, axis=0)[0], np.mean(self.magnetometer_data, axis=0)[1], np.mean(self.magnetometer_data, axis=0)[2]])
            writer.writerow(['StdDev', np.std(self.euler_data, axis=0)[0], np.std(self.euler_data, axis=0)[1], np.std(self.euler_data, axis=0)[2],
                             np.std(self.angular_velocity_data, axis=0)[0], np.std(self.angular_velocity_data, axis=0)[1], np.std(self.angular_velocity_data, axis=0)[2],
                             np.std(self.linear_acceleration_data, axis=0)[0], np.std(self.linear_acceleration_data, axis=0)[1], np.std(self.linear_acceleration_data, axis=0)[2],
                             np.std(self.magnetometer_data, axis=0)[0], np.std(self.magnetometer_data, axis=0)[1], np.std(self.magnetometer_data, axis=0)[2]])

def main(args=None):
    rclpy.init(args=args)
    imu_data_subscriber = IMUDataSubscriber()

    try:
        rclpy.spin(imu_data_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        imu_data_subscriber.update_plots()
        imu_data_subscriber.save_to_csv('imu_data.csv')  # Save data before exiting
        imu_data_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
