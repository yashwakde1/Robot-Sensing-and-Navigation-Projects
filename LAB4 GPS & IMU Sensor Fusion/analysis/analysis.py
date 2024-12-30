import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate, signal

# Load IMU data
imu_data = pd.read_csv('/home/pablo/analysis/imu_data_driving.csv')
# Load GPS data
gps_data = pd.read_csv('/home/pablo/analab4/gps_data_driving.csv')

# ---- Yaw Estimation and Magnetometer Calibration ----
# Rename 'yaw' column to 'imu_computed_yaw' for comparison
imu_data.rename(columns={'yaw': 'imu_computed_yaw'}, inplace=True)

# Extract magnetometer and gyro columns
mag_x = imu_data['mag_field_x'].values
mag_y = imu_data['mag_field_y'].values
gyro_z = imu_data['ang_vel_z'].values

# Gyro Bias Estimation and Correction
initial_samples = 100
gyro_bias = np.mean(gyro_z[:initial_samples])
gyro_z_corrected = gyro_z - gyro_bias

# Magnetometer Calibration: Hard-Iron Correction
mag_x_offset = np.mean(mag_x)
mag_y_offset = np.mean(mag_y)
corrected_mag_x = mag_x - mag_x_offset
corrected_mag_y = mag_y - mag_y_offset

# Soft-Iron Correction (scaling)
scale_x = (np.max(corrected_mag_x) - np.min(corrected_mag_x)) / 2
scale_y = (np.max(corrected_mag_y) - np.min(corrected_mag_y)) / 2
corrected_mag_x /= scale_x
corrected_mag_y /= scale_y

# Calculate yaw from corrected magnetometer data (in radians)
yaw_mag_corrected = np.arctan2(corrected_mag_y, corrected_mag_x)
yaw_mag_degrees = np.degrees(np.unwrap(yaw_mag_corrected))

# Integrate corrected gyro data to estimate yaw over time
time_step = 1 / 40
integrated_yaw = integrate.cumtrapz(gyro_z_corrected, dx=time_step, initial=0)
integrated_yaw_degrees = -np.degrees(np.unwrap(integrated_yaw))

# High-pass filter to remove low-frequency drift from integrated gyro yaw
cutoff_hp = 0.01
order_hp = 1
sampling_frequency_imu = 40.0
b_hp, a_hp = signal.butter(order_hp, cutoff_hp / (0.5 * sampling_frequency_imu), btype='highpass')
filtered_yaw_gyro = signal.filtfilt(b_hp, a_hp, integrated_yaw_degrees)

# Complementary Filter to blend magnetometer and gyro yaw estimates
alpha = 0.98
filtered_yaw_degrees = np.zeros_like(integrated_yaw_degrees)
filtered_yaw_degrees[0] = yaw_mag_degrees[0]

for i in range(1, len(filtered_yaw_degrees)):
    filtered_yaw_degrees[i] = (alpha * (filtered_yaw_degrees[i - 1] + gyro_z_corrected[i - 1] * time_step) +
                               (1 - alpha) * yaw_mag_degrees[i])

filtered_yaw_degrees_unwrapped = np.degrees(np.unwrap(np.radians(filtered_yaw_degrees)))

# Unwrap IMU-computed yaw
imu_computed_yaw = imu_data['imu_computed_yaw'].values
imu_computed_yaw_unwrapped = np.unwrap(imu_computed_yaw)
imu_computed_yaw_degrees = -np.degrees(imu_computed_yaw_unwrapped)

# ---- Velocity Estimation from IMU and GPS ----
# Extract and preprocess IMU linear acceleration and time
time_imu = imu_data['Time'].values
lin_acc_x = imu_data['lin_acc_x'].values
dt = np.diff(time_imu, prepend=time_imu[0])

# Integrate forward acceleration to estimate IMU velocity
velocity_estimate_acc = np.cumsum(lin_acc_x * dt)

# Calculate GPS velocity from position
gps_time = gps_data['Time'].values
gps_easting = gps_data['utm_easting'].values
gps_northing = gps_data['utm_northing'].values
gps_distances = np.sqrt(np.diff(gps_easting, prepend=gps_easting[0])**2 + np.diff(gps_northing, prepend=gps_northing[0])**2)
gps_time_diffs = np.diff(gps_time, prepend=gps_time[0])
gps_velocity = gps_distances / gps_time_diffs
gps_velocity_interp = np.interp(time_imu, gps_time, gps_velocity)

# Adjust IMU Velocity by removing Acceleration Bias
adjusted_lin_acc_x = lin_acc_x - np.mean(lin_acc_x)
adjusted_velocity_estimate_acc = np.cumsum(adjusted_lin_acc_x * dt)
adjusted_velocity_estimate_acc[gps_velocity_interp == 0] = 0

# ---- Trajectory Estimation with Alignment ----
heading_radians = np.radians(filtered_yaw_degrees)
ve = adjusted_velocity_estimate_acc * np.sin(heading_radians)
vn = adjusted_velocity_estimate_acc * np.cos(heading_radians)
xe = integrate.cumtrapz(ve, time_imu, initial=0)
xn = integrate.cumtrapz(vn, time_imu, initial=0)

# Align GPS with IMU starting point
gps_xe_aligned = gps_easting - gps_easting[0] + xe[0]
gps_xn_aligned = gps_northing - gps_northing[0] + xn[0]

# Initial heading correction to align trajectories
initial_heading_gps = np.arctan2(gps_xn_aligned[1] - gps_xn_aligned[0], gps_xe_aligned[1] - gps_xe_aligned[0])
initial_heading_imu = np.arctan2(xn[1] - xn[0], xe[1] - xe[0])
heading_offset = initial_heading_gps - initial_heading_imu
rotation_matrix = np.array([[np.cos(heading_offset), -np.sin(heading_offset)], [np.sin(heading_offset), np.cos(heading_offset)]])
imu_trajectory_aligned = np.dot(rotation_matrix, np.vstack((xe, xn)))

# ---- Plotting ----
# 1. Magnetometer and Integrated Gyro Yaw
plt.figure(figsize=(12, 4))
plt.plot(yaw_mag_degrees, label='Magnetometer Yaw (Corrected)', color='blue')
plt.plot(integrated_yaw_degrees, label='Integrated Gyro Yaw', color='orange')
plt.title("Magnetometer Yaw vs. Integrated Gyro Yaw")
plt.xlabel("Samples")
plt.ylabel("Yaw Angle (degrees)")
plt.legend()
plt.grid()
plt.show()

# 2. Comparison of Filtered Results (LPF, HPF, CF)
lpf_result = yaw_mag_degrees * alpha
hpf_result = filtered_yaw_gyro * alpha

plt.figure(figsize=(12, 4))
plt.plot(lpf_result, label='Low-Pass Filter (Magnetometer)', color='cyan')
plt.plot(hpf_result, label='High-Pass Filter (Gyro)', color='magenta')
plt.plot(filtered_yaw_degrees_unwrapped, label='Complementary Filter Result', color='green')
plt.title("Filter Results: LPF, HPF, and Complementary Filter")
plt.xlabel("Samples")
plt.ylabel("Yaw Angle (degrees)")
plt.legend()
plt.grid()
plt.show()

# 3. Complementary Filter vs. IMU Computed Yaw
plt.figure(figsize=(12, 4))
plt.plot(filtered_yaw_degrees_unwrapped, label='Complementary Filter Yaw', color='green')
plt.plot(imu_computed_yaw_degrees, label='IMU Computed Yaw', color='red')
plt.title("Complementary Filter Yaw vs. IMU Computed Yaw")
plt.xlabel("Samples")
plt.ylabel("Yaw Angle (degrees)")
plt.legend()
plt.grid()
plt.show()

# 4. Velocity Comparison from GPS and IMU
plt.figure(figsize=(12, 6))
plt.plot(time_imu, velocity_estimate_acc, label='IMU Velocity', color='blue')
plt.plot(time_imu, gps_velocity_interp, label='GPS Velocity', color='red', linestyle='--')
plt.title("Velocity from GPS and IMU")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.legend()
plt.grid()
plt.show()

# 5. Adjusted Velocity Comparison
plt.figure(figsize=(12, 6))
plt.plot(time_imu, adjusted_velocity_estimate_acc, label='IMU Velocity (Adjusted)', color='green')
plt.plot(time_imu, gps_velocity_interp, label='GPS Velocity', color='red', linestyle='--')
plt.title("Adjusted Velocity from GPS and IMU")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.legend()
plt.grid()
plt.show()

# 6. Trajectory Comparison
plt.figure(figsize=(10, 8))
plt.plot(gps_xe_aligned, gps_xn_aligned, label="GPS Track", color="blue")
plt.plot(imu_trajectory_aligned[0], imu_trajectory_aligned[1], label="IMU Estimated Track (Aligned)", color="green", linestyle="--")
plt.xlabel("Easting (m)")
plt.ylabel("Northing (m)")
plt.legend()
plt.title("Trajectory Comparison: GPS vs. IMU")
plt.grid()
plt.show()

# 7. ωẊ and ÿ_obs Comparison
omega_x_dot = gyro_z_corrected * velocity_estimate_acc
plt.figure(figsize=(10, 4))
plt.plot(omega_x_dot, label=r'$\omega Ẋ$', color='purple')
plt.plot(imu_data['lin_acc_y'].values, label=r'$ÿ_{obs}$', color='green', linestyle='--')
plt.title(r"Comparison of $\omega Ẋ$ and $ÿ_{obs}$")
plt.xlabel("Samples")
plt.ylabel("Acceleration (m/s²)")
plt.legend()
plt.grid()
plt.show()
