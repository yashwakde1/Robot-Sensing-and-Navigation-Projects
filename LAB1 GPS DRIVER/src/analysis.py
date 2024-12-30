import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

# Load the CSV Data
df = pd.read_csv('/home/pablo/analysis/gps_data_walking.csv')  # Replace with your actual file path

# Check for missing values and drop rows with NaN in key columns
df = df.dropna(subset=['utm_easting', 'utm_northing'])

# Display the first few rows to inspect the data
print(df.head())

#  Plot the GPS Trajectory (using correct column names for UTM coordinates)
# Convert UTM columns to NumPy arrays
utm_easting = df['utm_easting'].to_numpy()
utm_northing = df['utm_northing'].to_numpy()

plt.figure()
plt.plot(utm_easting, utm_northing, marker='o', linestyle='-', color='b')
plt.xlabel('UTM Easting')
plt.ylabel('UTM Northing')
plt.title('GPS Trajectory - Walking')
plt.grid(True)
plt.show()

#  Fit a Straight Line to the Path and Compute Errors
# Fit linear regression model
X = utm_easting.reshape(-1, 1)  # Reshaping for sklearn
y = utm_northing

model = LinearRegression()
model.fit(X, y)

# Predicted northing values (on the straight line)
y_pred = model.predict(X)

# Calculate residuals (errors) as the deviation from the line
errors = y - y_pred

#  Plot the Errors
plt.figure()
plt.plot(errors, marker='o', linestyle='-', color='r')
plt.xlabel('')
plt.ylabel('Error (meters)')
plt.title('Error Deviation from Walking')
plt.grid(True)
plt.show()

#  Statistical Analysis of Errors
mean_error = np.mean(errors)
std_error = np.std(errors)
rmse_error = np.sqrt(np.mean(errors**2))

print(f"Mean Error: {mean_error:.2f} meters")
print(f"Standard Deviation of Error: {std_error:.2f} meters")
print(f"RMSE (Root Mean Square Error): {rmse_error:.2f} meters")

#  Plot Error Distribution (Histogram)
plt.figure()
plt.hist(errors, bins=30, color='g', alpha=0.7)
plt.xlabel('Error (meters)')
plt.ylabel('Frequency')
plt.title('Distribution of GPS Errors')
plt.grid(True)
plt.show()

#  Calculate Error Bounds
lower_bound = mean_error - 2 * std_error
upper_bound = mean_error + 2 * std_error

print(f"errors are expected to be between {lower_bound:.2f} and {upper_bound:.2f} meters.")
