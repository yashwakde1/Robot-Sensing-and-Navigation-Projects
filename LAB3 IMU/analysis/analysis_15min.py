import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load CSV data
file_path = '/home/pablo/analysis/i mu/imu_raw_data.csv'  # Update this path to your CSV file
data = pd.read_csv(file_path)

# Initial columns in the CSV
print("Initial columns in the CSV:", data.columns.tolist())

# Clean column names
data.columns = data.columns.str.strip()  # Remove leading and trailing whitespace

# Update required columns based on your actual data
required_columns = [
    'Yaw', 
    'Pitch', 
    'Roll',  
    'Magnetic Field X', 
    'Magnetic Field Y', 
    'Magnetic Field Z', 
    'Linear Acceleration X', 
    'Linear Acceleration Y', 
    'Linear Acceleration z', 
    'Angular Velocity X', 
    'Angular Velocity Y', 
    'Angular Velocity Z'
]

# Check for required columns
if not all(col in data.columns for col in required_columns):
    raise ValueError("Required columns not found in the data. Found columns: " + str(data.columns.tolist()))

# Function to convert to float and handle non-numeric values
def convert_to_float(series):
    # Convert to string first to use .str accessor, then replace non-numeric characters
    return pd.to_numeric(series.astype(str).str.replace(r'[^0-9\.\+\-eE]', '', regex=True), errors='coerce')

# Convert columns to float
for col in required_columns:
    data[col] = convert_to_float(data[col])

# Check for any conversion issues
if data[required_columns].isnull().any().any():
    print("Warning: Some values could not be converted to float and were replaced with NaN.")

print("Cleaned columns in the CSV:", data.columns.tolist())

# Data types after conversion
print("Data types after conversion:\n", data.dtypes)

# Convert Magnetic Field from Gauss to Tesla
data['Magnetic Field X'] *= 1e-4
data['Magnetic Field Y'] *= 1e-4
data['Magnetic Field Z'] *= 1e-4

# Function to plot time series data and print mean & standard deviation
def plot_data(data):
    plt.figure(figsize=(15, 10))
    
    # Create a time array, adjust the length of time based on the number of rows in the data
    time = np.arange(len(data)) / 40  # Assuming 40 Hz frequency, adjust if needed
    
    for idx, col in enumerate(required_columns):
        plt.subplot(3, 4, idx + 1)
        
        # Convert both time and data columns to numpy arrays
        time_np = time  # Already a numpy array
        data_np = data[col].to_numpy()  # Convert the data column to a numpy array
        
        plt.plot(time_np, data_np, label=col, alpha=0.7)
        plt.title(f'{col} vs Time')
        plt.xlabel('Time (s)')
        plt.ylabel(col)
        plt.grid(True)
        
        # Calculate mean and standard deviation
        mean = data_np.mean()
        std_dev = data_np.std()
        
        # Plot mean and standard deviation
        # plt.axhline(mean, color='r', linestyle='--', label='Mean')
        # plt.axhline(mean + std_dev, color='g', linestyle='--', label='Mean ± 1 Std Dev')
        # plt.axhline(mean - std_dev, color='g', linestyle='--')
        # plt.legend()
        
        # Print the mean and standard deviation
        print(f'{col} - Mean: {mean:.5f}, Std Dev: {std_dev:.5f}')

    plt.tight_layout()
    plt.show()

# Function to plot histograms
def plot_histograms(data):
    plt.figure(figsize=(15, 10))
    
    for idx, col in enumerate(required_columns):
        plt.subplot(3, 4, idx + 1)
        plt.hist(data[col].dropna(), bins=30, alpha=0.7, color='blue', edgecolor='black')
        plt.title(f'Histogram of {col}')
        plt.xlabel(col)
        plt.ylabel('Frequency')
        plt.grid(True)

    plt.tight_layout()
    plt.show()

# Function to plot error distributions
def plot_error_distributions(data):
    plt.figure(figsize=(15, 10))
    
    for idx, col in enumerate(required_columns):
        errors = data[col] - data[col].mean()  # Error as difference from mean
        plt.subplot(3, 4, idx + 1)
        plt.hist(errors.dropna(), bins=30, alpha=0.7, color='orange', edgecolor='black')
        plt.title(f'Error Distribution of {col}')
        plt.xlabel('Error')
        plt.ylabel('Frequency')
        plt.grid(True)

    plt.tight_layout()
    plt.show()

# Plotting the data
plot_data(data)
plot_histograms(data)
plot_error_distributions(data)
