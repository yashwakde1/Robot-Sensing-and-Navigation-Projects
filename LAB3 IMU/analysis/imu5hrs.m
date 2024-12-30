% Load the CSV file with original column headers
filename = '/MATLAB Drive/imu/imu_data_5hrs.csv';  % Make sure your CSV file is in the same directory
data = readtable(filename, 'VariableNamingRule', 'preserve');  % Preserve original column headers

% Extract the time and IMU data from the table
time = data{:,1};  % Assuming the first column is time
imu_data = data{:, 2:end};  % All other columns are the IMU data

% Sampling frequency
Fs = 40;  % 40 Hz sampling rate
t0 = 1/Fs;

% List of IMU data labels
labels = {'Roll', 'Pitch', 'Yaw', ...
          'Angular Velocity X', 'Angular Velocity Y', 'Angular Velocity Z', ...
          'Linear Acceleration X', 'Linear Acceleration Y', 'Linear Acceleration Z', ...
          'Magnetometer X', 'Magnetometer Y', 'Magnetometer Z'};

% Number of data points
L = size(imu_data, 1);

% Pre-allocate cell arrays to store results for each signal
allan_deviation_results = cell(size(imu_data, 2), 1);
tau_values = cell(size(imu_data, 2), 1);

% Loop over each column (each IMU signal) to calculate Allan variance and plot them individually
for i = 1:size(imu_data, 2)
    signal = imu_data(:, i);  % Extract each signal (roll, pitch, etc.)
    theta = cumsum(signal, 1) * t0;  % Integrated signal (for Allan variance)

    % Define maximum number of m values
    maxNumM = 100;
    maxM = 2.^floor(log2(L/2));
    m = logspace(log10(1), log10(maxM), maxNumM).';
    m = ceil(m); % m must be an integer.
    m = unique(m); % Remove duplicates.

    tau = m * t0;  % Tau values

    % Compute Allan variance
    avar = zeros(numel(m), 1);
    for j = 1:numel(m)
        mi = m(j);
        avar(j,:) = sum( ...
            (theta(1+2*mi:L) - 2*theta(1+mi:L-mi) + theta(1:L-2*mi)).^2, 1);
    end
    avar = avar ./ (2 * tau.^2 .* (L - 2*m));
    adev = sqrt(avar);  % Allan deviation

    % Store results
    allan_deviation_results{i} = adev;
    tau_values{i} = tau;

    % Create a new figure for each signal and plot
    figure;  % Opens a new figure for each IMU signal

    % Plot Allan deviation for this signal
    loglog(tau, adev, 'LineWidth', 1.5);
    
    % Update titles for Linear Acceleration
    if contains(labels{i}, 'Acceleration')
        title(['Allan Deviation for Linear ' labels{i}], 'FontSize', 10);
    else
        title(['Allan Deviation for ' labels{i}], 'FontSize', 10);
    end

    xlabel('T (seconds)', 'FontSize', 9);
    ylabel('\sigma', 'FontSize', 9);
    grid on;

    % Estimate parameters for Angle Random Walk (N), Rate Random Walk (K), Bias Instability (B)
    N = adev(find(tau == min(tau)));  % Angle Random Walk (use the first point in the plot)
    K = adev(end);  % Rate Random Walk (use the last point in the plot)
    
    % Bias Instability occurs at the minimum of Allan deviation
    [B, min_idx] = min(adev);
    tau_B = tau(min_idx);  % Tau value at bias instability
    
    % Add text annotations for N, K, and B
    text(0.1, 0.1, {['N (Angle Random Walk): ', num2str(N)], ...
                    ['K (Rate Random Walk): ', num2str(K)], ...
                    ['B (Bias Instability): ', num2str(B), ' at T = ', num2str(tau_B), 's']}, ...
         'Units', 'normalized', 'FontSize', 8, 'Color', 'black');
end

disp('Allan deviation plots displayed successfully.');
