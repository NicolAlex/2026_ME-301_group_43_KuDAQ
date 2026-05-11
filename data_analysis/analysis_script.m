input_data = readmatrix('50_02.log'); % Read data from log file
time_vect = input_data(:, 4) / 1e3; % Extract time vector and convert from microseconds to seconds
% Extract sensor data (assuming columns 1-3 are sensor readings)
axis1_vect = input_data(:, 1); % Sensor data for axis 1
axis2_vect = input_data(:, 2); % Sensor data for axis 2
axis3_vect = input_data(:, 3); % Sensor data for axis 3

% Plotting the sensor data for debugging
close all; % Close any existing figures
figure("Name", "Raw sensor data");
plot(time_vect, axis1_vect, 'r', 'DisplayName', 'Axis 1');
hold on;
plot(time_vect, axis2_vect, 'g', 'DisplayName', 'Axis 2');
plot(time_vect, axis3_vect, 'b', 'DisplayName', 'Axis 3');
xlabel('Time (s)');
ylabel('Sensor Reading');
title('Sensor Data Over Time');
legend;
grid on;


axis1_rad = deg2rad(axis1_vect); % Convert axis 1 data from degrees to radians
axis2_rad = deg2rad(axis2_vect); % Convert axis 2 data from degrees to radians
axis3_rad = deg2rad(axis3_vect); % Convert axis 3 data from degrees to radians


% the angles can be non periodic, so we can use sin(angle) to get continuous values
axis1_sin = sin(axis1_rad); % Compute sine of axis 1 data
axis2_sin = sin(axis2_rad); % Compute sine of axis 2 data
axis3_sin = sin(axis3_rad); % Compute sine of axis 3 data

% remove the mean to center the data around zero for better FFT analysis
axis1_sin = axis1_sin - mean(axis1_sin); % Center axis 1 sine data around zero
axis2_sin = axis2_sin - mean(axis2_sin); % Center axis 2 sine data around zero
axis3_sin = axis3_sin - mean(axis3_sin); % Center axis 3 sine data around zero


% Plotting the sine of each axis and their raw values for debugging
figure("Name", "Sine of Axes vs Raw Axes");
plot(time_vect, axis1_sin, 'r', 'DisplayName', 'Axis 1 (degrees)');
hold on;
plot(time_vect, axis2_sin, 'g', 'DisplayName', 'Sine of Axis 2');
plot(time_vect, axis3_sin, 'b', 'DisplayName', 'Sine of Axis 3');
xlabel('Time (s)');
ylabel('Value');
title('Sine of Axes vs Raw Axes');
legend;
grid on;


% perform FFT on the sines to analyze their frequency content
Fs = 1 / mean(diff(time_vect)); % Sampling frequency
length_axis1 = length(axis1_sin); % Length of the signal
FT_axis1_sin = fft(axis1_sin); % Compute FFT
FT_axis2_sin = fft(axis2_sin); % Compute FFT
FT_axis3_sin = fft(axis3_sin); % Compute FFT
f = (0:length_axis1/2-1) * Fs / length_axis1; % Frequency vector

% Plotting the FFT of the sines for debugging
figure("Name", "FFT of the sines of the axes");
plot(f, abs(FT_axis1_sin(1:length_axis1/2)), 'r', 'DisplayName', 'FFT of Sine of Axis 1');
hold on;
plot(f, abs(FT_axis2_sin(1:length_axis1/2)), 'g', 'DisplayName', 'FFT of Sine of Axis 2');
plot(f, abs(FT_axis3_sin(1:length_axis1/2)), 'b', 'DisplayName', 'FFT of Sine of Axis 3');
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('FFT of the Sines of the Axes');
legend;
grid on;



% printing system constants for debugging

fprintf('Sampling Frequency: %.2f Hz\n', Fs);
fprintf('Length of Signal: %d samples\n', length_axis1);
fprintf('Frequency Resolution : %.2f Hz\n', Fs / length_axis1);
fprintf('Duration of the test: %.2f s\n', length_axis1 / Fs);