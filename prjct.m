% Load or generate input torque and output velocity data
% Model/tf for first vehicle only
torque_data = systemInputData{1}.Data; % Torque data in Nm
velocity_data_kmph = systemResponseData{1}.Data; % Velocity data in km/h

% Convert velocity from km/h to m/s
velocity_data_mps = velocity_data_kmph * (1000 / 3600);

% Sampling frequency and time vector
fs = 1000; % Sampling frequency in Hz
T = 1/fs;  % Sampling interval
t = 0:T:(length(torque_data)-1)*T; % Time vector

% Perform FFT on the input and output data
torque_fft = fft(torque_data);
velocity_fft = fft(velocity_data_mps);

% Compute frequency vector
f = fs*(0:(length(torque_data)/2))/length(torque_data);

% Calculate transfer function (H) in frequency domain
H_freq_domain = velocity_fft ./ torque_fft;

% Plot magnitude and phase of the transfer function
figure;
subplot(2, 1, 1);
plot(f, abs(H_freq_domain(1:length(torque_data)/2+1)));
title('Magnitude of Transfer Function');
xlabel('Frequency (Hz)');
ylabel('Magnitude');

subplot(2, 1, 2);
plot(f, angle(H_freq_domain(1:length(torque_data)/2+1)));
title('Phase of Transfer Function');
xlabel('Frequency (Hz)');
ylabel('Phase (rad)');

% Identify parameters from frequency response
[~, idx_peak] = max(abs(H_freq_domain));
natural_frequency = f(idx_peak);
damping_ratio = -real(log(H_freq_domain(idx_peak))) / (2*pi*natural_frequency);

fprintf('Estimated Natural Frequency: %.2f Hz\n', natural_frequency);
fprintf('Estimated Damping Ratio: %.4f\n', damping_ratio);

% Build transfer function model
K = abs(H_freq_domain(idx_peak));
numerator = K;
denominator = [1, 2*damping_ratio*natural_frequency, natural_frequency^2];
sys = tf(numerator, denominator);

% Display the transfer function model
disp('Estimated Transfer Function:');
disp(sys);