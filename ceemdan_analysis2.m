% Hyper-Realistic Drone Signal Simulation for CEEMDAN Validation
clear; clc; close all;

%% 1. Physical Parameters
fs = 2000; % Higher sampling frequency to capture harmonics cleanly
t = 0:1/fs:1.0; % 1 second of dense data

% Base rotational frequencies of the 4 individual motors (BPF)
f1 = 115; f2 = 120; f3 = 125; f4 = 128;

%% 2. Generate Truly Realistic Components
% Component A: Microphone/Sensor Electrical White Noise
electrical_noise = 0.05 * randn(size(t));

% Component B: Blade Vortices (Aerodynamic Broadband Noise)
% Passing white noise through a high-pass filter to mimic high-frequency air ripping
aerodynamic_noise = 0.15 * highpass(randn(size(t)), 400, fs);

% Component C: The 4 Motors WITH structural harmonics (1st, 2nd, and 3rd harmonics)
blades = zeros(size(t));
motor_freqs = [f1, f2, f3, f4];
amplitudes = [0.4, 0.2, 0.1]; % Fundamental is strongest, harmonics get weaker

for f_base = motor_freqs
    blades = blades + ...
        amplitudes(1) * sin(2 * pi * f_base * t) + ... % Fundamental Tone
        amplitudes(2) * sin(2 * pi * (2*f_base) * t) + ... % 2nd Harmonic
        amplitudes(3) * sin(2 * pi * (3*f_base) * t); % 3rd Harmonic
end

% Component D: Realistic Chaotic Wind (Low-frequency random walk trend)
wind_gust = cumsum(0.08 * randn(size(t)));
wind_gust = wind_gust - mean(wind_gust); % Center it around 0

%% 3. The Final Composite Signal
x = blades + aerodynamic_noise + electrical_noise + wind_gust;

%% 4. Run CEEMDAN (Optimized for real-world signal distribution)
Nstd = 0.25;
NR = 80;
MaxIter = 150;
fprintf('Processing realistic signal via CEEMDAN... \n');
[modes, its] = ceemdan(x, Nstd, NR, MaxIter, 1);
num_modes = size(modes, 1);
fprintf('Decomposition finished.\n');

%% 5. Plot Output Windows
% Plot the Master Input
figure('Name', 'REALISTIC INPUT: Raw Drone Audio/Vibration', 'NumberTitle', 'off');
plot(t, x, 'r'); title('Raw Realistic Sensor Data'); xlabel('Time (s)'); ylabel('Amplitude'); grid on;

% Plot Individual IMFs
for i = 1:num_modes
    figure('Name', ['REALISTIC OUTPUT: Mode ' num2str(i)], 'NumberTitle', 'off');
    plot(t, modes(i, :), 'b');
    grid on; xlabel('Time (s)'); ylabel('Amplitude');
    
    if i <= 2
        title(['Mode ' num2str(i) ': Aerodynamic Air Ripping & Sensor Hiss (High Frequencies)']);
    elseif i == 3 || i == 4
        title(['Mode ' num2str(i) ': Isolated Motor Blades & Structural Harmonics']);
    elseif i >= 5 && i < num_modes
        title(['Mode ' num2str(i) ': Chaotic Environmental Wind Buffet']);
    else
        title('Final Mode: Residual Flight Path Bias / Trend');
    end
end

%% 5. Time-Frequency Analysis (Hilbert Spectrum Approximation)
figure('Name', 'Time-Frequency Analysis of All Modes', 'NumberTitle', 'off');
hold on;

% Create a distinct color map so each mode stands out clearly
colors = lines(num_modes);

% Loop through each mode to calculate and plot its frequency over time
for i = 1:num_modes-1 % Skip the final residual trend as its frequency is ~0 Hz
    current_mode = modes(i, :);
    
    % Compute the Instantaneous Frequency of the current mode
    % 'instfreq' requires the analytic signal, which we get using the Hilbert Transform
    try
        [inst_f, t_inst] = instfreq(current_mode, fs, 'Method', 'hilbert');
        
        % Plot the frequency track over time
        % We use a scatter plot or a fine line to show how the frequency behaves
        plot(t_inst, inst_f, '.', 'Color', colors(i, :), 'MarkerSize', 4);
    catch
        % Fallback if a mode is too flat or short to compute
        continue;
    end
end

title('Time-Frequency Profile (Instantaneous Frequency Per Mode)');
xlabel('Time (seconds)');
ylabel('Frequency (Hz)');
grid on;

% Adjust the Y-axis to focus on your drone blade zone and lower harmonics
ylim([0 400]);

% Add a simple legend matching the color layers
legend_labels = cell(1, num_modes-1);
for i = 1:num_modes-1
    legend_labels{i} = ['Mode ' num2str(i)];
end
legend(legend_labels, 'Location', 'eastoutside');
