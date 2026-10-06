% ========================================================================
% Title: Multi-Channel Augmented EMD Profile for Drone Flight Telemetry
% Framework: Native Multichannel Adaptive Spline EMD
% ========================================================================

clear; clc; close all;

%% 1. Flight Constants & Physical Parameters
fs = 2000;                    % High-speed sampling rate (Hz)
t = 0:1/fs:1.0;               % 1.0 second snapshot window
N = length(t);

% Safe flight regime envelope for target diagnostic engine
MIN_SAFE_FREQ = 112; 
MAX_SAFE_FREQ = 130;

% Target rotational signatures (BPFs) 
% Notice Motor 4 is falling out of nominal speed limits (110 Hz)
f1 = 115; f2 = 120; f3 = 125; f4 = 110; 
motor_freqs = [f1, f2, f3, f4];
amplitudes = [0.4, 0.2, 0.1]; % Fundamental, 2nd, and 3rd harmonics

%% 2. Generate Truly Realistic Multi-Axis Sensor Array (X, Y, Z Channels)
% In real aviation systems, vibrations impact different axes with distinct phases
X_blades = zeros(size(t)); Y_blades = zeros(size(t)); Z_blades = zeros(size(t));

for f_base = motor_freqs
    % X-Axis Structural Loading
    X_blades = X_blades + amplitudes(1)*sin(2*pi*f_base*t) + ...
                          amplitudes(2)*sin(2*pi*(2*f_base)*t) + ...
                          amplitudes(3)*sin(2*pi*(3*f_base)*t);
    % Y-Axis Structural Loading (90-degree phase shifts due to spatial positioning)
    Y_blades = Y_blades + amplitudes(1)*cos(2*pi*f_base*t) + ...
                          amplitudes(2)*cos(2*pi*(2*f_base)*t) + ...
                          amplitudes(3)*cos(2*pi*(3*f_base)*t);
    % Z-Axis Structural Loading (3D vector combination)
    Z_blades = Z_blades + amplitudes(1)*sin(2*pi*f_base*t + 0.25*pi) + ...
                          amplitudes(2)*sin(2*pi*(2*f_base)*t + 0.1*pi);
end

% Multi-channel environmental weather modeling (Chaotic Wind Trends)
X_wind = cumsum(0.08 * randn(size(t))); X_wind = X_wind - mean(X_wind);
Y_wind = cumsum(0.07 * randn(size(t))); Y_wind = Y_wind - mean(Y_wind);
Z_wind = cumsum(0.09 * randn(size(t))); Z_wind = Z_wind - mean(Z_wind);

% High-frequency aerodynamic pressure rips & electrical noise across sensors
X_noise = 0.05*randn(size(t)) + 0.12*highpass(randn(size(t)), 400, fs);
Y_noise = 0.05*randn(size(t)) + 0.12*highpass(randn(size(t)), 400, fs);
Z_noise = 0.05*randn(size(t)) + 0.12*highpass(randn(size(t)), 400, fs);

% Combine individual channels into a unified Multivariate Signal Matrix [Samples x Channels]
X_total = X_blades + X_wind + X_noise;
Y_total = Y_blades + Y_wind + Y_noise;
Z_total = Z_blades + Z_wind + Z_noise;

raw_multivariate_signal = [X_total; Y_total; Z_total]'; 

%% 3. Execute Multi-Channel Augmented EMD Core
fprintf('Executing Augmented/Multivariate EMD across all channels... \n');

% Passing a matrix [Samples x Channels] forces MATLAB to run a joint sifting loop. 
% This ensures channel components remain perfectly synced across IMF layers.
[augmented_modes, residual] = emd(raw_multivariate_signal, 'Interpolation', 'spline');

% Structure size check
[num_samples, num_channels, num_modes] = size(augmented_modes);
fprintf('Decomposition success. Unified scale space aligned into %d distinct IMFs.\n', num_modes);

%% 4. Automated Failure Diagnostics Execution (Feature Extraction)
detected_blade_freqs = [];

% Analyze Mode 2 and Mode 3 across all channels (The prime structural oscillation bands)
for ch = 1:num_channels
    for m = 2:3
        current_layer = augmented_modes(:, ch, m);
        
        % Calculate spectrum signatures via FFT
        fft_val = fft(current_layer);
        fft_pow = abs(fft_val(1:floor(N/2)+1)).^2;
        freqs_vector = (0:floor(N/2)) * (fs / N);
        
        % Pull localized peak items within operational motor profile spectrum thresholds
        [~, locs] = findpeaks(fft_pow, 'MinPeakDistance', 4, 'SortStr', 'descend');
        for p_idx = 1:min(length(locs), 4)
            found_f = freqs_vector(locs(p_idx));
            if found_f >= 95 && found_f <= 140 
                detected_blade_freqs = [detected_blade_freqs, found_f];
            end
        end
    end
end
detected_blade_freqs = unique(round(detected_blade_freqs));

% Print Telemetry Diagnosis Verdict directly to Command Window
fprintf('\n================ TELEMETRY HEALTH REPORT ================ \n');
fault_flag = false;
for f_val = detected_blade_freqs
    if f_val < MIN_SAFE_FREQ
        fprintf('🔴 ANOMALY DETECTED: Engine component running dangerously slow at %d Hz! (Safety Minimum: %d Hz)\n', f_val, MIN_SAFE_FREQ);
        fault_flag = true;
    elseif f_val > MAX_SAFE_FREQ
        fprintf('🔴 ANOMALY DETECTED: Structural Over-speed / Resonance detected at %d Hz! (Safety Maximum: %d Hz)\n', f_val, MAX_SAFE_FREQ);
        fault_flag = true;
    else
        fprintf('🟢 NOMINAL OPERATIONAL LAYER: Active engine rotation verified at %d Hz.\n', f_val);
    end
end
if ~fault_flag, fprintf('✅ STATUS: Systems nominal. All structural dynamics match clear flight patterns.\n'); end
fprintf('========================================================= \n\n');

%% 5. Unified Graphical Output Windows
% Window 1: Raw 3-Axis Telemetry Input Data
figure('Name', 'Multivariate Sensor Inputs', 'NumberTitle', 'off');
subplot(3,1,1); plot(t, X_total, 'r'); title('Raw Telemetry Profile: X-Axis Sensor'); ylabel('Amplitude'); grid on;
subplot(3,1,2); plot(t, Y_total, 'g'); title('Raw Telemetry Profile: Y-Axis Sensor'); ylabel('Amplitude'); grid on;
subplot(3,1,3); plot(t, Z_total, 'b'); title('Raw Telemetry Profile: Z-Axis Sensor'); ylabel('Amplitude'); xlabel('Time (s)'); grid on;

% Window 2: Visualizing Aligned IMF Mode 2 Across All 3 Axes
figure('Name', 'Augmented Aligned Components (IMF 2)', 'NumberTitle', 'off');
subplot(3,1,1); plot(t, augmented_modes(:, 1, 2), 'r'); title('Isolated Dynamics: IMF Mode 2 [X-Axis]'); ylabel('Amplitude'); grid on;
subplot(3,1,2); plot(t, augmented_modes(:, 2, 2), 'g'); title('Isolated Dynamics: IMF Mode 2 [Y-Axis]'); ylabel('Amplitude'); grid on;
subplot(3,1,3); plot(t, augmented_modes(:, 3, 2), 'b'); title('Isolated Dynamics: IMF Mode 2 [Z-Axis]'); ylabel('Amplitude'); xlabel('Time (s)'); grid on;

% Window 3: Cross-Channel Joint Time Frequency Mapping (X-Axis Baseline Example)
figure('Name', 'Augmented Time Frequency Mapping Profile', 'NumberTitle', 'off');
hold on; colors = lines(num_modes);
for m = 1:num_modes
    try
        [inst_f, t_inst] = instfreq(augmented_modes(:, 1, m), fs, 'Method', 'hilbert');
        plot(t_inst, inst_f, '.', 'Color', colors(m, :), 'MarkerSize', 4);
    catch, continue; end
end
title('Augmented Time-Frequency Mapping Network (X-Axis Processing Chain)');
xlabel('Time (s)'); ylabel('Frequency (Hz)'); ylim([0 400]); grid on;
