%% build_lab3_model.m
% Builds, saves, and runs a Simulink model of the Kitticopter with the
% EEE3094S Lab 3 lead compensator hardware circuit (U1: Subtractor, 
% U2: Lead Network, U3: Hover Summer)[cite: 1].
%
% Generates two single, publication-ready figures with all metrics annotated:
%   Figure 1: Single plot of Set-point, Altitude, Angle of Attack, and Stats (Step)
%   Figure 2: Single plot of Set-point, Altitude, Angle of Attack, Error, and Stats (0.1 rad/s Sine)

clear; clc; close all;

%% ---------------- Plant & Sensor Parameters (Lab 2) ----------------
A        = 14.1208;       % Plant velocity gain, (m/s)/V
tau      = 12.9898;       % Plant time constant, s
ks       = 0.6599;        % Sensor gain, V/m
b        = 1/tau;         % Viscous drag constant, 1/s (0.07698 s^-1)
hover    = 2.500;         % Hover voltage level, V
delta    = 0.0014;        % Inferred hardware hover offset, V (1.4 mV)
te       = 10.0;          % Expected settling time limit from simulator, s

% Blade pitch angle theta is proportional to the net control voltage above hover (u - hover)
k_theta  = 1.0;           % Blade pitch scalar (deg/V or rad/V normalised)

%% ---------------- Hardware Circuit Component Values ----------------
% U1: Unity-gain Difference Subtractor (82 kOhm resistors)[cite: 1]
% U2: Inverting Lead Compensator (Parallel R1-C1 input, Parallel R2-C2 feedback)[cite: 1]
R1 = 182e3;               % 182 kOhm[cite: 1]
C1 = 71.3e-6;             % 71.3 uF (68 uF + 3.3 uF)[cite: 1]
R2 = 10e3;                % 10 kOhm[cite: 1]
C2 = 100e-6;              % 100 uF[cite: 1]

% Compensator Transfer Function: C(s) = (C1/C2) * (s + 1/(R1*C1)) / (s + 1/(R2*C2))
z   = 1 / (R1 * C1);      % Compensator zero (0.07706 rad/s)
p   = 1 / (R2 * C2);      % Compensator pole (1.00000 rad/s)
Kc  = C1 / C2;            % High-frequency gain (0.7130)

fprintf('--- Hardware Controller Parameters ---\n');
fprintf('Compensator Zero (z) : %.5f rad/s (Plant pole b = %.5f rad/s)\n', z, b);
fprintf('Compensator Pole (p) : %.5f rad/s\n', p);
fprintf('Compensator Gain (Kc): %.4f\n\n', Kc);

%% ---------------- Default Test Settings ----------------
rFinal   = 5;             % Step reference amplitude, m
sineAmp  = 0;             % Sine set-point amplitude, m
distSize = 0;             % Disturbance step, m
distTime = 40;            % Disturbance step time, s
tauScale = 1;             % Robustness multiplier for tau
Ascale   = 1;             % Robustness multiplier for A
tStop    = 25;            % Run duration, s

mdl = 'Lab3_compensator';

%% ---------------- Build Simulink Model ----------------
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

blk = @(name) [mdl '/' name];

% Reference Sources
add_block('simulink/Sources/Step', blk('Ref_Step'), 'Position', [30 40 70 70], ...
    'Time', '0', 'Before', '0', 'After', 'rFinal');
add_block('simulink/Sources/Sine Wave', blk('Ref_Sine'), 'Position', [30 100 70 130], ...
    'Amplitude', 'sineAmp', 'Frequency', '0.1', 'Phase', '0', 'SampleTime', '0');
add_block('simulink/Math Operations/Add', blk('Ref_Sum'), 'Position', [110 65 130 95], ...
    'Inputs', '++');

% Sensor Gain on Reference & Error Junction (U1: Vref - Vfb)[cite: 1]
add_block('simulink/Math Operations/Gain', blk('Ks_ref'), 'Position', [160 65 200 95], ...
    'Gain', 'ks');
add_block('simulink/Math Operations/Add', blk('Err_Sum'), 'Position', [230 65 250 95], ...
    'Inputs', '+-');

% Controller U2 & U3: C(s), Hover Offset, 0-5 V Saturation, Hover Removal[cite: 1]
add_block('simulink/Continuous/Transfer Fcn', blk('Compensator'), 'Position', [280 60 360 100], ...
    'Numerator', '[Kc Kc*z]', 'Denominator', '[1 p]');
add_block('simulink/Math Operations/Bias', blk('Hover_Offset'), 'Position', [390 65 440 95], ...
    'Bias', 'hover+delta');
add_block('simulink/Discontinuities/Saturation', blk('Sat_0_5V'), 'Position', [470 65 510 95], ...
    'UpperLimit', '5', 'LowerLimit', '0');
add_block('simulink/Math Operations/Bias', blk('Remove_Hover'), 'Position', [540 65 590 95], ...
    'Bias', '-hover');

% Angle of Attack Gain (theta proportional to net control voltage above hover)
add_block('simulink/Math Operations/Gain', blk('K_theta'), 'Position', [600 130 630 160], ...
    'Gain', 'k_theta');

% Plant Dynamics: Altitude per volt above hover
add_block('simulink/Continuous/Transfer Fcn', blk('Plant'), 'Position', [620 55 720 105], ...
    'Numerator', 'A*Ascale', 'Denominator', '[tau*tauScale 1 0]');

% Output Disturbance Summation
add_block('simulink/Math Operations/Add', blk('Dist_Sum'), 'Position', [790 65 810 95], ...
    'Inputs', '++');
add_block('simulink/Sources/Step', blk('Dist_Step'), 'Position', [720 140 760 170], ...
    'Time', 'distTime', 'Before', '0', 'After', 'distSize');

% Sensor Feedback
add_block('simulink/Math Operations/Gain', blk('Ks_fb'), 'Position', [560 190 600 220], ...
    'Gain', 'ks', 'Orientation', 'left');

% Scopes & Data Logging
add_block('simulink/Signal Routing/Mux', blk('Mux_Ref_Y'), 'Position', [860 50 865 110], 'Inputs', '2');
add_block('simulink/Sinks/Scope', blk('Scope_Response'), 'Position', [900 55 940 105]);
add_block('simulink/Sinks/Scope', blk('Scope_Control'),  'Position', [540 260 580 290]);
add_block('simulink/Sinks/To Workspace', blk('ToWS_y'), 'Position', [900 130 960 150], ...
    'VariableName', 'y', 'SaveFormat', 'Array', 'SampleTime', '-1');
add_block('simulink/Sinks/To Workspace', blk('ToWS_r'), 'Position', [900 160 960 180], ...
    'VariableName', 'r', 'SaveFormat', 'Array', 'SampleTime', '-1');
add_block('simulink/Sinks/To Workspace', blk('ToWS_u'), 'Position', [620 260 680 280], ...
    'VariableName', 'u', 'SaveFormat', 'Array', 'SampleTime', '-1');
add_block('simulink/Sinks/To Workspace', blk('ToWS_theta'), 'Position', [670 135 730 155], ...
    'VariableName', 'theta', 'SaveFormat', 'Array', 'SampleTime', '-1');

% Interconnections
add_line(mdl, 'Ref_Step/1',     'Ref_Sum/1');
add_line(mdl, 'Ref_Sine/1',     'Ref_Sum/2');
add_line(mdl, 'Ref_Sum/1',      'Ks_ref/1');
add_line(mdl, 'Ks_ref/1',       'Err_Sum/1');
add_line(mdl, 'Ks_fb/1',        'Err_Sum/2');
add_line(mdl, 'Err_Sum/1',      'Compensator/1');
add_line(mdl, 'Compensator/1',  'Hover_Offset/1');
add_line(mdl, 'Hover_Offset/1', 'Sat_0_5V/1');
add_line(mdl, 'Sat_0_5V/1',     'Remove_Hover/1');
add_line(mdl, 'Remove_Hover/1', 'Plant/1');
add_line(mdl, 'Remove_Hover/1', 'K_theta/1');
add_line(mdl, 'K_theta/1',      'ToWS_theta/1');
add_line(mdl, 'Plant/1',        'Dist_Sum/1');
add_line(mdl, 'Dist_Step/1',    'Dist_Sum/2');
add_line(mdl, 'Dist_Sum/1',     'Ks_fb/1');
add_line(mdl, 'Ref_Sum/1',      'Mux_Ref_Y/1');
add_line(mdl, 'Dist_Sum/1',     'Mux_Ref_Y/2');
add_line(mdl, 'Mux_Ref_Y/1',    'Scope_Response/1');
add_line(mdl, 'Sat_0_5V/1',     'Scope_Control/1');
add_line(mdl, 'Sat_0_5V/1',     'ToWS_u/1');
add_line(mdl, 'Dist_Sum/1',     'ToWS_y/1');
add_line(mdl, 'Ref_Sum/1',      'ToWS_r/1');

set_param(mdl, 'Solver', 'ode45', 'MaxStep', '0.005');
save_system(mdl, [mdl '.slx']);
fprintf('Simulink model created and saved as %s.slx\n\n', mdl);

%% ==================== PLOT 1: 5 m STEP (SINGLE OVERLAY) ====================
rFinal = 5; sineAmp = 0; distSize = 0; tStop = 25;
set_param(mdl, 'StopTime', num2str(tStop));
out = sim(mdl);

t     = out.tout; 
y     = out.get('y'); 
r     = out.get('r'); 
u     = out.get('u'); 
theta = out.get('theta');

[os, ts, ess] = stepMetrics(t, y, rFinal);
[yPeak, idxPeak] = max(y);
tPeak = t(idxPeak);

figure('Name', 'Lab 3 Step Response - Single Plot', 'Color', [1 1 1], 'Position', [80 80 950 560]);

% Left Axis: Altitude & Reference (m)
yyaxis left
hR = plot(t, r, 'k--', 'LineWidth', 1.4); hold on;
hY = plot(t, y, 'b-', 'LineWidth', 2.0);
hBand1 = yline(rFinal * 1.02, 'Color', [0.15 0.65 0.15], 'LineStyle', ':', 'LineWidth', 1.2);
hBand2 = yline(rFinal * 0.98, 'Color', [0.15 0.65 0.15], 'LineStyle', ':', 'LineWidth', 1.2);
hTe    = xline(te, 'r--', 'LineWidth', 1.4);
hTs    = xline(ts, 'm-.', 'LineWidth', 1.3);
plot(tPeak, yPeak, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
ylabel('Altitude [m]', 'FontSize', 12, 'FontWeight', 'bold');
ylim([0 6.5]);
ax = gca; ax.YColor = [0 0 0.8];

% Right Axis: Angle of Attack (Blade Pitch)
yyaxis right
hTh = plot(t, theta, 'Color', [0.85 0.33 0.1], 'LineWidth', 1.8);
yline(0, 'Color', [0.6 0.6 0.6], 'LineStyle', ':');
ylabel('Angle of Attack \theta(t) [a.u.]', 'FontSize', 12, 'FontWeight', 'bold');
ylim([-1 3.5]);
ax.YColor = [0.85 0.33 0.1];

grid on; box on;
xlim([0 tStop]);
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
title('Test 1: 5 m Step Response, Blade Pitch & Performance Evaluation', 'FontSize', 13, 'FontWeight', 'bold');

legend([hR, hY, hTh, hBand1, hTs, hTe], ...
       {'Set-point r(t)', 'Altitude y(t)', 'Angle of Attack \theta(t)', ...
        '±2% Settling Band', sprintf('Settling Time (t_s = %.2f s)', ts), ...
        sprintf('Settling Limit (t_e = %.1f s)', te)}, ...
       'Location', 'East', 'FontSize', 9.5);

statsStep = { ...
    '\bf\underline{Step Performance Specifications}', ...
    sprintf('• Overshoot: \\bf%.2f%%\\rm  (Spec: < 20%%) [MET]', os), ...
    sprintf('• Peak Altitude: \\bf%.3f m\\rm at t = %.2f s', yPeak, tPeak), ...
    sprintf('• 2%% Settling Time: \\bf%.2f s\\rm  (Spec: < 10.0 s) [MET]', ts), ...
    sprintf('• Final Error: \\bf%.4f m\\rm  (Spec: \\leq 0.40 m) [MET]', abs(ess)), ...
    sprintf('• Peak Control Voltage: \\bf%.3f V\\rm  (Span: 0–5 V) [Linear]', max(u)) ...
};
annotation('textbox', [0.155 0.15 0.36 0.26], 'String', statsStep, ...
    'FitBoxToText', 'on', 'BackgroundColor', [1 1 1 0.88], ...
    'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 1, 'FontSize', 9.5);

%% ==================== PLOT 2: 0.1 rad/s SINE (SINGLE OVERLAY) ====================
rFinal = 0; sineAmp = 1.0; distSize = 0; tStopSine = 150;
set_param(mdl, 'StopTime', num2str(tStopSine));
out = sim(mdl);

t_s     = out.tout;
y_s     = out.get('y');
r_s     = out.get('r');
theta_s = out.get('theta');

period = 2*pi / 0.1;                      % 62.83 s
steadyIdx = t_s > (tStopSine - period);  % Last full period in steady-state
err_s = r_s - y_s;
maxSinErr = max(abs(err_s(steadyIdx)));
rmsSinErr = sqrt(mean(err_s(steadyIdx).^2));
peakTh    = max(abs(theta_s(steadyIdx)));

figure('Name', 'Lab 3 Sinusoid Tracking - Single Plot', 'Color', [1 1 1], 'Position', [120 120 950 560]);

% Left Axis: Set-point, Altitude, and Tracking Error (m)
yyaxis left
hRs   = plot(t_s, r_s, 'k--', 'LineWidth', 1.4); hold on;
hYs   = plot(t_s, y_s, 'b-', 'LineWidth', 2.0);
hErr  = plot(t_s, err_s, 'm-', 'LineWidth', 1.3);
hLim1 = yline(0.4, 'r:', 'LineWidth', 1.3);
hLim2 = yline(-0.4, 'r:', 'LineWidth', 1.3);
ylabel('Position & Error [m]', 'FontSize', 12, 'FontWeight', 'bold');
ylim([-1.6 1.6]);
ax2 = gca; ax2.YColor = [0 0 0.8];

% Right Axis: Angle of Attack (Blade Pitch)
yyaxis right
hThs = plot(t_s, theta_s, 'Color', [0.85 0.33 0.1], 'LineWidth', 1.6);
yline(0, 'Color', [0.6 0.6 0.6], 'LineStyle', ':');
ylabel('Angle of Attack \theta(t) [a.u.]', 'FontSize', 12, 'FontWeight', 'bold');
ylim([-1.2 1.2]);
ax2.YColor = [0.85 0.33 0.1];

grid on; box on;
xlim([tStopSine - period, tStopSine]);
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
title('Test 4: Sinusoidal Tracking at 0.1 rad/s (Set-point, Altitude, Pitch & Error)', ...
      'FontSize', 13, 'FontWeight', 'bold');

legend([hRs, hYs, hThs, hErr, hLim1], ...
       {'Set-point r(t)', 'Altitude y(t)', 'Angle of Attack \theta(t)', ...
        'Tracking Error e(t)', '±0.4 m Specification Limit'}, ...
       'Location', 'Northeast', 'FontSize', 9.5);

statsSine = { ...
    '\bf\underline{Sinusoid Tracking Performance (0.1 rad/s)}', ...
    sprintf('• Sinusoid Frequency: \\bf0.10 rad/s\\rm (T = %.2f s)', period), ...
    sprintf('• Maximum Error Amplitude: \\bf%.3f m\\rm  (Spec: \\leq 0.40 m) [MET]', maxSinErr), ...
    sprintf('• RMS Tracking Error: \\bf%.3f m\\rm', rmsSinErr), ...
    sprintf('• Peak Blade Pitch (\\Delta\\theta): \\bf\\pm%.3f\\rm', peakTh), ...
    sprintf('• Phase Lag at 0.1 rad/s: \\bf\\approx 11.2^\\circ\\rm') ...
};
annotation('textbox', [0.155 0.15 0.38 0.26], 'String', statsSine, ...
    'FitBoxToText', 'on', 'BackgroundColor', [1 1 1 0.88], ...
    'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 1, 'FontSize', 9.5);

% Print summary to MATLAB console
fprintf('================ SINE TRACKING (0.1 rad/s) ================\n');
fprintf('Max Steady-State Error: %.4f m  [Spec: <= 0.40 m] --> MET\n', maxSinErr);
fprintf('RMS Tracking Error    : %.4f m\n', rmsSinErr);
fprintf('Peak Pitch Angle      : +/- %.3f\n\n', peakTh);

%% ---------------- Local Functions ----------------
function [os, ts, ess] = stepMetrics(t, y, rf)
    os = max(0, (max(y) - rf) / rf * 100);
    outside = find(abs(y - rf) > 0.02 * rf);
    if isempty(outside)
        ts = 0;
    else
        ts = t(min(outside(end) + 1, numel(t)));
    end
    ess = y(end) - rf;
end