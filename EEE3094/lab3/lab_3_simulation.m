%% build_lab3_model.m
% Builds, saves and runs a Simulink model of the kiLicopter with the
% EEE3094S Lab 3 lead compensator (design B), including:
%   - 0-5 V clipping of the controller output (hover level = 2.5 V)
%   - hover-level offset delta added before the clipping
%   - output disturbance and sinusoidal set-point sources
% Needs MATLAB R2016b or later (local functions at the end of a script)
% and Simulink. Run the whole file; it creates Lab3_compensator.slx
% in the current folder.
%
% NOTE: this script has not been run in MATLAB by the author. If a block
% or parameter name errors on your release, tell me the message.

clear; clc;

%% ---------------- Parameters (edit here) ----------------
A     = 14.1208;     % plant gain, (m/s)/V          (Lab 2, Table 1)
tau   = 12.9898;     % plant time constant, s       (Lab 2, Table 1)
ks    = 0.6599;      % sensor gain, V/m             (Lab 2, Table 1)
b     = 1/tau;       % drag constant, 1/s

% Compensator C(s) = Kc (s+z)/(s+p)   -- design B
Kc    = 0.711;
z     = b;           % zero cancels the plant pole
p     = 1.000;

hover = 2.5;         % hover level, V
delta = 0.0014;      % hover-level offset, V (inferred from Lab 2)

% Test settings (changed automatically by the test sections below)
rFinal   = 5;        % set-point step, m
sineAmp  = 0;        % sinusoidal set-point amplitude, m (0.1 rad/s)
distSize = 0;        % output-disturbance step size, m
distTime = 40;       % disturbance time, s
tauScale = 1;        % robustness: multiplies tau
Ascale   = 1;        % robustness: multiplies A
tStop    = 40;       % stop time, s

mdl = 'Lab3_compensator';

%% ---------------- Build the model ----------------
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

blk = @(name) [mdl '/' name];

% Set-point sources
add_block('simulink/Sources/Step',      blk('Ref_Step'), 'Position', [30 40 70 70], ...
    'Time', '0', 'Before', '0', 'After', 'rFinal');
add_block('simulink/Sources/Sine Wave', blk('Ref_Sine'), 'Position', [30 100 70 130], ...
    'Amplitude', 'sineAmp', 'Frequency', '0.1', 'Phase', '0', 'SampleTime', '0');
add_block('simulink/Math Operations/Add', blk('Ref_Sum'), 'Position', [110 65 130 95], ...
    'Inputs', '++');

% Sensor gain on the reference, error junction
add_block('simulink/Math Operations/Gain', blk('Ks_ref'), 'Position', [160 65 200 95], ...
    'Gain', 'ks');
add_block('simulink/Math Operations/Add', blk('Err_Sum'), 'Position', [230 65 250 95], ...
    'Inputs', '+-');

% Compensator, hover level + offset, 0-5 V clipping, remove hover
add_block('simulink/Continuous/Transfer Fcn', blk('Compensator'), 'Position', [280 60 360 100], ...
    'Numerator', '[Kc Kc*z]', 'Denominator', '[1 p]');
add_block('simulink/Math Operations/Bias', blk('Hover_Offset'), 'Position', [390 65 440 95], ...
    'Bias', 'hover+delta');
add_block('simulink/Discontinuities/Saturation', blk('Sat_0_5V'), 'Position', [470 65 510 95], ...
    'UpperLimit', '5', 'LowerLimit', '0');
add_block('simulink/Math Operations/Bias', blk('Remove_Hover'), 'Position', [540 65 590 95], ...
    'Bias', '-hover');

% Plant: altitude per volt above hover
add_block('simulink/Continuous/Transfer Fcn', blk('Plant'), 'Position', [620 55 720 105], ...
    'Numerator', 'A*Ascale', 'Denominator', '[tau*tauScale 1 0]');

% Output disturbance
add_block('simulink/Math Operations/Add', blk('Dist_Sum'), 'Position', [790 65 810 95], ...
    'Inputs', '++');
add_block('simulink/Sources/Step', blk('Dist_Step'), 'Position', [720 140 760 170], ...
    'Time', 'distTime', 'Before', '0', 'After', 'distSize');

% Feedback sensor gain (drawn right-to-left)
add_block('simulink/Math Operations/Gain', blk('Ks_fb'), 'Position', [560 160 600 190], ...
    'Gain', 'ks', 'Orientation', 'left');

% Displays and logging
add_block('simulink/Signal Routing/Mux', blk('Mux_Ref_Y'), 'Position', [860 50 865 110], ...
    'Inputs', '2');
add_block('simulink/Sinks/Scope', blk('Scope_Response'), 'Position', [900 55 940 105]);
add_block('simulink/Sinks/Scope', blk('Scope_Control'),  'Position', [540 230 580 260]);
add_block('simulink/Sinks/To Workspace', blk('ToWS_y'), 'Position', [900 130 960 150], ...
    'VariableName', 'y', 'SaveFormat', 'Array', 'SampleTime', '-1');
add_block('simulink/Sinks/To Workspace', blk('ToWS_r'), 'Position', [900 160 960 180], ...
    'VariableName', 'r', 'SaveFormat', 'Array', 'SampleTime', '-1');
add_block('simulink/Sinks/To Workspace', blk('ToWS_u'), 'Position', [620 230 680 250], ...
    'VariableName', 'u', 'SaveFormat', 'Array', 'SampleTime', '-1');

% Wiring
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

set_param(mdl, 'Solver', 'ode45', 'MaxStep', '0.01', 'StopTime', 'tStop');
save_system(mdl, [mdl '.slx']);
fprintf('Model saved as %s.slx\n\n', mdl);

%% ---------------- Test 1: 5 m step (nominal) ----------------
rFinal = 5; sineAmp = 0; distSize = 0; tauScale = 1; Ascale = 1; tStop = 40;
out = sim(mdl);
t = out.tout; y = out.get('y'); r = out.get('r'); u = out.get('u');
[os, ts, ess] = stepMetrics(t, y, rFinal);
fprintf('STEP (nominal):  overshoot = %.2f %%,  2%% settling = %.2f s,  final error = %.4f m,  peak u = %.3f V\n', ...
    os, ts, ess, max(u));

figure('Name', 'Lab 3 step response');
subplot(2,1,1); plot(t, r, '--', t, y); grid on;
ylabel('Altitude (m)'); legend('set-point', 'output', 'Location', 'southeast');
title('5 m step, design B (clipping and offset included)');
subplot(2,1,2); plot(t, u); grid on; yline(0, ':'); yline(5, ':');
ylabel('Controller output (V)'); xlabel('Time (s)');

%% ---------------- Test 2: robustness (b and A +/-10 %) ----------------
fprintf('\nROBUSTNESS (5 m step):\n');
for bScale = [0.9 1.1]
    tauScale = 1/bScale; Ascale = 1;
    out = sim(mdl); [os, ts, ess] = stepMetrics(out.tout, out.get('y'), rFinal);
    fprintf('  b x %.1f:  overshoot = %.2f %%,  settling = %.2f s,  final error = %.4f m\n', bScale, os, ts, ess);
end
for aScale = [0.9 1.1]
    tauScale = 1; Ascale = aScale;
    out = sim(mdl); [os, ts, ess] = stepMetrics(out.tout, out.get('y'), rFinal);
    fprintf('  A x %.1f:  overshoot = %.2f %%,  settling = %.2f s,  final error = %.4f m\n', aScale, os, ts, ess);
end
tauScale = 1; Ascale = 1;

%% ---------------- Test 3: output disturbance ----------------
% distSize is a PLACEHOLDER: set it from the simulator options.
distSize = 1; distTime = 40; tStop = 60;
out = sim(mdl); t = out.tout; y = out.get('y');
iPre = find(t < distTime, 1, 'last');
pre  = y(iPre);
bad  = find(abs(y(iPre:end) - pre) > 0.02*abs(pre), 1, 'last');
if isempty(bad), tRet = 0; else, tRet = t(iPre - 1 + bad) - distTime; end
fprintf('\nDISTURBANCE (%.2f m step): output back within 2%% of %.3f m after %.2f s (te = 10 s)\n', ...
    distSize, pre, tRet);
distSize = 0; tStop = 40;

%% ---------------- Test 4: 0.1 rad/s sinusoid ----------------
% sineAmp is a PLACEHOLDER: set it from the simulator options.
rFinal = 0; sineAmp = 1; tStop = 300;
out = sim(mdl); t = out.tout; y = out.get('y'); r = out.get('r');
lastTwoPeriods = t > tStop - 2*(2*pi/0.1);
maxErr = max(abs(r(lastTwoPeriods) - y(lastTwoPeriods)));
fprintf('\nSINUSOID (amplitude %.2f m, 0.1 rad/s): steady tracking error amplitude = %.3f m (limit 0.4 m)\n', ...
    sineAmp, maxErr);

%% ---------------- Local function ----------------
function [os, ts, ess] = stepMetrics(t, y, rf)
    os  = max(0, (max(y) - rf) / rf * 100);
    outside = find(abs(y - rf) > 0.02 * rf);
    if isempty(outside)
        ts = 0;
    else
        ts = t(min(outside(end) + 1, numel(t)));
    end
    ess = y(end) - rf;
end