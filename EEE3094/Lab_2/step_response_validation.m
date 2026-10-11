%% EEE3094S Lab 2 - Model validation overlay
% Overlays the step response of the identified model on the recorded data,
% for velocity and for position. Set runChoice to 2 (second test) or 1 (first test).
%
% The model response is the exact solution of the identified transfer function
% for a step of size du applied at t0, so no toolbox is needed:
%   velocity: v(t) = A*du*(1 - exp(-t/tau))
%   position: y(t) = A*du*( t - tau*(1 - exp(-t/tau)) )
% Needs R2018b+ (yline).

clear; clc; close all;

%% 1. Identified parameters and settings
A      = 14.1208;     % gain [(m/s)/V]
tau    = 12.9898;     % time constant [s]
ks     = 0.6599;      % sensor constant [V/m] (not used in this plot)
Vhover = 2.5;         % hover voltage [V]
Vstep  = 5.0;         % step level [V]
du     = Vstep - Vhover;   % net step = 2.5 V

runChoice = 2;        % 2 = second test (default), 1 = first test
folder = 'C:\Users\Student\Documents\MATLAB\tadala_main\EEE3094\Lab_2\';   % <-- change to your folder
files  = {'HelicopterData_MTMTAD006.csv', 'Final_HelicopterTESTData.csv'};
tEndAbs = [80, inf];  % end of the compared window [s]: run 1 is ~159 s long, run 2 ends at 35.3 s

%% 2. Load the recorded data
T = readtable([folder files{runChoice}]);
t = T{:,1};            % time [s]
u = T{:,2};            % input [V]
y = T{:,4};            % position [m]  (Output(m); the voltage column clips at 10 V)

i0 = find(u > 3.0, 1, 'first');    % step start: first sample above 3 V
t0 = t(i0);
fprintf('Run %d: step start t0 = %.1f s\n', runChoice, t0);

%% 3. Velocity from position (same forward difference as in the report)
v = [diff(y) ./ diff(t); NaN];

mask   = t >= t0 & t <= tEndAbs(runChoice) & ~isnan(v);
tt     = t(mask) - t0;      % time since the step [s]
v_meas = v(mask);
y_meas = y(mask);

%% 4. Model at the same sample times, and on a fine grid for smooth lines
v_mod = A*du*(1 - exp(-tt/tau));
y_mod = A*du*(tt - tau*(1 - exp(-tt/tau)));

tf     = linspace(0, max(tt), 800);
v_line = A*du*(1 - exp(-tf/tau));
y_line = A*du*(tf - tau*(1 - exp(-tf/tau)));

%% 5. How close are they?
rmse_v = sqrt(mean((v_mod - v_meas).^2));
rmse_y = sqrt(mean((y_mod - y_meas).^2));
fprintf('Velocity RMSE = %.4f m/s (final value %.2f m/s)\n', rmse_v, A*du);
fprintf('Position RMSE = %.3f m (position at end of window = %.1f m)\n', rmse_y, y_meas(end));

%% 6. Velocity overlay
figure('Name','Velocity validation','Color','w'); hold on; grid on;
plot(tt, v_meas, 'b.', 'MarkerSize', 9);
plot(tf, v_line, 'r-', 'LineWidth', 1.8);
yline(0.632*A*du, 'k:', '63.2% of final value', 'LabelHorizontalAlignment', 'left');
xline(tau, 'k:', sprintf('\\tau = %.4f s', tau), 'LabelOrientation', 'horizontal');
xlabel('Time since step (s)'); ylabel('Velocity (m/s)');
title(sprintf('Velocity: identified model vs recorded (run %d), RMSE = %.3f m/s', runChoice, rmse_v));
legend('Recorded (derivative of position)', ...
       sprintf('Model: %.4f/(%.4f s + 1) \\times %.1f V', A, tau, du), 'Location', 'southeast');

%% 7. Position overlay
figure('Name','Position validation','Color','w'); hold on; grid on;
plot(tt, y_meas, 'b.', 'MarkerSize', 9);
plot(tf, y_line, 'r-', 'LineWidth', 1.8);
xlabel('Time since step (s)'); ylabel('Position (m)');
title(sprintf('Position: identified model vs recorded (run %d), RMSE = %.2f m', runChoice, rmse_y));
legend('Recorded', 'Model: A/(s(\tau s + 1)) \times 2.5 V', 'Location', 'northwest');