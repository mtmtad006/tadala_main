%% EEE3094S Lab 2 - System identification plots (Session 1 step test)
% Plots position vs time and velocity vs time from t = 17 s onwards,
% marks the 63.2% point and the steady-state value on the velocity plot,
% and extracts the gain A and time constant tau.
%
% No toolboxes needed (uses base MATLAB only; yline/xline need R2018b+).

clear; clc; close all;

%% 1. Settings
dataFile = 'C:\Users\Student\Documents\MATLAB\tadala_main\EEE3094\Lab_2\Final_HelicopterTESTData.csv';
tStart   = 17;       % plot/analyse from this time [s]
Vhover   = 2.5;      % hover voltage [V] (net step = input - Vhover)
A_pre    = 14.1208;  % your earlier values, for comparison only
tau_pre  = 12.9898;

%% 2. Load data and keep t >= 17 s
T  = readtable(dataFile);
t  = T{:,1};  u = T{:,2};  ym = T{:,4};        % time, input V, position m

vel_all = movmean(gradient(ym, t), 5);         % velocity = d(position)/dt, lightly smoothed

m   = t >= tStart;
tp  = t(m);  up = u(m);  pos = ym(m);  vel = vel_all(m);

%% 3. Step time and net step size
iStep    = find(up > (Vhover + 5)/2, 1, 'first');
tStep    = tp(iStep);
stepSize = mean(up(iStep:end)) - Vhover;
fprintf('Step detected at t = %.2f s, net step = %.3f V\n', tStep, stepSize);

%% 4. Fit a first-order response  v(t) = Vf*(1 - exp(-(t - t0)/tau))
% The recording stops before the velocity settles, so the steady-state value
% is EXTRAPOLATED from this fit (not read directly from the last sample).
model = @(p, tt) p(1) * (1 - exp(-(tt - p(3))/p(2))) .* (tt >= p(3));
fitMask = tp >= tStep + 1;                     % skip the smoothed step edge
cost = @(p) sum((model(p, tp(fitMask)) - vel(fitMask)).^2);
p0   = [1.3*max(vel), 10, tStep];
opts = optimset('Display','off','TolX',1e-8,'TolFun',1e-10,'MaxFunEvals',5000,'MaxIter',5000);
p    = fminsearch(cost, p0, opts);

Vf      = p(1);          % steady-state velocity [m/s]
tau_fit = p(2);          % time constant [s]
t0_fit  = p(3);          % step time from the fit [s]
Gain    = Vf / stepSize; % A = steady-state velocity / net input step

%% 5. 63.2% point read directly from the data
V63 = 0.632 * Vf;
k   = find(tp > t0_fit & vel >= V63, 1, 'first');
if ~isempty(k) && k > 1
    t63 = interp1(vel(k-1:k), tp(k-1:k), V63);   % linear interpolation
else
    t63 = t0_fit + tau_fit;                      % fall back on the fit
end
tau_data = t63 - t0_fit;

fprintf('\n--- Results ---\n');
fprintf('Steady-state velocity Vf = %.3f m/s (extrapolated from fit)\n', Vf);
fprintf('Gain A = Vf / step       = %.4f (m/s)/V   (pre-lab: %.4f)\n', Gain, A_pre);
fprintf('tau from fit             = %.4f s\n', tau_fit);
fprintf('tau from 63.2%% crossing  = %.4f s   (pre-lab: %.4f)\n', tau_data, tau_pre);
fprintf('Last measured velocity   = %.2f m/s (%.0f%% of Vf)\n', vel(end), 100*vel(end)/Vf);

%% 6. Plot 1: position vs time
figure('Name','Position vs time','Color','w');
plot(tp, pos, 'b-', 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('Position (m)');
title('Session 1 step test: kiLicopter position vs time (open loop)');
xlim([tStart tp(end)]);

%% 7. Plot 2: velocity vs time with 63.2% and steady-state markers
tEnd = t0_fit + 5*tau_fit;                     % show ~5 time constants
tFit = linspace(t0_fit, tEnd, 500);

figure('Name','Velocity vs time','Color','w'); hold on; grid on;
h1 = plot(tp, vel, 'b.', 'MarkerSize', 9);
h2 = plot(tFit, model(p, tFit), 'r--', 'LineWidth', 1.5);

% Steady-state value (the gain)
h3 = yline(Vf, 'k-', sprintf('Steady state = %.2f m/s', Vf), ...
    'LabelHorizontalAlignment','left','LineWidth',1.2);

% 63.2% point: horizontal and vertical guides + marker
plot([tStart t63], [V63 V63], 'g--', 'LineWidth', 1.2);
plot([t63 t63],    [0 V63],   'g--', 'LineWidth', 1.2);
h4 = plot(t63, V63, 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 9);
text(t63 + 1, V63 - 2.5, sprintf('63.2%% of final = %.2f m/s\nt = %.2f s  (\\tau = %.2f s after step)', ...
    V63, t63, tau_data), 'FontSize', 9);

xlabel('Time (s)'); ylabel('Velocity (m/s)');
title(sprintf('Session 1 step test: velocity vs time  (Gain A = %.3f (m/s)/V)', Gain));
legend([h1 h2 h4], {'Measured velocity (derivative of position)', ...
    'First-order fit (dashed = extrapolated beyond data)', '63.2% point'}, ...
    'Location','southeast');
xlim([tStart tEnd]); ylim([0 1.15*Vf]);