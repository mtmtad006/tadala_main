%% EEE3094S Lab 2 - Controller simulation result for ONE chosen K
% Figure 1: step input + position response, labelled with the peak,
%           the settling time and the gain chosen.
% Figure 2: angle-of-attack command (controller output), labelled with K.
%
% Needs the Simulink model 'lab2_closed_loop' created by
% lab2_step2_closed_loop_sweep.m (same folder or on the path).

clear; clc; close all;

%% 1. Values (same names the Simulink model reads from the workspace)
A      = 14.1208;
tau    = 12.9898;
ks     = 0.6599;
Vhover = 2.5;      % hover voltage [V]
r_m    = 5;        % setpoint step [m]
K      = 0.0122;   % <-- your chosen controller gain (the ONLY gain used)
tStop  = 400;      % [s]

%% 2. Run the model once at this K
mdl = 'lab2_closed_loop';
if ~exist([mdl '.slx'], 'file')
    error('Model %s.slx not found. Run lab2_step2_closed_loop_sweep.m first.', mdl);
end
load_system(mdl);
set_param(mdl, 'StopTime', num2str(tStop));
out = sim(mdl);

y = out.get('y_pos');   t = y.Time;  pos = squeeze(y.Data);   % position [m]
u = out.get('u_cmd');   uc = squeeze(u.Data);                  % command before limiter [V]

setpoint = r_m * (t >= 0);          % step input [m]
u_sat    = min(max(uc, 0), 5);      % angle-of-attack command the cat receives [V]

%% 3. Metrics
[pk, ip] = max(pos);                 % peak of the response
tpk      = t(ip);
os       = max(0, (pk - r_m)/r_m*100);
errF     = abs(r_m - pos(end))/r_m*100;

idx = find(abs(pos - r_m) > 0.02*r_m, 1, 'last');    % last time outside the 2% band
if isempty(idx)
    ts = 0;
elseif idx >= numel(t)
    ts = NaN;                                        % never settled within tStop
else
    ts = t(idx+1);
end
fprintf('K = %.4g: peak = %.3f m at %.1f s, overshoot = %.1f %%, final error = %.2f %%, 2%% settling = %.1f s\n', ...
        K, pk, tpk, os, errF, ts);

%% 4. Figure 1: step input and position response
figure('Name','Step response','Color','w','Position',[100 100 850 500]);
hold on; grid on;
plot(t, setpoint, 'k--', 'LineWidth', 1.5);
plot(t, pos, 'b-', 'LineWidth', 2);
yline(1.02*r_m, ':', 'Color', [0.4 0.4 0.4]);
yline(0.98*r_m, ':', 'Color', [0.4 0.4 0.4]);

% Peak label
plot(tpk, pk, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
text(tpk + 8, pk + 0.15*r_m, sprintf('Peak = %.2f m\nt = %.1f s\nOvershoot = %.1f %%', pk, tpk, os), ...
     'Color', 'r', 'FontWeight', 'bold', 'BackgroundColor', 'w', 'EdgeColor', 'r');

% Settling-time label
if ~isnan(ts)
    ys = interp1(t, pos, ts);
    plot([ts ts], [0 ys], 'g-', 'LineWidth', 1.5);
    plot(ts, ys, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    text(ts + 8, 0.35*r_m, sprintf('2%% settling time\n= %.1f s', ts), ...
         'Color', [0 0.5 0], 'FontWeight', 'bold', 'BackgroundColor', 'w', 'EdgeColor', [0 0.5 0]);
else
    text(0.55*tStop, 0.35*r_m, 'Did not settle within simulation time', 'Color', [0 0.5 0]);
end

% Gain chosen
text(0.02, 0.97, sprintf('Gain chosen: K = %.4g', K), 'Units', 'normalized', ...
     'VerticalAlignment', 'top', 'FontWeight', 'bold', 'BackgroundColor', 'w', 'EdgeColor', 'k');

xlabel('Time (s)'); ylabel('Position (m)');
title(sprintf('Closed-loop step response (K = %.4g)', K));
legend('Step input (5 m setpoint)', 'Position response', '2% band', 'Location', 'southeast');
xlim([0 tStop]); ylim([0 1.4*r_m]);

%% 5. Figure 2: angle-of-attack command
figure('Name','Angle of attack','Color','w','Position',[150 150 850 450]);
hold on; grid on;
plot(t, u_sat, 'm-', 'LineWidth', 2);
yline(Vhover, 'k--', 'Hover (2.5 V = gravity cancelled)');

% Label the starting value and the lowest dip
plot(0, u_sat(1), 'ko', 'MarkerFaceColor', 'm', 'MarkerSize', 7);
text(8, u_sat(1), sprintf('Start = %.4f V', u_sat(1)), 'VerticalAlignment', 'bottom');
[umin, iu] = min(u_sat);
if umin < Vhover
    plot(t(iu), umin, 'ko', 'MarkerFaceColor', 'm', 'MarkerSize', 7);
    text(t(iu) + 8, umin, sprintf('Lowest = %.4f V (t = %.1f s)', umin, t(iu)), 'VerticalAlignment', 'top');
end

text(0.98, 0.97, sprintf('Gain chosen: K = %.4g', K), 'Units', 'normalized', ...
     'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
     'FontWeight', 'bold', 'BackgroundColor', 'w', 'EdgeColor', 'k');

xlabel('Time (s)'); ylabel('Angle-of-attack command (V)');
title(sprintf('Controller output: angle-of-attack command (K = %.4g)', K));
xlim([0 tStop]);