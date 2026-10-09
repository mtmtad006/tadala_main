%% EEE3094S Lab 2 - STEP 1 (Simulink version): open-loop model validation
% Builds a Simulink model programmatically, runs it, and compares the
% result with the Session 1 measured data. NO feedback in this model.
%
% Needs: Simulink (Control System Toolbox no longer required here).

clear; clc; close all;

%% 1. Your model values
A      = 14.1208;    % velocity gain  [(m/s) per volt of net input]
tau    = 12.9898;    % time constant  [s]
ks     = 0.6599;     % sensor constant [V/m] (data check only)
Vhover = 2.5;        % hover voltage [V]

%% 2. Load the Session 1 data
dataFile = 'C:\Users\Student\Documents\MATLAB\tadala_main\EEE3094\Lab_2\Final_HelicopterTESTData.csv';
T  = readtable(dataFile);
t  = T{:,1};
u  = T{:,2};
yV = T{:,3};
ym = T{:,4};

%% 3. Find the step
idx      = find(u > (Vhover + 5)/2, 1, 'first');
t0       = t(idx);
stepSize = mean(u(idx:end)) - Vhover;
fprintf('Step applied at t0 = %.2f s, net step size = %.3f V\n', t0, stepSize);

mask     = t >= t0;
tt       = t(mask) - t0;
pos_meas = ym(mask);

%% 4. Measured velocity
vel_all  = movmean(gradient(ym, t), 5);
vel_meas = vel_all(mask);

% Measured data packaged for Simulink "From Workspace" blocks: [time, value]
posMeasData = [tt(:), pos_meas(:)];
velMeasData = [tt(:), vel_meas(:)];

%% 5. Build the Simulink model
mdl = 'kiLicopter_openloop';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

% --- Blocks -------------------------------------------------------------
% Input voltage: sits at Vhover, then steps up by stepSize at t = 0
add_block('simulink/Sources/Step', [mdl '/InputVoltage'], ...
    'Time', '0', 'Before', num2str(Vhover), ...
    'After', num2str(Vhover + stepSize), ...
    'Position', [30 80 90 110]);

% Hover voltage (cancels gravity) subtracted to get the NET input
add_block('simulink/Sources/Constant', [mdl '/Vhover'], ...
    'Value', num2str(Vhover), ...
    'Position', [30 150 90 180]);

add_block('simulink/Math Operations/Sum', [mdl '/NetInput'], ...
    'Inputs', '+-', 'IconShape', 'round', ...
    'Position', [140 90 170 120]);

% Velocity = A/(tau*s + 1) * net input
add_block('simulink/Continuous/Transfer Fcn', [mdl '/VelocityModel'], ...
    'Numerator', num2str(A), ...
    'Denominator', ['[' num2str(tau) ' 1]'], ...
    'Position', [220 80 320 130]);

% Position = integral of velocity
add_block('simulink/Continuous/Integrator', [mdl '/PositionIntegrator'], ...
    'InitialCondition', '0', ...
    'Position', [400 90 440 120]);

% Measured data (for side-by-side scopes)
add_block('simulink/Sources/From Workspace', [mdl '/MeasVelocity'], ...
    'VariableName', 'velMeasData', ...
    'OutputAfterFinalValue', 'Holding final value', ...
    'Position', [220 200 320 230]);
add_block('simulink/Sources/From Workspace', [mdl '/MeasPosition'], ...
    'VariableName', 'posMeasData', ...
    'OutputAfterFinalValue', 'Holding final value', ...
    'Position', [400 260 500 290]);

% Scopes (input 1 = model, input 2 = measured)
add_block('simulink/Sinks/Scope', [mdl '/VelocityScope'], ...
    'NumInputPorts', '2', 'Position', [560 80 600 140]);
add_block('simulink/Sinks/Scope', [mdl '/PositionScope'], ...
    'NumInputPorts', '2', 'Position', [560 200 600 260]);

% Log model outputs to the workspace
add_block('simulink/Sinks/To Workspace', [mdl '/LogVel'], ...
    'VariableName', 'vel_sim', 'SaveFormat', 'Timeseries', ...
    'Position', [400 20 470 50]);
add_block('simulink/Sinks/To Workspace', [mdl '/LogPos'], ...
    'VariableName', 'pos_sim', 'SaveFormat', 'Timeseries', ...
    'Position', [500 150 570 180]);

% --- Wiring -------------------------------------------------------------
add_line(mdl, 'InputVoltage/1',      'NetInput/1',          'autorouting', 'on');
add_line(mdl, 'Vhover/1',            'NetInput/2',          'autorouting', 'on');
add_line(mdl, 'NetInput/1',          'VelocityModel/1',     'autorouting', 'on');
add_line(mdl, 'VelocityModel/1',     'PositionIntegrator/1','autorouting', 'on');

add_line(mdl, 'VelocityModel/1',     'VelocityScope/1',     'autorouting', 'on');
add_line(mdl, 'MeasVelocity/1',      'VelocityScope/2',     'autorouting', 'on');
add_line(mdl, 'VelocityModel/1',     'LogVel/1',            'autorouting', 'on');

add_line(mdl, 'PositionIntegrator/1','PositionScope/1',     'autorouting', 'on');
add_line(mdl, 'MeasPosition/1',      'PositionScope/2',     'autorouting', 'on');
add_line(mdl, 'PositionIntegrator/1','LogPos/1',            'autorouting', 'on');

% --- Solver settings ----------------------------------------------------
set_param(mdl, 'StopTime', num2str(tt(end)), ...
               'Solver', 'ode45', ...
               'MaxStep', '0.1');

save_system(mdl, fullfile(pwd, [mdl '.slx']));
fprintf('Simulink model saved as %s.slx\n', mdl);

%% 6. Run the simulation
simOut = sim(mdl);
vel_sim = simOut.vel_sim;     % timeseries
pos_sim = simOut.pos_sim;

% Resample onto the measured time vector so we can compute errors
vel_model = interp1(vel_sim.Time, squeeze(vel_sim.Data), tt, 'linear', 'extrap');
pos_model = interp1(pos_sim.Time, squeeze(pos_sim.Data), tt, 'linear', 'extrap');

rmse_vel = sqrt(mean((vel_model(:) - vel_meas(:)).^2));
rmse_pos = sqrt(mean((pos_model(:) - pos_meas(:)).^2));
fprintf('Velocity RMSE = %.3f m/s  (model final value = %.2f m/s)\n', rmse_vel, A*stepSize);
fprintf('Position RMSE = %.3f m\n', rmse_pos);

ok = yV > 0 & yV < 9.9;
fprintf('Sensor check: mean(Output_V / Output_m) = %.4f   (your ks = %.4f)\n', ...
        mean(yV(ok) ./ ym(ok)), ks);

%% 7. Plots (from the Simulink results)
figure('Name','Velocity validation (Simulink)','Color','w');
plot(tt, vel_meas, 'b.', 'MarkerSize', 8); hold on;
plot(vel_sim.Time, squeeze(vel_sim.Data), 'r-', 'LineWidth', 1.8);
yline(A*stepSize, 'k--', 'Model final value');
grid on; xlabel('Time since step (s)'); ylabel('Velocity (m/s)');
title('Open-loop velocity: measured vs Simulink model');
legend('Measured (derivative of position)', 'Simulink model', 'Location','southeast');

figure('Name','Position validation (Simulink)','Color','w');
plot(tt, pos_meas, 'b.', 'MarkerSize', 8); hold on;
plot(pos_sim.Time, squeeze(pos_sim.Data), 'r-', 'LineWidth', 1.8);
grid on; xlabel('Time since step (s)'); ylabel('Position (m)');
title('Open-loop position: measured vs Simulink model');
legend('Measured', 'Simulink model', 'Location','northwest');

% Optional: open the diagram
open_system(mdl);