%% EEE3094S Lab 2 - STEP 2: Closed-loop Simulink model + K sweep
% This script (1) BUILDS the Simulink model of Figure 1 (appendix) for you,
% (2) runs it once for every K in your table, and (3) overlays the results.
%
% Needs: Simulink.  Run it from any folder; it creates 'lab2_closed_loop.slx'.

clear; clc; close all;

%% 1. Model values (from Session 1)
A      = 14.1208;    % velocity gain [(m/s) per volt of net input]
tau    = 12.9898;    % time constant [s]
ks     = 0.6599;     % sensor constant [V/m]
Vhover = 2.5;        % hover / level-shift voltage [V]

%% 2. Test settings
r_m   = 5;           % setpoint step size [m] (the demo uses a 5 m step)
tStop = 400;         % simulation length [s] - the loop is slow, keep this long

%% 3. YOUR TABLE OF K VALUES - edit this list
Kvals = [0.002 0.005 0.010 0.016 0.030 0.050];
% Or load them from a file instead (one column of K values):
% Kvals = readmatrix('C:\path\to\your_K_table.csv')';

%% 4. Build the Simulink model
mdl = 'lab2_closed_loop';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

% --- helper to place blocks in a row ---
pos = @(x,y,w,h) [x y x+w y+h];

% Blocks (names match the appendix Figure 1)
add_block('simulink/Sources/Step',            [mdl '/Setpoint_m'], ...
    'Time','0','Before','0','After','r_m','Position',pos(30,100,40,30));
add_block('simulink/Math Operations/Gain',    [mdl '/ks_input'], ...
    'Gain','ks','Position',pos(110,100,40,30));
add_block('simulink/Math Operations/Sum',     [mdl '/Error'], ...
    'Inputs','+-','Position',pos(190,100,30,30));
add_block('simulink/Math Operations/Gain',    [mdl '/K_controller'], ...
    'Gain','K','Position',pos(260,100,50,30));
add_block('simulink/Sources/Constant',        [mdl '/Hover_levelshift'], ...
    'Value','Vhover','Position',pos(260,180,50,30));
add_block('simulink/Math Operations/Sum',     [mdl '/LevelShift'], ...
    'Inputs','++','Position',pos(350,100,30,30));
add_block('simulink/Discontinuities/Saturation',[mdl '/Limit_0to5V'], ...
    'UpperLimit','5','LowerLimit','0','Position',pos(420,100,40,30));
add_block('simulink/Sources/Constant',        [mdl '/Gravity_offset'], ...
    'Value','Vhover','Position',pos(420,180,50,30));
add_block('simulink/Math Operations/Sum',     [mdl '/NetThrust'], ...
    'Inputs','+-','Position',pos(510,100,30,30));
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Kitticopter'], ...
    'Numerator','[A]','Denominator','[tau 1 0]','Position',pos(580,95,90,40));
add_block('simulink/Math Operations/Gain',    [mdl '/ks_sensor'], ...
    'Gain','ks','Orientation','left','Position',pos(400,260,40,30));
add_block('simulink/Sinks/To Workspace',      [mdl '/Log_position'], ...
    'VariableName','y_pos','SaveFormat','Timeseries','Position',pos(730,100,60,30));
add_block('simulink/Sinks/To Workspace',      [mdl '/Log_command'], ...
    'VariableName','u_cmd','SaveFormat','Timeseries','Position',pos(420,20,60,30));

% Wires
L = @(a,b) add_line(mdl, a, b, 'autorouting','on');
L('Setpoint_m/1',      'ks_input/1');
L('ks_input/1',        'Error/1');
L('Error/1',           'K_controller/1');
L('K_controller/1',    'LevelShift/1');
L('Hover_levelshift/1','LevelShift/2');
L('LevelShift/1',      'Limit_0to5V/1');
L('LevelShift/1',      'Log_command/1');
L('Limit_0to5V/1',     'NetThrust/1');
L('Gravity_offset/1',  'NetThrust/2');
L('NetThrust/1',       'Kitticopter/1');
L('Kitticopter/1',     'Log_position/1');
L('Kitticopter/1',     'ks_sensor/1');
L('ks_sensor/1',       'Error/2');

set_param(mdl, 'StopTime', num2str(tStop), 'Solver', 'ode45', 'MaxStep', '0.5');
save_system(mdl);
fprintf('Model "%s" built and saved. Open it to inspect the blocks.\n', mdl);

%% 5. Sweep K
nK  = numel(Kvals);
res = struct('t',cell(1,nK),'y',[],'u',[]);

for i = 1:nK
    K  = Kvals(i);
    assignin('base','K',K);                   % Simulink reads K from workspace
    out = sim(mdl);
    y = out.get('y_pos');  u = out.get('u_cmd');
    res(i).t = y.Time;  res(i).y = squeeze(y.Data);  res(i).u = squeeze(u.Data);
end

%% 6. Metrics for each K
Overshoot_pct = zeros(nK,1); FinalErr_pct = zeros(nK,1);
Settle2pct_s  = zeros(nK,1); PeakCmd_V    = zeros(nK,1);
Zeta_theory   = zeros(nK,1); OS_theory_pct = zeros(nK,1);

for i = 1:nK
    t = res(i).t; y = res(i).y;
    Overshoot_pct(i) = max(0, (max(y) - r_m)/r_m*100);
    FinalErr_pct(i)  = abs(r_m - y(end))/r_m*100;
    idx = find(abs(y - r_m) > 0.02*r_m, 1, 'last');
    if isempty(idx), Settle2pct_s(i) = 0;
    elseif idx == numel(t), Settle2pct_s(i) = NaN;      % never settled in tStop
    else, Settle2pct_s(i) = t(idx+1); end
    PeakCmd_V(i) = max(res(i).u);

    % Hand calculation to cross-check the simulation:
    z = 1/(2*sqrt(tau*Kvals(i)*ks*A));
    Zeta_theory(i) = z;
    if z < 1, OS_theory_pct(i) = 100*exp(-pi*z/sqrt(1-z^2)); end
end

Results = table(Kvals(:), Zeta_theory, OS_theory_pct, Overshoot_pct, ...
    Settle2pct_s, FinalErr_pct, PeakCmd_V, 'VariableNames', ...
    {'K','Zeta_theory','Overshoot_theory_pct','Overshoot_sim_pct', ...
     'Settling_2pct_s','FinalError_pct','PeakCommand_V'});
disp(Results);
fprintf('Spec check: overshoot < 30%%, final error < 8%%. PeakCommand must stay <= 5 V (else it saturates).\n');

%% 7. Plots
figure('Name','K sweep - position','Color','w'); hold on; grid on;
for i = 1:nK
    plot(res(i).t, res(i).y, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('K = %.3g', Kvals(i)));
end
yline(r_m,       'k-',  'Setpoint 5 m');
yline(1.3*r_m,   'r--', '30% overshoot limit');
yline(0.92*r_m,  'g--', '8% error band');
xlabel('Time (s)'); ylabel('Position (m)');
title('Closed-loop 5 m step response for different K');
legend('Location','southeast');

figure('Name','K sweep - command voltage','Color','w'); hold on; grid on;
for i = 1:nK
    plot(res(i).t, res(i).u, 'LineWidth', 1.2, ...
        'DisplayName', sprintf('K = %.3g', Kvals(i)));
end
yline(5, 'r--', '5 V limit'); yline(0, 'r--', '0 V limit');
xlabel('Time (s)'); ylabel('Voltage into saturation (V)');
title('Command voltage (check it does not hit 0 V or 5 V)');
legend('Location','best');