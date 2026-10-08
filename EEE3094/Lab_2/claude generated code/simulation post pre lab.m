%% EEE3094S Lab 2 - closed-loop Kitticopter model (proportional control)
% Run this script. Section 2 builds and saves 'kitticopter_closedloop.slx' in the
% current folder. Section 3 sweeps Kp, robustness cases and level-shift error.
% NOTE: written without access to MATLAB/Simulink, so it is untested - if a block
% name errors on your release, tell me the message and I will fix it.

%% 1. Parameters (EDIT THESE WITH YOUR OWN VALUES)
A     = 14.1208;  % plant DC gain, (m/s) per volt above hover  (your confirmed value)
T     = 12.9898;  % plant time constant, s                     (your confirmed value)
k     = 0.6599;   % sensor gain, V per m                       (your confirmed value)
Vh    = 2.5;      % hover voltage, V (assumed; replace with your measured value)
Kp    = 0.010;    % proportional gain, V/V
dV    = 0;        % error in the level-shift voltage, V
Xstep = 5;        % setpoint step, m

%% 2. Build the model
mdl = 'kitticopter_closedloop';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl); open_system(mdl);

blk = @(type,name,pos) add_block(type,[mdl '/' name],'Position',pos);
blk('simulink/Sources/Step','Setpoint_m',[20 95 60 125]);
set_param([mdl '/Setpoint_m'],'Time','0','Before','0','After','Xstep');
blk('simulink/Math Operations/Gain','k_in',[90 95 140 125]);
set_param([mdl '/k_in'],'Gain','k');
blk('simulink/Math Operations/Sum','Error_sum',[170 95 200 125]);
set_param([mdl '/Error_sum'],'Inputs','+-');
blk('simulink/Math Operations/Gain','Kp_gain',[230 95 280 125]);
set_param([mdl '/Kp_gain'],'Gain','Kp');
blk('simulink/Math Operations/Sum','Level_shift',[310 95 340 125]);
set_param([mdl '/Level_shift'],'Inputs','++');
blk('simulink/Sources/Constant','Level_shift_V',[300 40 350 65]);
set_param([mdl '/Level_shift_V'],'Value','Vh+dV');
blk('simulink/Discontinuities/Saturation','Sat_0_5V',[370 95 420 125]);
set_param([mdl '/Sat_0_5V'],'UpperLimit','5','LowerLimit','0');
blk('simulink/Math Operations/Sum','Remove_hover',[450 95 480 125]);
set_param([mdl '/Remove_hover'],'Inputs','+-');
blk('simulink/Sources/Constant','Hover_V',[440 160 490 185]);
set_param([mdl '/Hover_V'],'Value','Vh');
blk('simulink/Continuous/Transfer Fcn','Speed_plant',[510 90 600 130]);
set_param([mdl '/Speed_plant'],'Numerator','[A]','Denominator','[T 1]');
blk('simulink/Continuous/Integrator','Position',[630 95 670 125]);
set_param([mdl '/Position'],'InitialCondition','0');
blk('simulink/Math Operations/Gain','k_sensor',[400 230 450 260]);
set_param([mdl '/k_sensor'],'Gain','k','Orientation','left');
blk('simulink/Sinks/To Workspace','y_out',[740 60 810 90]);
set_param([mdl '/y_out'],'VariableName','y_sim','SaveFormat','Timeseries');
blk('simulink/Sinks/To Workspace','u_out',[440 40 510 65]);
set_param([mdl '/u_out'],'VariableName','u_sim','SaveFormat','Timeseries');
blk('simulink/Sinks/Scope','Scope',[740 120 780 150]);

L = @(a,b) add_line(mdl,a,b,'autorouting','on');
L('Setpoint_m/1','k_in/1');     L('k_in/1','Error_sum/1');
L('Error_sum/1','Kp_gain/1');   L('Kp_gain/1','Level_shift/1');
L('Level_shift_V/1','Level_shift/2');
L('Level_shift/1','Sat_0_5V/1');
L('Sat_0_5V/1','Remove_hover/1'); L('Sat_0_5V/1','u_out/1');
L('Hover_V/1','Remove_hover/2');
L('Remove_hover/1','Speed_plant/1');
L('Speed_plant/1','Position/1');
L('Position/1','k_sensor/1');   L('Position/1','y_out/1');  L('Position/1','Scope/1');
L('k_sensor/1','Error_sum/2');

set_param(mdl,'StopTime','400','Solver','ode45','MaxStep','0.1');
save_system(mdl);
fprintf('Saved %s.slx\n',mdl);

%% 3a. Kp sweep (5 m step)
Kp_list = [0.004 0.006 0.008 0.010 0.012 0.016];
for i = 1:numel(Kp_list)
    in(i) = Simulink.SimulationInput(mdl);
    in(i) = in(i).setVariable('Kp',Kp_list(i));
end
out = sim(in,'ShowProgress','off');
figure; hold on; grid on
fprintf('\nKp sweep:\n  Kp      overshoot%%   final error (m)   t_settle 2%% (s)\n');
for i = 1:numel(Kp_list)
    y = out(i).y_sim;  m = stepmetrics(y.Data,y.Time,Xstep);
    plot(y.Time,y.Data,'DisplayName',sprintf('K_p = %.3f',Kp_list(i)));
    fprintf('  %.3f   %8.1f      %8.3f          %8.1f\n',Kp_list(i),m(1),m(2),m(3));
end
yline(Xstep,'--k','Setpoint','HandleVisibility','off');
xlabel('Time (s)'); ylabel('Position (m)'); title('Step response for several K_p'); legend show

%% 3b. Robustness: b +/-10% (with c = A/T held fixed) and gain 0.82x-1.22x
Kp0 = 0.010;  sb = [0.9 1 1.1];  sg = [0.82 1 1.22];  c = A/T;  n = 0;
for i = 1:3
    for j = 1:3
        n = n+1;  Ti = T/sb(i);  Ai = c*Ti;
        rb(n) = Simulink.SimulationInput(mdl);
        rb(n) = rb(n).setVariable('T',Ti).setVariable('A',Ai).setVariable('Kp',Kp0*sg(j));
        lab{n} = sprintf('b x%.1f, gain x%.2f',sb(i),sg(j));
    end
end
ro = sim(rb,'ShowProgress','off');
fprintf('\nRobustness (nominal Kp = %.3f):\n',Kp0);
for n = 1:9
    y = ro(n).y_sim;  m = stepmetrics(y.Data,y.Time,Xstep);
    fprintf('  %-22s overshoot %5.1f %%   final error %6.3f m\n',lab{n},m(1),m(2));
end

%% 3c. Level-shift error: final position offset versus dV
dV_list = [0 0.001 0.005 0.010];
for i = 1:numel(dV_list)
    rd(i) = Simulink.SimulationInput(mdl);
    rd(i) = rd(i).setVariable('Kp',Kp0).setVariable('dV',dV_list(i));
end
rdo = sim(rd,'ShowProgress','off');
fprintf('\nLevel-shift error (Kp = %.3f):\n',Kp0);
for i = 1:numel(dV_list)
    y = rdo(i).y_sim;  m = stepmetrics(y.Data,y.Time,Xstep);
    fprintf('  dV = %5.3f V -> final error %7.3f m\n',dV_list(i),m(2));
end

%% Local function
function m = stepmetrics(y,t,X)
    os   = max(0,(max(y)-X)/X*100);          % overshoot, %
    fe   = X - y(end);                       % final error, m (positive = below target)
    idx  = find(abs(y-X) > 0.02*X,1,'last'); % last time outside the 2 % band
    if isempty(idx), ts = 0; elseif idx == numel(t), ts = NaN; else, ts = t(idx+1); end
    m = [os fe ts];
end