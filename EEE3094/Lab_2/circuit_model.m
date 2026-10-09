%% EEE3094S Lab 2 - Simulink model of the ACTUAL two-op-amp controller circuit
% Builds 'lab2_circuit_model.slx' from the real component values, runs a 5 m
% step, and compares it with the ideal-gain model (K = 0.0122).
%
% CIRCUIT (see the breakdown in the chat):
%   Stage 1 (U1) differential amp : e1 = g_r1*Vref - g_f1*Vfb      (subtract + gain K)
%   Stage 2 (U2) differential amp : u  = e1 + level shift (~2.5 V) (add hover voltage)
%   u then goes to the lab PC (ADC) which clips it to 0-5 V.
%
% Needs: Simulink, Control System Toolbox (only for the ideal-model overlay).

clear; clc; close all;

%% 1. Plant and test settings (from Session 1)
A = 14.1208;  tau = 12.9898;  ks = 0.6599;
Vhover = 2.5;          % true hover voltage the cat needs [V]
r_m    = 5;            % setpoint step [m]
tStop  = 400;          % [s]
Vrail  = 13.5;         % assumed op-amp output swing with +/-15 V supplies [V] (CHECK datasheet)

%% 2. COMPONENT VALUES (nominal) - edit to match the parts you actually use
% Stage 1: differential amplifier with gain K = Rb/Ra = 1k/82k = 0.0122
Ra = 82e3;   % Vfb  -> inverting input
Rb = 1e3;    % feedback resistor (output -> inverting input)
Rc = 82e3;   % Vref -> non-inverting input
Rd = 1e3;    % non-inverting input -> ground
% Stage 2: unity-gain differential amplifier (adds the hover level)
Re  = 47e3;  % -Vlevel node -> inverting input
Rf2 = 47e3;  % feedback resistor
Rg  = 47e3;  % e1 -> non-inverting input
Rh  = 47e3;  % non-inverting input -> ground
% Level-shift divider from the -15 V rail (gives about -2.5 V)
Rtop  = 10e3;   % fixed resistor to -15 V
Rbot  = 2.2e3;  % fixed resistor to ground
% Rtrim (5 kOhm multi-turn trimmer, in series with Rtop) is solved below

%% 3. OPTIONAL: random 10% tolerances (set useTolerance = true to try it)
useTolerance = false;     % false = ideal nominal resistors
retrimHover  = true;      % true = trimmer is re-tuned for 2.5 V (what you do in the lab)
tolPct       = 10;        % resistor tolerance [%]
Vos1 = 0;  Vos2 = 0;      % op-amp input offset voltages [V] (put datasheet value here)

if useTolerance
    jitter = @() 1 + (tolPct/100)*(2*rand - 1);
    Ra=Ra*jitter(); Rb=Rb*jitter(); Rc=Rc*jitter(); Rd=Rd*jitter();
    Re=Re*jitter(); Rf2=Rf2*jitter(); Rg=Rg*jitter(); Rh=Rh*jitter();
    Rtop=Rtop*jitter(); Rbot=Rbot*jitter();
end

%% 4. Circuit equations
% Stage 1
g_f1 = Rb/Ra;                              % gain on the feedback voltage
g_r1 = (Rd/(Rc+Rd))*(1 + Rb/Ra);           % gain on the reference voltage
Voff1 = Vos1*(1 + Rb/Ra);                  % offset at stage-1 output
% Stage 2
g_p2 = (Rh/(Rg+Rh))*(1 + Rf2/Re);          % gain on e1
g_n2 = Rf2/Re;                             % gain on the level-shift node
% Level-shift output (includes loading of the divider by Re)
levelOut = @(Rt) g_n2 * (15*Rbot/(Rtop+Rt+Rbot)) * Re / (Re + (Rbot*(Rtop+Rt))/(Rtop+Rt+Rbot));
if retrimHover
    Rtrim = fzero(@(Rt) levelOut(Rt) - Vhover, [0 5e3]);   % trimmer setting for exactly 2.5 V
else
    Rtrim = 1e3;                                            % untrimmed guess
end
Vlev_out = levelOut(Rtrim);                % actual level shift delivered [V]
Voff2 = Vos2*(1 + Rf2/Re);

fprintf('--- Circuit summary ---\n');
fprintf('Stage 1: g_f1 = %.5f (this is K),  g_r1 = %.5f,  g_r1/g_f1 = %.4f\n', g_f1, g_r1, g_r1/g_f1);
fprintf('Stage 2: g_p2 = %.4f on the error signal\n', g_p2);
fprintf('Trimmer setting for hover = %.0f ohm,  level shift = %.4f V (target %.2f V)\n', Rtrim, Vlev_out, Vhover);
fprintf('Effective loop gain K_eff = g_f1*g_p2 = %.5f\n', g_f1*g_p2);

%% 5. Build the Simulink model
mdl = 'lab2_circuit_model';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);  open_system(mdl);
pos = @(x,y,w,h) [x y x+w y+h];

add_block('simulink/Sources/Step',[mdl '/Setpoint_m'], ...
    'Time','0','Before','0','After','r_m','Position',pos(20,100,40,30));
add_block('simulink/Math Operations/Gain',[mdl '/Vref_pot_ks'], ...
    'Gain','ks','Position',pos(90,100,40,30));

% ---- Stage 1 (U1) ----
add_block('simulink/Math Operations/Gain',[mdl '/U1_ref_gain'], ...
    'Gain','g_r1','Position',pos(170,100,40,30));
add_block('simulink/Math Operations/Gain',[mdl '/U1_fb_gain'], ...
    'Gain','g_f1','Position',pos(170,170,40,30));
add_block('simulink/Sources/Constant',[mdl '/U1_offset'], ...
    'Value','Voff1','Position',pos(170,240,40,30));
add_block('simulink/Math Operations/Sum',[mdl '/U1_sum'], ...
    'Inputs','+-+','Position',pos(260,120,30,60));
add_block('simulink/Discontinuities/Saturation',[mdl '/U1_rails'], ...
    'UpperLimit','Vrail','LowerLimit','-Vrail','Position',pos(330,135,40,30));

% ---- Stage 2 (U2) ----
add_block('simulink/Math Operations/Gain',[mdl '/U2_sig_gain'], ...
    'Gain','g_p2','Position',pos(410,135,40,30));
add_block('simulink/Sources/Constant',[mdl '/U2_level_shift'], ...
    'Value','Vlev_out','Position',pos(410,200,40,30));
add_block('simulink/Sources/Constant',[mdl '/U2_offset'], ...
    'Value','Voff2','Position',pos(410,250,40,30));
add_block('simulink/Math Operations/Sum',[mdl '/U2_sum'], ...
    'Inputs','+++','Position',pos(500,150,30,60));
add_block('simulink/Discontinuities/Saturation',[mdl '/U2_rails'], ...
    'UpperLimit','Vrail','LowerLimit','-Vrail','Position',pos(570,165,40,30));

% ---- ADC / cat input limits and hover ----
add_block('simulink/Discontinuities/Saturation',[mdl '/Limit_0to5V'], ...
    'UpperLimit','5','LowerLimit','0','Position',pos(650,165,40,30));
add_block('simulink/Sources/Constant',[mdl '/Gravity_offset'], ...
    'Value','Vhover','Position',pos(650,230,50,30));
add_block('simulink/Math Operations/Sum',[mdl '/NetThrust'], ...
    'Inputs','+-','Position',pos(740,165,30,30));
add_block('simulink/Continuous/Transfer Fcn',[mdl '/Kitticopter'], ...
    'Numerator','[A]','Denominator','[tau 1 0]','Position',pos(810,160,90,40));
add_block('simulink/Math Operations/Gain',[mdl '/Sensor_ks'], ...
    'Gain','ks','Orientation','left','Position',pos(500,330,40,30));

% ---- Logging ----
add_block('simulink/Sinks/To Workspace',[mdl '/Log_position'], ...
    'VariableName','y_pos','SaveFormat','Timeseries','Position',pos(960,165,60,30));
add_block('simulink/Sinks/To Workspace',[mdl '/Log_command'], ...
    'VariableName','u_cmd','SaveFormat','Timeseries','Position',pos(650,90,60,30));
add_block('simulink/Sinks/To Workspace',[mdl '/Log_stage1'], ...
    'VariableName','e1','SaveFormat','Timeseries','Position',pos(330,70,60,30));

% ---- Wires ----
L = @(a,b) add_line(mdl, a, b, 'autorouting','on');
L('Setpoint_m/1','Vref_pot_ks/1');
L('Vref_pot_ks/1','U1_ref_gain/1');
L('U1_ref_gain/1','U1_sum/1');
L('Sensor_ks/1','U1_fb_gain/1');
L('U1_fb_gain/1','U1_sum/2');
L('U1_offset/1','U1_sum/3');
L('U1_sum/1','U1_rails/1');
L('U1_rails/1','Log_stage1/1');
L('U1_rails/1','U2_sig_gain/1');
L('U2_sig_gain/1','U2_sum/1');
L('U2_level_shift/1','U2_sum/2');
L('U2_offset/1','U2_sum/3');
L('U2_sum/1','U2_rails/1');
L('U2_rails/1','Log_command/1');
L('U2_rails/1','Limit_0to5V/1');
L('Limit_0to5V/1','NetThrust/1');
L('Gravity_offset/1','NetThrust/2');
L('NetThrust/1','Kitticopter/1');
L('Kitticopter/1','Log_position/1');
L('Kitticopter/1','Sensor_ks/1');

set_param(mdl,'StopTime',num2str(tStop),'Solver','ode45','MaxStep','0.5');
save_system(mdl);

%% 6. Run and measure
out = sim(mdl);
y  = out.get('y_pos');  t = y.Time;  pos_ = squeeze(y.Data);
u  = out.get('u_cmd');  uc = squeeze(u.Data);
e1 = out.get('e1');     e1d = squeeze(e1.Data);

[pk, ip] = max(pos_);
os   = max(0,(pk - r_m)/r_m*100);
errF = abs(r_m - pos_(end))/r_m*100;
idx  = find(abs(pos_ - r_m) > 0.02*r_m, 1, 'last');
if isempty(idx), ts = 0; elseif idx >= numel(t), ts = NaN; else, ts = t(idx+1); end
fprintf('\n--- 5 m step result (circuit model) ---\n');
fprintf('Peak = %.2f m at %.0f s, overshoot = %.1f %%, final error = %.2f %%, 2%% settling = %.0f s\n', ...
        pk, t(ip), os, errF, ts);
fprintf('Specs: overshoot < 30%%, tracking error < 8%%.  Command range %.3f to %.3f V\n', min(uc), max(uc));

%% 7. Plots: circuit model vs ideal gain model
K_ideal = 0.0122;
Gcl = tf(K_ideal*ks*A, [tau 1 K_ideal*ks*A]);
yi  = r_m * step(Gcl, t);

figure('Name','Circuit model response','Color','w'); hold on; grid on;
plot(t, r_m*(t>=0), 'k--', 'LineWidth', 1.5);
plot(t, yi,   'c-', 'LineWidth', 1.5);
plot(t, pos_, 'b-', 'LineWidth', 2);
plot(t(ip), pk, 'ro', 'MarkerFaceColor','r');
text(t(ip)+8, pk+0.4, sprintf('Peak %.2f m (%.1f%% overshoot)', pk, os), 'Color','r','FontWeight','bold');
if ~isnan(ts), xline(ts,'g-', sprintf('2%% settling %.0f s', ts)); end
yline(1.3*r_m,'r:','30% overshoot limit'); yline(0.92*r_m,'g:','8% error band');
text(0.02,0.97,sprintf('K_{eff} = %.4g (82k/1k stage)', g_f1*g_p2),'Units','normalized', ...
     'VerticalAlignment','top','FontWeight','bold','BackgroundColor','w','EdgeColor','k');
xlabel('Time (s)'); ylabel('Position (m)');
title('Step response: real two-op-amp circuit model vs ideal gain');
legend('Step input','Ideal K = 0.0122','Circuit model','Location','southeast');
xlim([0 tStop]); ylim([0 1.4*r_m]);

figure('Name','Circuit voltages','Color','w'); hold on; grid on;
plot(t, uc, 'm-', 'LineWidth', 2);
yline(Vhover,'k--','Hover 2.5 V');
xlabel('Time (s)'); ylabel('Stage-2 output = angle-of-attack command (V)');
title('Controller output (into the PC ADC)');
xlim([0 tStop]);