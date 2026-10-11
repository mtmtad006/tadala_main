%% EEE3094S Lab 2 - Step input for both system identification runs
% Plots the logged Input(v) column for run 1 and run 2:
%   top row    : the whole log
%   bottom row : a zoom around the step, one marker per 100 ms sample, so you can
%                see how quickly the input was switched and where t0 is taken.
%
% Needs R2019b+ (tiledlayout, yline, xline).

clear; clc; close all;

%% 1. Files and settings
folder = 'C:\Users\Student\Documents\MATLAB\tadala_main\EEE3094\Lab_2\';   % <-- change to your folder
files  = {'HelicopterData_MTMTAD006.csv', 'Final_HelicopterTESTData.csv'};
names  = {'Run 1 (first attempt)', 'Run 2 (clean step)'};
Vhover        = 2.5;   % hover voltage [V]
stepThreshold = 3.0;   % t0 = first sample where the input exceeds this [V]
zoomBefore    = 1;     % seconds shown before t0 in the zoom
zoomAfter     = 2;     % seconds shown after t0 in the zoom

%% 2. Plot
figure('Name','Step inputs','Color','w','Position',[80 80 1100 650]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for k = 1:2
    T = readtable([folder files{k}]);
    t = T{:,1};          % time [s]
    u = T{:,2};          % input voltage [V]

    i0 = find(u > stepThreshold, 1, 'first');
    t0 = t(i0);
    fprintf('%s: t0 = %.1f s. Input %.3f V at %.1f s, then %.3f V at %.1f s, %.3f V at %.1f s\n', ...
        names{k}, t0, u(i0-1), t(i0-1), u(i0), t(i0), u(i0+1), t(i0+1));

    % --- full log (top row) ---
    nexttile(k); hold on; grid on;
    plot(t, u, 'b-', 'LineWidth', 1.8);
    yline(Vhover, 'k:', 'Hover 2.5 V', 'LabelHorizontalAlignment', 'left');
    yline(5, 'r:', '5 V', 'LabelHorizontalAlignment', 'left');
    xline(t0, 'g--', sprintf('t_0 = %.1f s', t0), 'LabelOrientation', 'horizontal');
    xlabel('Time (s)'); ylabel('Input voltage (V)');
    title([names{k} ': input over the whole log']);
    ylim([0 6]); xlim([t(1) t(end)]);

    % --- zoom around the step (bottom row) ---
    nexttile(2 + k); hold on; grid on;
    w = t >= t0 - zoomBefore & t <= t0 + zoomAfter;
    plot(t(w), u(w), 'b.-', 'LineWidth', 1.5, 'MarkerSize', 16);
    yline(Vhover, 'k:', 'Hover 2.5 V', 'LabelHorizontalAlignment', 'left');
    yline(5, 'r:', '5 V', 'LabelHorizontalAlignment', 'left');
    xline(t0, 'g--', sprintf('t_0 = %.1f s', t0), 'LabelOrientation', 'horizontal');
    xlabel('Time (s)'); ylabel('Input voltage (V)');
    title([names{k} ': zoom on the step (one marker per 100 ms sample)']);
    ylim([0 6]); xlim([t0 - zoomBefore, t0 + zoomAfter]);
end

title(tl, 'Step inputs applied in the two open-loop tests');