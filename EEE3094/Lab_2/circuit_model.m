%% 6. Run the circuit model and resample onto a uniform time grid
out  = sim(mdl);
dt   = 0.05;  tu = (0:dt:tStop)';
pos_ = resampleTS(out.get('y_pos'), tu);     % position [m]
uc   = resampleTS(out.get('u_cmd'), tu);     % controller output [V]

[pk, ip] = max(pos_);
os    = max(0, (pk - r_m)/r_m*100);                  % overshoot [%]
yss   = mean(pos_(tu >= tStop-10));                  % final value
errF  = abs(r_m - yss)/r_m*100;                      % tracking error [%]
idx   = find(abs(pos_ - r_m) > 0.02*r_m, 1, 'last');
if isempty(idx), ts = 0; elseif idx >= numel(tu), ts = NaN; else, ts = tu(idx+1); end

strs   = ["FAIL","PASS"];  pf = @(b) strs(b+1);
passOS = os   < 30;
passTR = errF < 8;
passU  = min(uc) > 0 && max(uc) < 5;                 % no ADC saturation

fprintf('\n--- 5 m step, circuit model ---\n');
fprintf('Peak %.3f m at %.1f s | overshoot %.1f %% | final error %.2f %% | 2%% settling %.1f s\n', ...
        pk, tu(ip), os, errF, ts);
fprintf('Overshoot <30%%: %s | Tracking <8%%: %s | u stays inside 0-5 V: %s (%.3f to %.3f V)\n', ...
        pf(passOS), pf(passTR), pf(passU), min(uc), max(uc));

% Ideal-gain overlay
K0  = g_f1*g_p2;                                     % effective K of the circuit
yi  = r_m*step(tf(K0*ks*A, [tau 1 K0*ks*A]), tu);

%% 7. Figure 1: step response with specification markers
fig1 = figure('Name','Circuit step response','Color','w','Position',[100 100 900 520]);
hold on; grid on;
hb = patch([0 tStop tStop 0], [0.92 0.92 1.08 1.08]*r_m, [0.82 0.94 0.82], ...
           'EdgeColor','none','FaceAlpha',0.7);
hr = plot(tu, r_m*ones(size(tu)), 'k--', 'LineWidth', 1.5);
hi = plot(tu, yi,   'c-', 'LineWidth', 1.5);
hc = plot(tu, pos_, 'b-', 'LineWidth', 2);
hl = yline(1.3*r_m, 'r:', '30% overshoot limit', 'LineWidth', 1.5, ...
           'LabelHorizontalAlignment','left');
hp = plot(tu(ip), pk, 'ro', 'MarkerFaceColor','r', 'MarkerSize', 7);
if ~isnan(ts), xline(ts, 'g--', sprintf('2%% settling %.0f s', ts), 'LineWidth', 1.2); end
text(tu(ip)+8, pk+0.3, sprintf('Peak %.2f m at %.1f s', pk, tu(ip)), ...
     'Color','r','FontWeight','bold');
txt = {sprintf('Overshoot = %.1f %%  (< 30 %%)  %s', os,   pf(passOS)), ...
       sprintf('Final error = %.2f %%  (< 8 %%)  %s', errF, pf(passTR)), ...
       sprintf('K = %.4f,  K_{eff} k_s A = %.3f', K0, K0*ks*A)};
text(0.98, 0.30, txt, 'Units','normalized', 'HorizontalAlignment','right', ...
     'VerticalAlignment','top', 'FontWeight','bold', 'BackgroundColor','w', 'EdgeColor','k');
xlabel('Time (s)'); ylabel('Position (m)');
title('5 m step response of the controller circuit');
legend([hr hb hc hi hl hp], {'Set-point 5 m','\pm8% tracking band','Circuit model', ...
       'Ideal K = 0.0122','30% overshoot limit','Peak'}, 'Location','southeast');
xlim([0 tStop]); ylim([0 1.45*r_m]);
print(fig1, 'fig_step_nominal', '-dpng', '-r300');

%% 8. Figure 2: robustness cases (ideal closed-loop model)
%        name                         b-scale  K-scale
cases = { 'Nominal',                   1,       1;
          'b -10%',                    0.9,     1;
          'b +10%',                    1.1,     1;
          'K +22% (resistors)',        1,       1.22;
          'K -18% (resistors)',        1,       0.82;
          'b -10% and K +22% (both)',  0.9,     1.22 };
fig2 = figure('Name','Robustness','Color','w','Position',[100 100 900 520]);
hold on; grid on;
patch([0 tStop tStop 0], [0.92 0.92 1.08 1.08]*r_m, [0.82 0.94 0.82], ...
      'EdgeColor','none','FaceAlpha',0.7);
cmap = lines(size(cases,1));  lg = cell(1, size(cases,1));  hh = gobjects(1, size(cases,1));
fprintf('\n%-28s %8s %8s %8s  %s\n','Case','Peak(m)','OS(%)','Err(%)','Overshoot<30%');
for k = 1:size(cases,1)
    bs = cases{k,2};  Ks = cases{k,3};
    tau_c = tau/bs;  A_c = A/bs;                 % tau = 1/b and A = k_theta*k_f/b
    Kl = K0*Ks*ks*A_c;                           % loop gain
    y  = r_m*step(tf(Kl, [tau_c 1 Kl]), tu);
    osk = max(0, (max(y)-r_m)/r_m*100);  erk = abs(r_m - y(end))/r_m*100;
    fprintf('%-28s %8.2f %8.1f %8.2f  %s\n', cases{k,1}, max(y), osk, erk, pf(osk < 30));
    hh(k) = plot(tu, y, 'Color', cmap(k,:), 'LineWidth', 1.8);
    lg{k} = sprintf('%s: OS %.1f%%', cases{k,1}, osk);
end
plot(tu, r_m*ones(size(tu)), 'k--', 'LineWidth', 1.2);
yline(1.3*r_m, 'r:', '30% overshoot limit', 'LineWidth', 1.5, 'LabelHorizontalAlignment','left');
xlabel('Time (s)'); ylabel('Position (m)');
title('5 m step response under parameter uncertainty');
legend(hh, lg, 'Location','southeast');
xlim([0 tStop]); ylim([0 1.45*r_m]);
print(fig2, 'fig_robustness', '-dpng', '-r300');

%% 9. Figure 3: controller output stays inside the ADC range
fig3 = figure('Name','Controller output','Color','w','Position',[100 100 900 420]);
hold on; grid on;
plot(tu, uc, 'm-', 'LineWidth', 2);
yline(Vhover, 'k--', 'Hover voltage', 'LabelHorizontalAlignment','left');
yline(0, 'r:', '0 V limit'); yline(5, 'r:', '5 V limit');
xlabel('Time (s)'); ylabel('Controller output u (V)');
title('Controller output u(t) for the 5 m step');
xlim([0 tStop]); ylim([-0.5 5.5]);
print(fig3, 'fig_controller_output', '-dpng', '-r300');

%% Local function (must stay at the end of the script)
function v = resampleTS(ts, tu)
    [tt, i] = unique(ts.Time);
    d = squeeze(ts.Data);
    v = interp1(tt, d(i), tu, 'linear', 'extrap');
end