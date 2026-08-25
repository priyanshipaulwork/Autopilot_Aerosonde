function plot_comparison(comparison, P)
%PLOT_COMPARISON Generate before/after, gain-ratio, and dashboard figures.

project_root = fileparts(fileparts(mfilename('fullpath')));
results_dir = fullfile(project_root,'results');
m = comparison.manual.nominal; a = comparison.autotuned.nominal;

fig = figure('Visible','off','Color','w','Position',[50 50 1200 760]);
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(m.time,m.h_cmd,'--','LineWidth',1.2); hold on
plot(m.time,m.h,'LineWidth',1.3); plot(a.time,a.h,'LineWidth',1.3); grid on
ylabel('Altitude [m]'); title('Altitude Tracking'); legend('Command','Manual','Autotuned','Location','best');
nexttile; plot(m.time,m.Va_cmd,'--','LineWidth',1.2); hold on
plot(m.time,m.Va,'LineWidth',1.3); plot(a.time,a.Va,'LineWidth',1.3); grid on
ylabel('Airspeed [m/s]'); title('Airspeed Tracking'); legend('Command','Manual','Autotuned','Location','best');
nexttile; plot(m.time,rad2deg(unwrap(m.chi_cmd)),'--','LineWidth',1.2); hold on
plot(m.time,rad2deg(unwrap(m.chi)),'LineWidth',1.3); plot(a.time,rad2deg(unwrap(a.chi)),'LineWidth',1.3); grid on
ylabel('Course [deg]'); title('Ground-Course Tracking'); legend('Command','Manual','Autotuned','Location','best');
nexttile; plot(m.time,rad2deg(m.delta_e),'LineWidth',1.3); hold on
plot(a.time,rad2deg(a.delta_e),'LineWidth',1.3); yline(25,'--'); yline(-25,'--'); grid on
ylabel('Elevator [deg]'); title('Elevator Command'); legend('Manual','Autotuned','Limits','Location','best');
nexttile; plot(m.time,rad2deg(m.delta_a),'LineWidth',1.3); hold on
plot(a.time,rad2deg(a.delta_a),'LineWidth',1.3); yline(25,'--'); yline(-25,'--'); grid on
ylabel('Aileron [deg]'); xlabel('Time [s]'); title('Aileron Command'); legend('Manual','Autotuned','Limits','Location','best');
nexttile; plot(m.time,m.delta_t,'LineWidth',1.3); hold on
plot(a.time,a.delta_t,'LineWidth',1.3); yline(0,'--'); yline(1,'--'); grid on
ylabel('Throttle [-]'); xlabel('Time [s]'); title('Throttle Command'); legend('Manual','Autotuned','Limits','Location','best');
set(findall(fig,'-property','FontSize'),'FontSize',9);
exportgraphics(fig,fullfile(results_dir,'manual_vs_autotuned.png'),'Resolution',170); close(fig)

plot_gain_ratios(results_dir,P);
plot_dashboard(results_dir,comparison,P);
end

function plot_gain_ratios(results_dir,P)
G = autotuned_gains();
labels = {'Roll Kp','Roll Ki','Roll Kd','Pitch Kp','Pitch Ki','Pitch Kd', ...
    'Course Kp','Course Ki','Altitude Kp','Altitude Ki','Airspeed Kp','Airspeed Ki'};
loops = {'roll','roll','roll','pitch','pitch','pitch','course','course', ...
    'altitude','altitude','airspeed','airspeed'};
names = {'Kp','Ki','Kd','Kp','Ki','Kd','Kp','Ki','Kp','Ki','Kp','Ki'};
ratios = zeros(size(labels));
for index = 1:numel(labels)
    ratios(index) = G.(loops{index}).(names{index})/P.control.manual.(loops{index}).(names{index});
end
fig = figure('Visible','off','Color','w','Position',[100 100 1100 600]);
bar(ratios); hold on; yline(1,'--','Unchanged'); grid on
xticks(1:numel(labels)); xticklabels(labels); xtickangle(35);
ylabel('Autotuned gain / manual gain [-]'); title('Controller Gain Changes');
set(findall(fig,'-property','FontSize'),'FontSize',10);
exportgraphics(fig,fullfile(results_dir,'gain_comparison.png'),'Resolution',170); close(fig)
end

function plot_dashboard(results_dir,c,P)
if c.accepted, final = c.autotuned; profile = 'Autotuned'; else, final = c.manual; profile = 'Manual'; end
m = c.metrics.manual; a = c.metrics.autotuned;
fig = figure('Visible','off','Color','w','Position',[30 30 1300 780]);
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
nexttile; plot(final.waypoint.pe,final.waypoint.pn,'LineWidth',1.4); hold on
wp=P.sim.waypoints; plot([0;wp(:,2)],[0;wp(:,1)],'--o','LineWidth',1); axis equal; grid on
xlabel('East [m]'); ylabel('North [m]'); title([profile ' Mission']);
nexttile; plot(final.nominal.time,final.nominal.h_cmd,'--','LineWidth',1.1); hold on
plot(final.nominal.time,final.nominal.h,'LineWidth',1.4); grid on; title('Altitude'); ylabel('m'); xlabel('s');
nexttile; plot(final.nominal.time,rad2deg(final.nominal.chi_cmd),'--','LineWidth',1.1); hold on
plot(final.nominal.time,rad2deg(final.nominal.chi),'LineWidth',1.4); grid on; title('Ground Course'); ylabel('deg'); xlabel('s');
nexttile; plot(final.nominal.time,final.nominal.Va_cmd,'--','LineWidth',1.1); hold on
plot(final.nominal.time,final.nominal.Va,'LineWidth',1.4); grid on; title('Airspeed'); ylabel('m/s'); xlabel('s');
nexttile; values=[m.nominal.altitude_rmse_m a.nominal.altitude_rmse_m; ...
    m.nominal.airspeed_rmse_mps a.nominal.airspeed_rmse_mps; ...
    m.nominal.course_rmse_deg a.nominal.course_rmse_deg];
bar(values); grid on; xticklabels({'Altitude [m]','Airspeed [m/s]','Course [deg]'}); xtickangle(15);
title('Nominal RMSE'); legend('Manual','Autotuned','Location','best');
nexttile; effort=[m.nominal.elevator_rms_deg/25 a.nominal.elevator_rms_deg/25; ...
    m.nominal.aileron_rms_deg/25 a.nominal.aileron_rms_deg/25; ...
    m.nominal.elevator_sat_fraction a.nominal.elevator_sat_fraction; ...
    m.nominal.aileron_sat_fraction a.nominal.aileron_sat_fraction];
bar(effort); grid on; xticklabels({'Elev RMS/limit','Ail RMS/limit','Elev sat','Ail sat'}); xtickangle(18);
title('Normalized Actuator Use'); legend('Manual','Autotuned','Location','best');
sgtitle(sprintf('Aerosonde Autopilot Performance Dashboard - %s Recommended',profile));
set(findall(fig,'-property','FontSize'),'FontSize',9);
exportgraphics(fig,fullfile(results_dir,'performance_dashboard.png'),'Resolution',170); close(fig)
end
