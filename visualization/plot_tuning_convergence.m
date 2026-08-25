function plot_tuning_convergence(history)
%PLOT_TUNING_CONVERGENCE Plot normalized best objective for all five loops.

project_root = fileparts(fileparts(mfilename('fullpath')));
fig = figure('Visible','off','Color','w','Position',[100 100 1000 620]); hold on
loops = {'roll','pitch','course','altitude','airspeed'};
for index = 1:numel(loops)
    item = history.(loops{index});
    normalized = item.best_cost/max(item.initial_cost,eps);
    plot(item.evaluation,normalized,'LineWidth',1.5,'DisplayName',capitalize(loops{index}));
end
yline(1,'--','Manual initial objective','LabelVerticalAlignment','bottom', ...
    'HandleVisibility','off');
xlabel('Nonlinear simulation evaluation'); ylabel('Best objective, J/J_{initial} [-]');
title('Sequential Simulation-Based PID Gain Optimization');
legend('Location','best'); grid on; ylim([0 max(1.05,ylim_value(gca))]);
set(findall(fig,'-property','FontSize'),'FontSize',10);
exportgraphics(fig,fullfile(project_root,'results','autotuning_convergence.png'),'Resolution',170);
close(fig)
end

function value = ylim_value(axis_handle)
limits = axis_handle.YLim; value = limits(2);
end

function text = capitalize(text)
text(1) = upper(text(1));
end
