function plot_3d_trajectory(data, P)
%PLOT_3D_TRAJECTORY Plot actual East/North/altitude waypoint mission.

project_root = fileparts(fileparts(mfilename('fullpath')));
fig = figure('Visible','off','Color','w','Position',[100 100 1000 650]);
plot3(data.pe,data.pn,data.h,'LineWidth',1.7); hold on
wp = P.sim.waypoints;
plot3([0;wp(:,2)],[0;wp(:,1)],[P.sim.h0;wp(:,3)],'--','LineWidth',1.1);
plot3(wp(:,2),wp(:,1),wp(:,3),'o','MarkerSize',7,'LineWidth',1.2);
plot3(data.pe(1),data.pn(1),data.h(1),'s','MarkerSize',8,'LineWidth',1.3);
plot3(data.pe(end),data.pn(end),data.h(end),'d','MarkerSize',8,'LineWidth',1.3);
for index = 1:size(wp,1)
    text(wp(index,2)+8,wp(index,1)+8,wp(index,3)+2,sprintf('WP%d',index), ...
        'FontSize',10,'FontWeight','bold');
end
xlabel('East [m]'); ylabel('North [m]'); zlabel('Altitude [m]');
title(sprintf('3D Waypoint Mission - %s Controller',capitalize(data.profile)));
legend('UAV trajectory','Planned path','Waypoints','Start','Final','Location','best');
grid on; box on; view(38,24);
set(findall(fig,'-property','FontSize'),'FontSize',10);
exportgraphics(fig,fullfile(project_root,'results','trajectory_3d.png'),'Resolution',180);
close(fig)
end

function text = capitalize(text)
text = char(text); text(1) = upper(text(1));
end

