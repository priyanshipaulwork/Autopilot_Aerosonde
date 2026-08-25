function output_file = animate_flight(data, P, playback_speed)
%ANIMATE_FLIGHT Render an attitude-correct fixed-wing waypoint animation.
%   OUTPUT_FILE = ANIMATE_FLIGHT(DATA,P,PLAYBACK_SPEED) uses actual NED
%   position and 3-2-1 Euler attitude. MPEG-4 is preferred; Motion JPEG AVI
%   is used only when MPEG-4 is unavailable on the current platform.

if nargin < 3, playback_speed = 5; end
project_root = fileparts(fileparts(mfilename('fullpath')));
results_dir = fullfile(project_root,'results');
frame_rate = 20;
requested_file = fullfile(results_dir,'aerosonde_flight.mp4');
try
    writer = VideoWriter(requested_file,'MPEG-4');
    writer.Quality = 92;
    output_file = requested_file;
    writer.FrameRate = frame_rate;
    open(writer);
catch
    output_file = fullfile(results_dir,'aerosonde_flight.avi');
    writer = VideoWriter(output_file,'Motion JPEG AVI');
    writer.FrameRate = frame_rate;
    open(writer);
end
writer_cleanup = onCleanup(@() close(writer));

fig = figure('Visible','on','Color','w','Position',[80 80 1100 700]);
figure_cleanup = onCleanup(@() close(fig));
axes_handle = axes(fig); hold(axes_handle,'on'); grid(axes_handle,'on'); box(axes_handle,'on');
wp = P.sim.waypoints;
plot3(axes_handle,[0;wp(:,2)],[0;wp(:,1)],[P.sim.h0;wp(:,3)],'--','LineWidth',1.1);
plot3(axes_handle,wp(:,2),wp(:,1),wp(:,3),'o','MarkerSize',7,'LineWidth',1.2);
for index = 1:size(wp,1)
    text(axes_handle,wp(index,2)+8,wp(index,1)+8,wp(index,3)+2, ...
        sprintf('WP%d',index),'FontWeight','bold');
end
trace = plot3(axes_handle,data.pe(1),data.pn(1),data.h(1),'LineWidth',1.5);
position_marker = plot3(axes_handle,data.pe(1),data.pn(1),data.h(1),'o','MarkerSize',5,'LineWidth',1.2);
target_marker = plot3(axes_handle,wp(1,2),wp(1,1),wp(1,3),'s','MarkerSize',11,'LineWidth',2);

% The glyph is deliberately enlarged relative to the airframe so attitude
% remains readable on a roughly 500 m mission-scale view.
glyph_scale = 2.2;
body_segments = {[-3.2 4.0;0 0;0 0], ...
                 [0.2 0.2;-5.0 5.0;0 0], ...
                 [-2.5 -2.5;-2.0 2.0;0 0], ...
                 [-2.7 -2.7;0 0;0 -1.7]};
body_segments = cellfun(@(vertices) glyph_scale*vertices,body_segments, ...
    'UniformOutput',false);
aircraft_lines = gobjects(numel(body_segments),1);
for index = 1:numel(body_segments)
    aircraft_lines(index) = plot3(axes_handle,nan,nan,nan,'LineWidth',4, ...
        'Color',[0.84 0.05 0.34],'HandleVisibility','off');
end
telemetry = text(axes_handle,0,0,0,'','FontName','Consolas','FontSize',10, ...
    'BackgroundColor','w','Margin',5,'VerticalAlignment','top');
xlabel(axes_handle,'East [m]'); ylabel(axes_handle,'North [m]'); zlabel(axes_handle,'Altitude [m]');
title(axes_handle,sprintf('Aerosonde Waypoint Flight - %s Controller',capitalize(data.profile)));
x_values = [data.pe(:);wp(:,2)]; y_values = [data.pn(:);wp(:,1)]; z_values = [data.h(:);wp(:,3)];
xlim(axes_handle,[min(x_values)-60 max(x_values)+80]);
ylim(axes_handle,[min(y_values)-60 max(y_values)+80]);
zlim(axes_handle,[min(z_values)-25 max(z_values)+35]);
view(axes_handle,38,24); daspect(axes_handle,[1 1 0.35]);
legend(axes_handle,'Planned path','Waypoints','Completed trajectory','Aircraft position', ...
    'Current target','Location','northeast');

frame_times = data.time(1):playback_speed/frame_rate:data.time(end);
for frame_index = 1:numel(frame_times)
    [~,sample] = min(abs(data.time-frame_times(frame_index)));
    R = body_to_ned(data.phi(sample),data.theta(sample),data.psi(sample));
    position_ned = [data.pn(sample);data.pe(sample);data.pd(sample)];
    for segment_index = 1:numel(body_segments)
        vertices_ned = R*body_segments{segment_index}+position_ned;
        set(aircraft_lines(segment_index),'XData',vertices_ned(2,:), ...
            'YData',vertices_ned(1,:),'ZData',-vertices_ned(3,:));
    end
    set(trace,'XData',data.pe(1:sample),'YData',data.pn(1:sample),'ZData',data.h(1:sample));
    set(position_marker,'XData',data.pe(sample),'YData',data.pn(sample),'ZData',data.h(sample));
    target_index = min(max(1,floor(data.active_waypoint(sample))),size(wp,1));
    set(target_marker,'XData',wp(target_index,2),'YData',wp(target_index,1),'ZData',wp(target_index,3));
    telemetry.Position = [axes_handle.XLim(1)+10 axes_handle.YLim(2)-15 axes_handle.ZLim(2)-3];
    telemetry.String = sprintf(['Time: %5.1f s\nVa: %5.2f m/s\nAltitude: %6.1f m\n' ...
        'Course: %6.1f deg\nWaypoint: %d / %d'],data.time(sample),data.Va(sample), ...
        data.h(sample),rad2deg(data.chi(sample)),target_index,size(wp,1));
    drawnow;
    writeVideo(writer,getframe(fig));
end
clear writer_cleanup
clear figure_cleanup
fprintf('Created flight animation: %s\n',output_file);
end

function R = body_to_ned(phi,theta,psi)
cphi=cos(phi); sphi=sin(phi); cth=cos(theta); sth=sin(theta); cpsi=cos(psi); spsi=sin(psi);
R=[cth*cpsi,sphi*sth*cpsi-cphi*spsi,cphi*sth*cpsi+sphi*spsi; ...
   cth*spsi,sphi*sth*spsi+cphi*cpsi,cphi*sth*spsi-sphi*cpsi; ...
   -sth,sphi*cth,cphi*cth];
end

function text = capitalize(text)
text = char(text); text(1)=upper(text(1));
end
