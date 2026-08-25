function [chi_cmd, h_cmd, active_index] = waypoint_guidance(pn, pe, waypoints, radius, reset)
%WAYPOINT_GUIDANCE Direct-to-waypoint line-of-sight guidance.
%   [CHI_CMD,H_CMD,INDEX] = WAYPOINT_GUIDANCE(PN,PE,WAYPOINTS,RADIUS,RESET)
%   uses NED horizontal position [m], waypoints [North East Altitude], and
%   a horizontal capture radius [m]. INDEX becomes N+1 after final capture.

persistent waypoint_index
if nargin < 5 || isempty(reset), reset = false; end
if isempty(waypoint_index) || reset
    waypoint_index = 1;
end

number_waypoints = size(waypoints, 1);
target_index = min(waypoint_index, number_waypoints);
target = waypoints(target_index, :);
distance = hypot(target(1)-pn, target(2)-pe);
if distance < radius && waypoint_index <= number_waypoints
    waypoint_index = waypoint_index + 1;
    target_index = min(waypoint_index, number_waypoints);
    target = waypoints(target_index, :);
end

chi_cmd = atan2(target(2)-pe, target(1)-pn);
chi_cmd = atan2(sin(chi_cmd), cos(chi_cmd));
h_cmd = target(3);
active_index = waypoint_index;
end

