function [P, history] = tune_airspeed(P, trim)
%TUNE_AIRSPEED Optimize airspeed PI gains on a 25-to-28 m/s maneuver.
[P, history] = optimize_loop('airspeed',P,trim);
end

