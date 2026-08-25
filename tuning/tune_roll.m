function [P, history] = tune_roll(P, trim)
%TUNE_ROLL Optimize roll PI-D gains on the nonlinear short maneuver.
[P, history] = optimize_loop('roll',P,trim);
end

