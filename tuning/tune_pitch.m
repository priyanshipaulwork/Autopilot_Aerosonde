function [P, history] = tune_pitch(P, trim)
%TUNE_PITCH Optimize pitch PI-D gains after fixing optimized roll gains.
[P, history] = optimize_loop('pitch',P,trim);
end

