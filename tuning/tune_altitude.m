function [P, history] = tune_altitude(P, trim)
%TUNE_ALTITUDE Optimize altitude PI gains after fixing the pitch loop.
[P, history] = optimize_loop('altitude',P,trim);
end

