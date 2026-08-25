function [P, history] = tune_course(P, trim)
%TUNE_COURSE Optimize course PI gains with optimized attitude loops fixed.
[P, history] = optimize_loop('course',P,trim);
end

