function [cases, metrics] = run_profile_cases(P, trim, profile, print_metrics, keep_model_loaded)
%RUN_PROFILE_CASES Run unchanged nominal, wind, and waypoint validation.
%   PROFILE may be manual, autotuned, or active for an in-memory candidate.
%   KEEP_MODEL_LOADED is used by autotune_all only across the transition
%   from its fresh baseline to the first Fast Restart optimization.

if nargin < 4, print_metrics = false; end
if nargin < 5, keep_model_loaded = false; end
if ~strcmpi(profile,'active')
    P = apply_gain_profile(P,profile);
end
cases = struct(); metrics = struct();
[cases.nominal,metrics.nominal] = simulate_case('nominal','active',print_metrics,P,trim,false);
[cases.wind,metrics.wind] = simulate_case('wind','active',print_metrics,P,trim,false);
[cases.waypoint,metrics.waypoint] = simulate_case( ...
    'waypoint','active',print_metrics,P,trim,~keep_model_loaded);
end
