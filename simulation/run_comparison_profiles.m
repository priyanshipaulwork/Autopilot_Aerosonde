function [comparison, accepted] = run_comparison_profiles(P, trim, print_metrics)
%RUN_COMPARISON_PROFILES Execute all six manual/autotuned validation runs.

if nargin < 3, print_metrics = false; end
[comparison.manual,comparison.metrics.manual] = ...
    run_profile_cases(P,trim,'manual',print_metrics);
[comparison.autotuned,comparison.metrics.autotuned] = ...
    run_profile_cases(P,trim,'autotuned',print_metrics);
[accepted,comparison.summary,comparison.metric_table] = ...
    evaluate_autotuning(comparison,P,true);
comparison.accepted = accepted;
results_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))),'results');
save(fullfile(results_dir,'comparison_results.mat'),'comparison','trim');
end

