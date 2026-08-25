function [gain_table, tuning_table] = export_tuning_results(P, history)
%EXPORT_TUNING_RESULTS Save full histories and readable CSV summaries.

project_root = fileparts(fileparts(mfilename('fullpath')));
results_dir = fullfile(project_root,'results');
if ~isfolder(results_dir), mkdir(results_dir); end
save(fullfile(results_dir,'tuning_history.mat'),'history');

loops = {'roll','pitch','course','altitude','airspeed'};
number_rows = 12;
loop_column = strings(number_rows,1); gain_column = strings(number_rows,1);
manual_column = zeros(number_rows,1); autotuned_column = zeros(number_rows,1);
ratio_column = zeros(number_rows,1); row = 0;
for loop_index = 1:numel(loops)
    loop_name = loops{loop_index};
    names = history.(loop_name).gain_names;
    for gain_index = 1:numel(names)
        name = names{gain_index};
        manual_value = P.control.manual.(loop_name).(name);
        auto_value = P.control.(loop_name).(name);
        row = row+1;
        loop_column(row) = string(loop_name);
        gain_column(row) = string(name);
        manual_column(row) = manual_value;
        autotuned_column(row) = auto_value;
        ratio_column(row) = auto_value/manual_value;
    end
end
gain_table = table(loop_column,gain_column,manual_column,autotuned_column,ratio_column, ...
    'VariableNames',{'Loop','Gain','Manual','Autotuned','Ratio'});
writetable(gain_table,fullfile(results_dir,'gain_comparison.csv'));

initial_cost = zeros(numel(loops),1); final_cost = initial_cost; improvement = initial_cost;
for index = 1:numel(loops)
    initial_cost(index) = history.(loops{index}).initial_cost;
    final_cost(index) = history.(loops{index}).final_cost;
    improvement(index) = history.(loops{index}).improvement_percent;
end
tuning_table = table(string(loops(:)),initial_cost,final_cost,improvement, ...
    'VariableNames',{'Loop','InitialCost','FinalCost','ImprovementPercent'});
writetable(tuning_table,fullfile(results_dir,'tuning_summary.csv'));
end
