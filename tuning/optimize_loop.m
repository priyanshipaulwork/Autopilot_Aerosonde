function [P, record] = optimize_loop(loop_name, P, trim)
%OPTIMIZE_LOOP Bounded fminsearch optimization for one controller loop.
%   This shared engine records every nonlinear simulation evaluation while
%   controller-specific public wrappers preserve the tuning sequence API.

loop_name = char(lower(string(loop_name)));
if ismember(loop_name,{'roll','pitch'})
    gain_names = {'Kp','Ki','Kd'};
else
    gain_names = {'Kp','Ki'};
end
manual = P.control.manual.(loop_name);
initial = P.control.(loop_name);
number_gains = numel(gain_names);
lower_bounds = zeros(1,number_gains); upper_bounds = zeros(1,number_gains);
initial_values = zeros(1,number_gains);
for index = 1:number_gains
    name = gain_names{index};
    lower_bounds(index) = manual.(name)*P.tuning.lower_multiplier.(name);
    upper_bounds(index) = manual.(name)*P.tuning.upper_multiplier.(name);
    initial_values(index) = initial.(name);
end
z0 = inverse_logistic(initial_values,lower_bounds,upper_bounds);

% Configure the maneuver before enabling Fast Restart. Each loop compiles
% once, then candidate gains are read dynamically by interpreted blocks.
model_name = 'UAV_Autopilot';
durations = struct('roll',25,'pitch',30,'course',35,'altitude',40,'airspeed',30);
set_param(model_name,'FixedStep',num2str(P.tuning.dt,'%.8g'), ...
    'StopTime',num2str(durations.(loop_name)),'FastRestart','on');
fast_restart_cleanup = onCleanup(@() set_param(model_name,'FastRestart','off'));

evaluation_number = 0; current_iteration = 0; best_cost = inf;
iterations = []; evaluations = []; candidates = []; costs = []; best_costs = [];
initial_cost = objective(z0);
options = optimset('Display','iter', ...
    'MaxIter',P.tuning.MaxIterations, ...
    'MaxFunEvals',P.tuning.MaxFunctionEvaluations, ...
    'TolX',P.tuning.TolX,'TolFun',P.tuning.TolFun, ...
    'OutputFcn',@output_function);
[z_final,final_cost,exitflag,output] = fminsearch(@objective,z0,options);
optimized_values = logistic(z_final,lower_bounds,upper_bounds);
for index = 1:number_gains
    P.control.(loop_name).(gain_names{index}) = optimized_values(index);
end

record.loop = loop_name;
record.gain_names = gain_names;
record.manual_gains = cellfun(@(name) manual.(name),gain_names);
record.initial_gains = initial_values;
record.optimized_gains = optimized_values;
record.lower_bounds = lower_bounds;
record.upper_bounds = upper_bounds;
record.iteration = iterations(:);
record.evaluation = evaluations(:);
record.candidate_gains = candidates;
record.cost = costs(:);
record.best_cost = best_costs(:);
record.initial_cost = initial_cost;
record.final_cost = final_cost;
record.improvement_percent = 100*(initial_cost-final_cost)/max(initial_cost,eps);
record.exitflag = exitflag;
record.output = output;
clear fast_restart_cleanup

    function J = objective(z)
        evaluation_number = evaluation_number+1;
        gain_values = logistic(z,lower_bounds,upper_bounds);
        candidate_P = P;
        for gain_index = 1:number_gains
            candidate_P.control.(loop_name).(gain_names{gain_index}) = gain_values(gain_index);
        end
        try
            tuning_data = simulate_tuning_case(loop_name,candidate_P,trim);
            J = tuning_cost(loop_name,tuning_data,candidate_P);
            if ~isfinite(J), J = 1e6; end
        catch caught_error
            J = 1e6 + min(1e5,numel(caught_error.message));
        end
        best_cost = min(best_cost,J);
        iterations(end+1,1) = current_iteration;
        evaluations(end+1,1) = evaluation_number;
        candidates(end+1,:) = gain_values;
        costs(end+1,1) = J;
        best_costs(end+1,1) = best_cost;
    end

    function stop = output_function(~,optim_values,~)
        current_iteration = optim_values.iteration;
        stop = false;
    end
end

function gains = logistic(z,lower,upper)
z = min(max(z,-60),60);
gains = lower+(upper-lower)./(1+exp(-z));
end

function z = inverse_logistic(gains,lower,upper)
ratio = (gains-lower)./(upper-gains);
ratio = max(ratio,1e-9);
z = log(ratio);
end
