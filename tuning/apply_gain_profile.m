function P = apply_gain_profile(P, profile)
%APPLY_GAIN_PROFILE Copy a named gain set into active controller fields.
%   P = APPLY_GAIN_PROFILE(P,'manual') restores the immutable engineering
%   baseline. 'autotuned' loads the human-readable generated candidate.

profile = lower(string(profile));
switch profile
    case "manual"
        gains = P.control.manual;
    case "autotuned"
        if exist('autotuned_gains','file') ~= 2
            error(['Auto-tuned gains do not exist. Run ' ...
                'run(''tuning/autotune_all.m'') from the project root.']);
        end
        gains = autotuned_gains();
        if isfield(gains,'generated') && ~gains.generated
            error(['Auto-tuned gains have not been generated yet. Run ' ...
                'run(''tuning/autotune_all.m'').']);
        end
    otherwise
        error('Unknown gain profile "%s". Use manual or autotuned.',profile);
end

loops = {'roll','pitch','course','altitude','airspeed'};
for index = 1:numel(loops)
    loop_name = loops{index};
    P.control.(loop_name) = gains.(loop_name);
end
P.control.profile = char(profile);
end

