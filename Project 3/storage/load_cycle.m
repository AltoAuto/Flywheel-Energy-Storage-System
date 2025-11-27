function [t, P_cmd, info] = load_cycle(cycle_type)
% LOAD_CYCLE  Loads baseline, generic, or team-specific storage cycle.
%
%   [t, P_cmd, info] = load_cycle()
%   [t, P_cmd, info] = load_cycle('baseline')
%   [t, P_cmd, info] = load_cycle('team')

    if nargin < 1 || isempty(cycle_type)
        cycle_type = 'baseline';
    end

    cycle_type = lower(cycle_type);

    switch cycle_type

        case 'baseline'
            % 15-minute baseline frequency regulation cycle
            T_end = 900;      % [s]
            dt    = 1.0;      % [s]
            t     = (0:dt:T_end)';  % column

            P_cmd    = baselineStorageCycle(t);
            info.type = 'baseline';
            info.name = 'Baseline XS cycle';

        case 'team'
            T_end = 21600;    % [s]
            dt    = 1.0;
            t     = (0:dt:T_end)';

            P_cmd    = team_6_cycle(t);
            info.type = 'team';
            info.name = 'Team 6 cycle';

        otherwise
            error('Unknown cycle type: %s', cycle_type);
    end
end

