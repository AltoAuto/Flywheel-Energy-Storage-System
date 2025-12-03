function sweep = deliverable2a(p_base, t_cycle, P_cycle)
%DELIVERABLE2A  Design trade-off sweep for new storage cycle.
%
%   sweep = deliverable2a(p_base, t_cycle, P_cycle)
%
% Inputs:
%   p_base  : baseline parameter struct from params()
%   t_cycle : time vector for team storage cycle [s]
%   P_cycle : power profile over cycle [W] (+ to grid)
%
% Output:
%   sweep : struct containing sweep grids and metrics:
%             .tMag_vals   [Nt x 1]   magnet thicknesses [m]
%             .rpm_vals    [1 x Nr]   max speeds [rpm]
%             .feasible    [Nt x Nr]  logical
%             .specE       [Nt x Nr]  specific energy [kWh/kg]
%             .specP       [Nt x Nr]  specific power [kW/kg]
%             .eta         [Nt x Nr]  cycle efficiency [-]

    fprintf('========== Deliverable 2(a): Design Trade-offs ==========\n');

    % ---------------------------------------------------------------------
    % 1) Define design-variable ranges
    % ---------------------------------------------------------------------
    % Magnet thickness range [m]
    tMag_vals = linspace(0.002, 0.010, 15);   % 2–10 mm

    % Max speed range [rpm]
    rpm_vals  = linspace(10000, 40000, 15);   % 10k–40k rpm

    Nt = numel(tMag_vals);
    Nr = numel(rpm_vals);

    % ---------------------------------------------------------------------
    % 2) Allocate arrays for metrics
    % ---------------------------------------------------------------------
    feasible = false(Nt, Nr);
    specE    = NaN(Nt, Nr);   % [kWh/kg]
    specP    = NaN(Nt, Nr);   % [kW/kg]
    eta_mat  = NaN(Nt, Nr);   % [-]

    % ---------------------------------------------------------------------
    % 3) Sweep over (tMag, rpm) and evaluate each design
    % ---------------------------------------------------------------------
    for iT = 1:Nt
        for iR = 1:Nr
            tMag = tMag_vals(iT);
            rpm  = rpm_vals(iR);

            metrics = evaluate_design(tMag, rpm, p_base, t_cycle, P_cycle);

            if metrics.feasible
                feasible(iT, iR) = true;
                specE(iT, iR)    = metrics.specific_energy_kWh_per_kg;
                specP(iT, iR)    = metrics.specific_power_kW_per_kg;
                eta_mat(iT, iR)  = metrics.eta_cycle;
            end
        end
    end

    nFeasible = nnz(feasible);
    fprintf('Number of feasible designs: %d\n', nFeasible);
    fprintf('======================================================\n\n');

    % ---------------------------------------------------------------------
    % 4) Trade-off plot: specific power vs specific energy vs efficiency
    % ---------------------------------------------------------------------
    if nFeasible > 0
        E_feas   = specE(feasible);
        P_feas   = specP(feasible);
        eta_feas = 100 * eta_mat(feasible);   % [%]

        figure; hold on; grid on; box on;
        scatter3(E_feas, P_feas, eta_feas, 50, eta_feas, 'filled');
        xlabel('Specific energy [kWh/kg]');
        ylabel('Specific power [kW/kg]');
        zlabel('Storage-cycle efficiency [%]');
        title('Design Trade-offs: Feasible Flywheel Designs');
        cb = colorbar;
        ylabel(cb, 'Storage-cycle efficiency [%]');
        view(135, 25);
    else
        % If nothing feasible, still create an empty figure for completeness
        figure; grid on; box on;
        title('Design Trade-offs: no feasible designs in current sweep');
        xlabel('Specific energy [kWh/kg]');
        ylabel('Specific power [kW/kg]');
        zlabel('Storage-cycle efficiency [%]');
    end

    % ---------------------------------------------------------------------
    % 5) Pack sweep struct for use in Deliverable 2(b)
    % ---------------------------------------------------------------------
    sweep = struct();
    sweep.tMag_vals = tMag_vals;
    sweep.rpm_vals  = rpm_vals;
    sweep.feasible  = feasible;
    sweep.specE     = specE;
    sweep.specP     = specP;
    sweep.eta       = eta_mat;
end
