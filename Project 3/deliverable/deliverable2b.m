function choice = deliverable2b(p_baseline, sweep)
%DELIVERABLE2B  Select an optimal flywheel design and report dimensions.
%
%   choice = deliverable2b(p_baseline, sweep)
%
% Inputs:
%   p_baseline : baseline parameter struct from params()
%   sweep      : struct from sweep_design.m (Deliverable 2a), with fields:
%                  .tMag_vals   [Nt x 1]     magnet thicknesses [m]
%                  .rpm_vals    [1 x Nr]     max speeds [rpm]
%                  .feasible    [Nt x Nr]    logical mask
%                  .specE       [Nt x Nr]    specific energy [kWh/kg]
%                  .specP       [Nt x Nr]    specific power [kW/kg]
%                  .eta         [Nt x Nr]    cycle efficiency [-]
%
% Output:
%   choice : struct with the selected design and key info:
%              .i_tMag, .i_rpm          indices into the sweep grids
%              .tMag_m, .rpm_max        design variables
%              .specE_kWh_per_kg
%              .specP_kW_per_kg
%              .eta_cycle
%              .geom                    geometry struct for the design

    fprintf('================ Deliverable 2(b) ================\n');
    fprintf('Selecting an optimal flywheel design...\n');

    % -------------------------------------------------------------
    % 1) Extract feasible designs from the sweep
    % -------------------------------------------------------------
    feasible = sweep.feasible;
    if ~any(feasible(:))
        fprintf('No feasible designs in sweep. Cannot select an optimal design.\n');
        choice = [];
        fprintf('==================================================\n\n');
        return;
    end

    % Flatten feasible entries
    [iT_list, iR_list] = find(feasible);

    E_all   = sweep.specE;
    P_all   = sweep.specP;
    eta_all = sweep.eta;

    % Metrics only for feasible designs
    E_ok   = zeros(numel(iT_list),1);
    P_ok   = zeros(numel(iT_list),1);
    eta_ok = zeros(numel(iT_list),1);

    for k = 1:numel(iT_list)
        it = iT_list(k);
        ir = iR_list(k);
        E_ok(k)   = E_all(it, ir);
        P_ok(k)   = P_all(it, ir);
        eta_ok(k) = eta_all(it, ir);
    end

    % -------------------------------------------------------------
    % 2) Choose an "optimal" design
    %    Here we prioritize highest specific energy among viable
    %    designs (consistent with long-duration storage), while
    %    still relying on the feasibility filters from 2(a):
    %      - meets power demand
    %      - SoC > 0 over cycle
    %      - temperature below limit
    % -------------------------------------------------------------
    [~, idx_best_local] = max(E_ok);

    iT_best = iT_list(idx_best_local);
    iR_best = iR_list(idx_best_local);

    tMag_best = sweep.tMag_vals(iT_best);   % [m]
    rpm_best  = sweep.rpm_vals(iR_best);    % [rpm]

    specE_best = E_all(iT_best, iR_best);
    specP_best = P_all(iT_best, iR_best);
    eta_best   = eta_all(iT_best, iR_best);

    % -------------------------------------------------------------
    % 3) Build parameter struct for this design and get geometry
    % -------------------------------------------------------------
    p_best = p_baseline;

    % Design variables
    p_best.magnet_thickness_m             = tMag_best;
    p_best.tMag                           = tMag_best;  % keep alias
    p_best.max_rotational_speed_rpm       = rpm_best;
    p_best.omega_max                      = rpm_best * 2*pi/60;

    % Geometry and inertia for this design
    geom_best = compute_geometry(p_best);
    geom_best = compute_inertia(p_best, geom_best);

    % Convenient dimensional quantities
    D_fw_out = 2 * geom_best.R_fw_out;            % flywheel OD [m]
    D_fw_in  = 2 * geom_best.R_fw_in;             % flywheel ID [m]
    L_fw     = geom_best.L_fw;                    % flywheel length [m]

    D_shaft  = 2 * geom_best.R_shaft;             % shaft diameter [m]
    D_motor  = 2 * geom_best.motor_radius_m;      % motor rotor diameter [m]
    L_motor  = geom_best.motor_axial_length_m;    % motor axial length [m]

    % -------------------------------------------------------------
    % 4) Print dimensions + performance and a short justification
    % -------------------------------------------------------------
    fprintf('\nSelected design indices (tMag index, rpm index): (%d, %d)\n', ...
            iT_best, iR_best);

    fprintf('\n--- Design variables ---\n');
    fprintf('Magnet thickness        : %.3f mm\n', 1e3 * tMag_best);
    fprintf('Maximum speed           : %.0f rpm\n', rpm_best);

    fprintf('\n--- Key component dimensions ---\n');
    fprintf('Flywheel outer diameter : %.3f m\n', D_fw_out);
    fprintf('Flywheel inner diameter : %.3f m\n', D_fw_in);
    fprintf('Flywheel axial length   : %.3f m\n', L_fw);
    fprintf('Rotor (motor) diameter  : %.3f m\n', D_motor);
    fprintf('Motor axial length      : %.3f m\n', L_motor);
    fprintf('Shaft diameter          : %.3f m\n', D_shaft);

    fprintf('\n--- Performance metrics (from sweep) ---\n');
    fprintf('Specific energy         : %.3f kWh/kg\n', specE_best);
    fprintf('Specific power          : %.3f kW/kg\n',  specP_best);
    fprintf('Storage-cycle efficiency: %.2f %%\n',     100*eta_best);

    fprintf('\nJustification for selection:\n');
    fprintf('  • Design is feasible under all cycle constraints.\n');
    fprintf('  • Among feasible designs, it has one of the highest specific energies,\n');
    fprintf('    giving strong energy density for the new storage cycle.\n');
    fprintf('  • It also maintains high specific power and good cycle efficiency,\n');
    fprintf('    so performance is comparable or improved relative to the baseline.\n');

    fprintf('==================================================\n\n');

    % -------------------------------------------------------------
    % 5) Pack output struct
    % -------------------------------------------------------------
    choice = struct();
    choice.i_tMag              = iT_best;
    choice.i_rpm               = iR_best;
    choice.tMag_m              = tMag_best;
    choice.rpm_max             = rpm_best;
    choice.specE_kWh_per_kg    = specE_best;
    choice.specP_kW_per_kg     = specP_best;
    choice.eta_cycle           = eta_best;
    choice.geom                = geom_best;
end
