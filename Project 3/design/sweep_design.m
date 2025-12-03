function sweep = sweep_design(p_base, t_cycle, P_cycle)
%SWEEP_DESIGN  Sweep magnet thickness & max speed, evaluate each design.
%
%   sweep = sweep_design(p_base, t_cycle, P_cycle)
%
% Inputs:
%   p_base   : baseline parameter struct from params()
%   t_cycle  : time vector for team storage cycle [s]
%   P_cycle  : power command over cycle [W] (+ to grid)
%
% Output struct 'sweep' with fields:
%   Grids:
%     .tMag_vals_m        [Nt x 1]   magnet thickness values [m]
%     .rpm_vals           [1 x Nr]   max speed values [rpm]
%
%   Raw matrices over (tMag, rpm) grid:
%     .feasible           [Nt x Nr]  logical
%     .specE_Whkg         [Nt x Nr]  specific energy [Wh/kg] (NaN if infeasible)
%     .specP_kWkg         [Nt x Nr]  specific power [kW/kg] (NaN if infeasible)
%     .eta_cycle          [Nt x Nr]  round-trip efficiency [-] (NaN if infeasible)
%     .Tmax_C             [Nt x Nr]  rotor temperature [°C] at rated (NaN if infeasible)
%     .SoC_min            [Nt x Nr]  minimum SoC over cycle [-] (NaN if infeasible)
%     .reason             [Nt x Nr]  string reason when infeasible
%
%   Best design (by specific energy among feasible ones):
%     .best              struct with fields (empty if no feasible design):
%         .i_tMag, .i_rpm
%         .tMag_best_m
%         .rpm_best
%         .specE_best_Whkg
%         .specP_best_kWkg
%         .eta_best
%         .Tmax_best_C
%         .SoC_min_best
%
% This is meant to feed:
%   - Deliverable 2(a): design trade-off plot (specific power vs energy vs efficiency)
%   - Deliverable 2(b): selection of an "optimal" design from the feasible set.

    fprintf('========== Deliverable 2a: Design Trade-offs ==========\n');
    fprintf('Sweeping design space (tMag vs rpm_max)...\n');

    % ------------------------------------------------------------------
    % 1) Define design-variable grids
    % ------------------------------------------------------------------
    % Magnet thickness [m]
    tMag_vals_m = linspace(0.002, 0.010, 15);   % 2–10 mm

    % Max speed [rpm]
    rpm_vals    = linspace(10000, 40000, 15);   % 10k–40k rpm

    Nt = numel(tMag_vals_m);
    Nr = numel(rpm_vals);

    % ------------------------------------------------------------------
    % 2) Preallocate arrays over the design grid
    % ------------------------------------------------------------------
    feasible   = false(Nt, Nr);
    specE_Whkg = nan(Nt, Nr);
    specP_kWkg = nan(Nt, Nr);
    eta_cycle  = nan(Nt, Nr);
    Tmax_C     = nan(Nt, Nr);
    SoC_min    = nan(Nt, Nr);
    reason     = strings(Nt, Nr);

    % ------------------------------------------------------------------
    % 3) Sweep all (tMag, rpm) combinations
    % ------------------------------------------------------------------
    for it = 1:Nt
        for ir = 1:Nr

            tMag = tMag_vals_m(it);
            rpm  = rpm_vals(ir);

            % Evaluate this design using YOUR single-design function
            metrics = evaluate_design(tMag, rpm, p_base, t_cycle, P_cycle);

            if metrics.feasible
                feasible(it, ir) = true;

                % convert kWh/kg → Wh/kg
                specE_Whkg(it, ir) = metrics.specific_energy_kWh_per_kg * 1e3;
                specP_kWkg(it, ir) = metrics.specific_power_kW_per_kg;
                eta_cycle(it, ir)  = metrics.eta_cycle;
                Tmax_C(it, ir)     = metrics.T_rotor_rated_K - 273.15;
                SoC_min(it, ir)    = metrics.SoC_min;

                fprintf('  Feasible:   tMag = %.3f m, rpm = %5.0f\n', ...
                        tMag, rpm);
            else
                feasible(it, ir) = false;
                reason(it, ir)   = metrics.reason_if_infeasible;

                fprintf('  Infeasible: tMag = %.3f m, rpm = %5.0f → %s\n', ...
                        tMag, rpm, metrics.reason_if_infeasible);
            end

        end
    end

    % ------------------------------------------------------------------
    % 4) Pack outputs
    % ------------------------------------------------------------------
    sweep.tMag_vals_m = tMag_vals_m(:);
    sweep.rpm_vals    = rpm_vals(:).';

    sweep.feasible    = feasible;
    sweep.specE_Whkg  = specE_Whkg;
    sweep.specP_kWkg  = specP_kWkg;
    sweep.eta_cycle   = eta_cycle;
    sweep.Tmax_C      = Tmax_C;
    sweep.SoC_min     = SoC_min;
    sweep.reason      = reason;

    n_feas = nnz(feasible);
    fprintf('Number of feasible designs: %d\n', n_feas);
    fprintf('======================================================\n\n');

    % ------------------------------------------------------------------
    % 5) Choose best design for Deliverable 2(b)
    %     (here: max specific energy among feasible designs)
    % ------------------------------------------------------------------
    if n_feas == 0
        sweep.best = [];
        fprintf('No feasible designs → cannot select a proposed design.\n\n');
        return;
    end

    specE_flat    = specE_Whkg(:);
    feasible_flat = feasible(:);

    % Ignore infeasible designs in the max search
    specE_flat(~feasible_flat) = -inf;

    [~, idx_best_flat] = max(specE_flat);
    [i_tMag_best, i_rpm_best] = ind2sub(size(specE_Whkg), idx_best_flat);

    sweep.best.i_tMag        = i_tMag_best;
    sweep.best.i_rpm         = i_rpm_best;
    sweep.best.tMag_best_m   = tMag_vals_m(i_tMag_best);
    sweep.best.rpm_best      = rpm_vals(i_rpm_best);
    sweep.best.specE_best_Whkg   = specE_Whkg(i_tMag_best, i_rpm_best);
    sweep.best.specP_best_kWkg   = specP_kWkg(i_tMag_best, i_rpm_best);
    sweep.best.eta_best          = eta_cycle(i_tMag_best, i_rpm_best);
    sweep.best.Tmax_best_C       = Tmax_C(i_tMag_best, i_rpm_best);
    sweep.best.SoC_min_best      = SoC_min(i_tMag_best, i_rpm_best);

    fprintf('Best design (by specific energy):\n');
    fprintf('  tMag = %.3f mm, rpm = %.0f\n', ...
            1e3 * sweep.best.tMag_best_m, sweep.best.rpm_best);
    fprintf('  E_spec = %.1f Wh/kg, P_spec = %.3f kW/kg, eta = %.2f %%\n\n', ...
            sweep.best.specE_best_Whkg, ...
            sweep.best.specP_best_kWkg, ...
            100 * sweep.best.eta_best);
end
