function metrics = evaluate_design(tMag_m, rpm_max, p_base, t_cycle, P_cycle)
%EVALUATE_DESIGN  Size a flywheel for a given cycle and check constraints.
%
%   metrics = evaluate_design(tMag_m, rpm_max, p_base, t_cycle, P_cycle)
%
% Inputs:
%   tMag_m   : magnet thickness [m] (design variable)
%   rpm_max  : maximum speed [rpm] (design variable)
%   p_base   : baseline params struct from params()
%   t_cycle  : time vector for team storage cycle [s]
%   P_cycle  : power command over cycle [W] (+ to grid)
%
% Output: struct 'metrics' with fields (main ones):
%   .feasible                   true/false
%   .reason_if_infeasible       string
%   .tMag_m                     [m]
%   .rpm_max                    [rpm]
%   .omega_max                  [rad/s]
%   .P_peak_req_W               [W]
%   .m_rotor_kg                 [kg]
%   .E_cap_J                    [J] usable energy (0.5→1.0 SoC)
%   .specific_energy_kWh_per_kg [kWh/kg]
%   .P_rated_W                  [W]
%   .specific_power_kW_per_kg   [kW/kg]
%   .eta_cycle                  [-]
%   .T_rotor_rated_K            [K]
%   .P_loss_rated_W             [W]
%   .SoC_min                    [-]

    % ---------------------------------------------------------------------
    % 0) Set up baseline + design variables
    % ---------------------------------------------------------------------
    p = p_base;

    % Design variables: magnet thickness & max speed
    p.magnet_thickness_m             = tMag_m;
    p.tMag                           = tMag_m;       % keep alias consistent

    p.max_rotational_speed_rpm       = rpm_max;
    p.max_rotational_speed_rad_per_s = rpm_max * 2*pi/60;
    p.omega_max                      = p.max_rotational_speed_rad_per_s;
    p.rpm_max                        = rpm_max;

    % SoC definition: 0% at 0.5*omega_max, 100% at omega_max
    omega_max = p.omega_max;
    omega_min = 0.5 * omega_max;

    % Make sure we start storage cycles at 50% SoC
    p.initial_state_of_charge_frac   = 0.50;

    % Peak power demand from the given cycle
    P_peak_req = max(P_cycle);


    % Initialize metrics
    metrics = struct();
    metrics.feasible             = false;
    metrics.reason_if_infeasible = "";
    metrics.tMag_m               = tMag_m;
    metrics.rpm_max              = rpm_max;
    metrics.omega_max            = omega_max;
    metrics.P_peak_req_W         = P_peak_req;

    % ---------------------------------------------------------------------
    % 1) Basic checks: magnet thickness, etc.
    % ---------------------------------------------------------------------

    % Magnet thickness minimum limit
    if tMag_m < p.magnet_thickness_min_m
        metrics.reason_if_infeasible = 'Magnet thickness below minimum limit';
        return;
    end

    % ---------------------------------------------------------------------
    % 2) Required usable kinetic energy from the new cycle
    % ---------------------------------------------------------------------
    % Net energy swing of the cycle (ideal, no losses)
    E_cum  = cumtrapz(t_cycle, P_cycle);            % [J]
    E_span = max(E_cum) - min(E_cum);               % [J]

    % Add a safety factor so SoC stays away from 0
    safety_factor = 1.2;                            
    DeltaE_req    = safety_factor * E_span;         % [J] required usable KE

    % If new cycle is trivial (no net swing), reject (nothing to design)
    if DeltaE_req <= 0
        metrics.reason_if_infeasible = 'Cycle energy swing is zero or negative';
        return;
    end

    % Required total inertia (rotating group) to supply DeltaE_req between
    % omega_min and omega_max:
    DeltaOmega2 = omega_max^2 - omega_min^2;
    if DeltaOmega2 <= 0
        metrics.reason_if_infeasible = 'omega_max must be larger than omega_min';
        return;
    end

    J_req = 2 * DeltaE_req / DeltaOmega2;           % [kg·m^2]

    % ---------------------------------------------------------------------
    % 3) Design flywheel outer radius to hit required inertia
    % ---------------------------------------------------------------------
    % We use your own geometry model as the backbone. We:
    %   1) Use baseline geometry to get inner radius and lengths
    %   2) Compute shaft+magnet inertia
    %   3) Size the flywheel annulus so that:
    %          J_total ≈ J_req  (flywheel + shaft + magnets)

    % First build geometry with some outer diameter; we only need inner radius,
    % shaft radius, and motor length. p.D_out is baseline flywheel diameter.
    geom0 = compute_geometry(p);

    % Known quantities that don't change when we resize the flywheel radius:
    R_i_fw  = geom0.R_fw_in;               % flywheel inner radius [m]
    L_fw    = geom0.L_fw;                  % flywheel axial length [m]
    R_sh    = geom0.R_shaft;               % shaft radius [m]
    L_motor = geom0.motor_axial_length_m;  % motor axial length [m]

    % Shaft inertia (solid cylinder)
    axial_clearance = p.axial_clearance_m; % from params, Table 1
    L_shaft_total   = L_fw + 2*L_motor + 4*axial_clearance;
    V_shaft         = pi * R_sh^2 * L_shaft_total;
    m_shaft         = p.rho_steel * V_shaft;
    J_shaft         = 0.5 * m_shaft * R_sh^2;

    % Magnet inertia (cylindrical ring: inner = R_sh, outer = R_sh + tMag)
    R_i_mag = R_sh;
    R_o_mag = R_sh + tMag_m;
    L_mag   = L_motor;
    V_mag   = pi * (R_o_mag^2 - R_i_mag^2) * L_mag;
    m_mag   = p.rho_mag * V_mag;
    J_mag   = 0.5 * m_mag * (R_o_mag^2 + R_i_mag^2);

    % Inertia needed from composite flywheel alone
    J_fixed  = J_shaft + J_mag;               % non-flywheel inertia
    J_f_req  = max(J_req - J_fixed, 0);       % [kg·m^2] needed from flywheel

    % Solve for flywheel outer radius R_o_fw such that:
    %   J_f = 0.5 * rho_comp * pi * L_fw * (R_o^4 - R_i^4) = J_f_req
    %  →  R_o^4 = (2*J_f_req) / (rho_comp*pi*L_fw) + R_i^4
    if J_f_req == 0
        R_o_fw = R_i_fw;
    else
        R_o4_fw = (2 * J_f_req) / (p.rho_comp * pi * L_fw) + R_i_fw^4;
        if R_o4_fw <= 0
            metrics.reason_if_infeasible = 'Computed flywheel outer radius is non-physical';
            return;
        end
        R_o_fw = R_o4_fw^(1/4);
    end

    % Update flywheel diameter in params and rebuild full geometry
    p.D_out = 2 * R_o_fw;
    geom    = compute_geometry(p);
    geom    = compute_inertia(p, geom);   % now has m_rotor_total_kg and J_total

    % ---------------------------------------------------------------------
    % 4) Rated torque, power, losses, rotor temperature
    % ---------------------------------------------------------------------
    T_rated = calcRatedTorque(p, geom);           % [N·m] (I_pu = 1.0)
    P_rated = T_rated * omega_max;               % [W]

    % Check rated power vs peak cycle requirement
    if P_rated < P_peak_req
        metrics.reason_if_infeasible = 'Rated power below peak cycle demand';
        return;
    end

    % Losses at rated condition (I_pu = 1, omega = omega_max)
    I_pu_rated = 1.0;
    [P_total_rated, P_rot_rated, ~, ~, ~] = ...
        compute_losses(p, geom, I_pu_rated, omega_max);

    % Radiative rotor temperature at rated point (same model as Part 1)
    T_rotor_rated_K = compute_temperature(p, geom, P_rot_rated);

    if T_rotor_rated_K > p.T_max
        metrics.reason_if_infeasible = 'Rotor temperature exceeds limit at rated power';
        return;
    end

    % ---------------------------------------------------------------------
    % 5) Energy capacity & specific metrics (consistent with Part 1)
    % ---------------------------------------------------------------------
    m_tot = geom.m_rotor_total_kg;
    J_tot = geom.J_total;

    % Usable energy between 50% SoC (omega_min) and 100% SoC (omega_max)
    E_cap = 0.5 * J_tot * (omega_max^2 - omega_min^2);   % [J]

    spec_energy_kWh_per_kg = E_cap / (3.6e6 * m_tot);    % [kWh/kg]
    spec_power_kW_per_kg   = (P_rated/1e3) / m_tot;      % [kW/kg]

    % ---------------------------------------------------------------------
    % 6) Simulate the new storage cycle with this design
    % ---------------------------------------------------------------------
    result = simulate_cycle(p, geom, t_cycle, P_cycle);
    % Expected fields: result.t, result.omega, result.soc, result.P_cmd, result.P_total

    soc_min = min(result.soc);
    if soc_min <= 0
        metrics.reason_if_infeasible = 'SoC reached 0 or below during cycle';
        return;
    end

    % ---------------------------------------------------------------------
    % 7) Cycle efficiency (including self-discharge recovery)
    % ---------------------------------------------------------------------
    t  = result.t;
    P  = result.P_cmd;     % [W], + to grid
    Pl = result.P_total;   % [W], total losses

    % Energy delivered to grid
    E_del  = trapz(t, max(P, 0));      % [J]

    % Energy charged from grid
    E_chg  = trapz(t, max(-P, 0));     % [J]

    % Stored energy at start and end
    E0 = 0.5 * J_tot * result.omega(1)^2;
    Ef = 0.5 * J_tot * result.omega(end)^2;

    % Extra energy needed to bring SoC back to initial (self-discharge recovery)
    E_recover = max(0, E0 - Ef);

    % Round-trip efficiency
    eta_cycle = E_del / (E_chg + E_recover);

    % ---------------------------------------------------------------------
    % 8) Fill metrics for feasible design
    % ---------------------------------------------------------------------
    metrics.feasible                   = true;
    metrics.reason_if_infeasible       = "";
    metrics.m_rotor_kg                 = m_tot;
    metrics.E_cap_J                    = E_cap;
    metrics.specific_energy_kWh_per_kg = spec_energy_kWh_per_kg;
    metrics.P_rated_W                  = P_rated;
    metrics.specific_power_kW_per_kg   = spec_power_kW_per_kg;
    metrics.eta_cycle                  = eta_cycle;
    metrics.T_rotor_rated_K            = T_rotor_rated_K;
    metrics.P_loss_rated_W             = P_total_rated;
    metrics.SoC_min                    = soc_min;
end
