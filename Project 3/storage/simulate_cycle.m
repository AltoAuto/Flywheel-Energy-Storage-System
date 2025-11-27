function result = simulate_cycle(p, geom, t, P_cmd)
%SIMULATE_CYCLE  Simulate flywheel speed, SoC, losses, and temperature over a cycle.
%
%   result = simulate_cycle(p, geom, t, P_cmd)
%
% Inputs:
%   p     - parameter struct from params()
%   geom  - geometry struct (compute_geometry + compute_inertia)
%   t     - time vector [s] (column or row)
%   P_cmd - commanded electrical power [W], same size as t
%           Convention (from project spec):
%             P_cmd > 0 : power from storage → grid (discharge)
%             P_cmd < 0 : power from grid   → storage (charge)
%
% Main assumptions (explicit):
%   - Rated torque is computed from magnetic shear at I_pu = 1 via:
%         T_rated = calcRatedTorque(p, geom)
%   - Electrical torque magnitude is |T_elec| ≈ |P_cmd| / ω.
%     Per-unit stator current:
%         I_pu = min(|T_elec| / T_rated, 1.0)
%   - compute_losses(p, geom, I_pu, omega) returns rotor/stator losses
%     and internally uses rotorSpeed in rev/min.
%   - Rotor energy balance:
%         E = 0.5 * J * ω^2
%         dE/dt = -P_cmd - P_stator - P_rotor
%         ⇒ J*ω*dω/dt = -P_cmd - P_stator - P_rotor
%   - Rotor temperature is computed from rotor losses only
%     (vacuum, radiation-only model in compute_temperature).
%
% Outputs (fields of result):
%   .t        - time vector [s]
%   .P_cmd    - commanded power [W]
%   .omega    - rotor speed [rad/s]
%   .soc      - state of charge [0..1]
%   .P_rotor  - rotor losses [W]
%   .P_stator - stator losses [W]
%   .P_total  - total losses [W]
%   .T_rotor  - rotor temperature [K]
%   .I_pu     - per-unit stator current history
%   .T_rated  - rated torque used for I_pu mapping [N·m]

    % ---- Basic checks ----
    if numel(t) ~= numel(P_cmd)
        error('simulate_cycle:SizeMismatch', ...
              't and P_cmd must have the same length.');
    end

    t     = t(:);
    P_cmd = P_cmd(:);
    N     = numel(t);

    J = geom.J_total;

    % ---- Rated torque from shear at I_pu = 1 ----
    T_rated = calcRatedTorque(p, geom);   % [N·m]

    % ---- Preallocate ----
    omega     = zeros(N,1);
    soc       = zeros(N,1);
    P_rotor   = zeros(N,1);
    P_stator  = zeros(N,1);
    P_total   = zeros(N,1);
    T_rot     = zeros(N,1);
    I_pu_hist = zeros(N,1);

    % ---- Initial condition from SoC ----
    soc0   = p.initial_state_of_charge_frac;
    omega0 = compute_soc('soc2omega', soc0, p, geom);
    omega(1) = omega0;
    soc(1)   = soc0;

    % ---- Time stepping ----
    for k = 1:N-1
        dt_k    = t(k+1) - t(k);
        omega_k = max(omega(k), 1e-3);   % avoid divide-by-zero
        Pk      = P_cmd(k);

        % Electrical torque from power command (approx, see header)
        T_elec = -Pk / omega_k;          % [N·m]

        % Per-unit current from torque ratio
        I_pu = min(abs(T_elec) / T_rated, 1.0);
        I_pu_hist(k) = I_pu;

        % Losses (rotor + stator)
        [P_tot_k, P_rot_k, P_stat_k, ~, ~] = ...
            compute_losses(p, geom, I_pu, omega_k);

        P_rotor(k)  = P_rot_k;
        P_stator(k) = P_stat_k;
        P_total(k)  = P_tot_k;

        % Rotor temperature from rotor losses only
        T_rot(k) = compute_temperature(p, geom, P_rot_k);

        % Rotor dynamics: J*ω*dω/dt = -P_cmd - P_stator - P_rotor
        domega_dt = (-Pk - P_stat_k - P_rot_k) / (J * omega_k);
        omega_next = omega_k + domega_dt * dt_k;
        omega_next = max(0, omega_next);

        omega(k+1) = omega_next;
        soc(k+1)   = compute_soc('omega2soc', omega_next, p, geom);
    end

    % Last point: hold last computed losses/temp/current
    if N > 1
        P_rotor(end)   = P_rotor(end-1);
        P_stator(end)  = P_stator(end-1);
        P_total(end)   = P_total(end-1);
        T_rot(end)     = T_rot(end-1);
        I_pu_hist(end) = I_pu_hist(end-1);
    end

    % ---- Pack results ----
    result.t        = t;
    result.P_cmd    = P_cmd;
    result.omega    = omega;
    result.soc      = soc;
    result.P_rotor  = P_rotor;
    result.P_stator = P_stator;
    result.P_total  = P_total;
    result.T_rotor  = T_rot;
    result.I_pu     = I_pu_hist;
    result.T_rated  = T_rated;
end
