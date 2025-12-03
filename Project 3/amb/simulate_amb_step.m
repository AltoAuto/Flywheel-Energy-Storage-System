function resp = simulate_amb_step(p, geom, gains_radial, F_step_frac, t_end)
%SIMULATE_AMB_STEP  Closed-loop AMB step response for given gains.
%
%   resp = simulate_amb_step(p, geom, gains_radial, F_step_frac, t_end)
%
% Inputs:
%   p            : params struct
%   geom         : geometry struct (with m_rotor_total_kg)
%   gains_radial : struct with fields Kp, Ki, Kd, wp, Kpc, Kic
%   F_step_frac  : step disturbance magnitude as fraction of F_rated
%   t_end        : simulation end time [s]
%
% Output resp struct:
%   resp.t_ms    : time [ms]
%   resp.x_um    : displacement [µm]
%   resp.i_A     : current [A]
%   resp.F_N     : AMB force [N]

    amb = build_amb_params(p, geom);

    m_rot   = amb.m_rotor_kg;
    ks      = amb.k_s_Npm;
    ki      = amb.k_i_NpA;
    R       = amb.R_Ohm;
    L       = amb.L_H;
    F_rated = amb.F_rated_N;
    I_rated = amb.ratedControlCurrent_A;

    Kpx = gains_radial.Kp;
    Kix = gains_radial.Ki;
    Kdx = gains_radial.Kd;
    wpx = gains_radial.wp;
    Kpc = gains_radial.Kpc;
    Kic = gains_radial.Kic;

    F_dist = F_step_frac * F_rated;

    tspan = linspace(0, t_end, 5000)';

    % State: [x; xdot; i; x_int; i_int; q]
    x0      = 0;
    xdot0   = 0;
    i0      = amb.biasCurrent_A;
    x_int0  = 0;
    i_int0  = 0;
    q0      = 0;
    y0      = [x0; xdot0; i0; x_int0; i_int0; q0];

    [t, s] = ode45(@(t,y) amb_ode_1d(t, y, ...
        m_rot, ks, ki, R, L, ...
        Kpx, Kix, Kdx, wpx, Kpc, Kic, ...
        F_dist, F_rated, I_rated), ...
        tspan, y0);

    x_um = s(:,1) * 1e6;                      % [µm]
    i_A  = s(:,3);                            % [A]
    F_N  = -ks * s(:,1) + ki * s(:,3);        % [N]
    F_N  = max(min(F_N, F_rated), -F_rated);  % saturate

    resp = struct();
    resp.t_ms = t * 1e3;
    resp.x_um = x_um;
    resp.i_A  = i_A;
    resp.F_N  = F_N;
end
