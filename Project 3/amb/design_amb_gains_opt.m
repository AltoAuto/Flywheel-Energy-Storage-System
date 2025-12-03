function amb_gains = design_amb_gains_opt(p_base, choice_opt)
%DESIGN_AMB_GAINS_OPT  AMB controller gains for optimal flywheel design.
%
%   amb_gains = design_amb_gains_opt(p_base, choice_opt)
%
% Inputs:
%   p_base     : baseline params from params()
%   choice_opt : struct from deliverable2b() with fields:
%                   .tMag_m, .rpm_max
%
% Output struct amb_gains with:
%   amb_gains.p_base,    amb_gains.p_opt
%   amb_gains.geom_base, amb_gains.geom_opt
%   amb_gains.amb_base,  amb_gains.amb_opt
%   amb_gains.base.radial,  amb_gains.base.tilt
%   amb_gains.opt.radial,   amb_gains.opt.tilt
%
% Each gains struct has fields:
%   Kp, Ki, Kd, wp, Kpc, Kic

    % ---------------- Baseline design ----------------
    p_base_local = p_base;   % avoid modifying caller's struct

    geom_base = compute_geometry(p_base_local);
    geom_base = compute_inertia(p_base_local, geom_base);

    amb_base  = build_amb_params(p_base_local, geom_base);

    % ---------------- Optimal design from choice_opt ----------------
    p_opt = p_base;  % copy baseline, then overwrite design vars

    p_opt.magnet_thickness_m             = choice_opt.tMag_m;
    p_opt.tMag                           = choice_opt.tMag_m;
    p_opt.max_rotational_speed_rpm       = choice_opt.rpm_max;
    p_opt.max_rotational_speed_rad_per_s = choice_opt.rpm_max * 2*pi/60;
    p_opt.omega_max                      = p_opt.max_rotational_speed_rad_per_s;
    p_opt.rpm_max                        = choice_opt.rpm_max;

    geom_opt = compute_geometry(p_opt);
    geom_opt = compute_inertia(p_opt, geom_opt);

    amb_opt  = build_amb_params(p_opt, geom_opt);

    % ---------------- Mass / inertia scaling ----------------
    m_base = amb_base.m_rotor_kg;
    m_opt  = amb_opt.m_rotor_kg;

    mass_ratio = m_opt / m_base;

    % Tilt inertia ratio (should match mass_ratio since radius is same)
    R_lever     = amb_base.rotor_diameter_m / 2;
    J_theta_base = 0.5 * m_base * R_lever^2;
    J_theta_opt  = 0.5 * m_opt  * R_lever^2;
    inertia_ratio = J_theta_opt / J_theta_base;

    % ---------------- Baseline gains (Appendix B) ----------------
    % Current controller (same for radial & tilt)
    Kpc_base = 345;
    Kic_base = 2149;

    % Radial position controller:
    %   F_x(s)/e_x(s) = kpx + kix/s + s*kdx/(1 + s/wp)
    radial_base.Kp  = 1.2639e8;
    radial_base.Ki  = 1.16868e9;
    radial_base.Kd  = 252790;
    radial_base.wp  = 3770;      % [rad/s]
    radial_base.Kpc = Kpc_base;
    radial_base.Kic = Kic_base;

    % Tilting position controller:
    %   F_alpha(s)/e_alpha(s) = kpα + kiα/s + s*kdα/(1 + s/ωpα)
    tilt_base.Kp  = 7.6992e7;
    tilt_base.Ki  = 1.18953e9;
    tilt_base.Kd  = 80294;
    tilt_base.wp  = 6283;       % [rad/s]
    tilt_base.Kpc = Kpc_base;
    tilt_base.Kic = Kic_base;

    % ---------------- Optimal gains (scaled) ----------------
    % Simple design rule:
    %   Kp_opt = Kp_base * mass_ratio
    %   Ki_opt = Ki_base * mass_ratio
    %   Kd_opt = Kd_base * mass_ratio
    %
    % This keeps the closed-loop stiffness ~ proportional to M, so that
    % the closed-loop bandwidth stays comparable when rotor mass changes.

    radial_opt      = radial_base;
    radial_opt.Kp   = radial_base.Kp * mass_ratio;
    radial_opt.Ki   = radial_base.Ki * mass_ratio;
    radial_opt.Kd   = radial_base.Kd * mass_ratio;
    % wp, Kpc, Kic unchanged

    tilt_opt        = tilt_base;
    tilt_opt.Kp     = tilt_base.Kp * inertia_ratio;
    tilt_opt.Ki     = tilt_base.Ki * inertia_ratio;
    tilt_opt.Kd     = tilt_base.Kd * inertia_ratio;
    % wp, Kpc, Kic unchanged

    % ---------------- Pack output ----------------
    amb_gains = struct();

    amb_gains.p_base    = p_base_local;
    amb_gains.p_opt     = p_opt;
    amb_gains.geom_base = geom_base;
    amb_gains.geom_opt  = geom_opt;
    amb_gains.amb_base  = amb_base;
    amb_gains.amb_opt   = amb_opt;

    amb_gains.base.radial = radial_base;
    amb_gains.base.tilt   = tilt_base;
    amb_gains.opt.radial  = radial_opt;
    amb_gains.opt.tilt    = tilt_opt;
end
