function tf_struct = build_amb_tf(p, geom)
%BUILD_AMB_TF  Build closed-loop AMB transfer functions for radial & tilt.
%
%   tf_struct = build_amb_tf(p, geom)
%
% Uses:
%   - build_amb_params(p, geom)  → amb struct
%   - Baseline controller gains from Appendix B:
%       * Radial position loop gains (kpx, kix, kdx, ωpx)
%       * Tilting position loop gains (kpα, kiα, kdα, ωpα)
%       * Current loop gains (Kpc, Kic)
%
% Returns struct with:
%   tf_struct.radial.GxF      : x(s)/F_dist(s)    [m / N]
%   tf_struct.tilt.GthetaT    : θ(s)/T_dist(s)   [rad / (N·m)]
%   tf_struct.radial.params   : parameters used for radial axis
%   tf_struct.tilt.params     : parameters used for tilting axis

    % -----------------------------
    % AMB + rotor parameters
    % -----------------------------
    amb = build_amb_params(p, geom);

    m_rot = amb.m_rotor_kg;       % [kg]
    ks    = amb.k_s_Npm;          % [N/m]
    ki    = amb.k_i_NpA;          % [N/A]
    L     = amb.L_H;              % [H]
    R     = amb.R_Ohm;            % [Ohm]

    % Lever arm for tilt (about center of rotor)
    R_lever = amb.rotor_diameter_m / 2;   % [m]

    % Effective tilting inertia and stiffness/force constants
    %   J_theta   ≈ (1/2) m R^2
    %   ks_theta  = ks * R^2        [N·m/rad]
    %   ki_theta  = ki * R          [N·m/A]
    J_theta   = 0.5 * m_rot * R_lever^2;  % [kg·m^2]
    ks_theta  = ks * R_lever^2;           % [N·m/rad]
    ki_theta  = ki * R_lever;             % [N·m/A]

    % -----------------------------
    % Controller gains (Appendix B)
    % -----------------------------
    % Current controller (same for radial & tilt)
    %   G_ci(s) = 345 + 2149/s
    gains_current.Kpc = 345;
    gains_current.Kic = 2149;

    % Radial position controller:
    %   F_ocx(s)/e_ocx(s) = kpx + kix/s + s*kdx/(1+s/ωpx)
    %   kpx = 1.2639e8, kix = 1.16868e9, kdx = 252790, ωpx = 3770 rad/s
    gains_radial.Kp  = 1.2639e8;
    gains_radial.Ki  = 1.16868e9;
    gains_radial.Kd  = 252790;
    gains_radial.wp  = 3770;            % [rad/s]
    gains_radial.Kpc = gains_current.Kpc;
    gains_radial.Kic = gains_current.Kic;

    % Tilting position controller:
    %   F_dcx(s)/ε_dcx(s) = kpα + kiα/s + s*kdα/(1+s/ωpα)
    %   kpα = 7.6992e7, kiα = 1.18953e9, kdα = 80294, ωpα = 6283 rad/s
    gains_tilt.Kp  = 7.6992e7;
    gains_tilt.Ki  = 1.18953e9;
    gains_tilt.Kd  = 80294;
    gains_tilt.wp  = 6283;              % [rad/s]
    gains_tilt.Kpc = gains_current.Kpc;
    gains_tilt.Kic = gains_current.Kic;

    % -----------------------------
    % Build single-axis TFs
    % -----------------------------
    GxF_radial = build_single_axis_tf(m_rot, ks, ki, L, R, gains_radial);
    GthetaT    = build_single_axis_tf(J_theta, ks_theta, ki_theta, L, R, gains_tilt);

    % -----------------------------
    % Pack outputs
    % -----------------------------
    tf_struct = struct();

    tf_struct.radial.GxF = GxF_radial;
    tf_struct.radial.params = struct( ...
        'm', m_rot, 'ks', ks, 'ki', ki, ...
        'L', L, 'R', R, 'gains', gains_radial);

    tf_struct.tilt.GthetaT = GthetaT;
    tf_struct.tilt.params = struct( ...
        'J', J_theta, 'ks_theta', ks_theta, 'ki_theta', ki_theta, ...
        'L', L, 'R', R, 'gains', gains_tilt);
end


% ========================================================================
% INTERNAL HELPER: build_single_axis_tf
% ========================================================================
function GxF = build_single_axis_tf(M, ks, ki, L, R, gains)
%BUILD_SINGLE_AXIS_TF  Closed-loop transfer x(s)/F_dist(s) for 1 DOF axis.
%
% Mechanical axis model:
%   M * x¨ + ks * x = ki * i + F_dist
%
% Current loop:
%   G_ci(s) = Kpc + Kic/s
%   i(s) = [G_ci / (L s + R + G_ci)] * i_ref(s)
%
% Position loop (outer loop):
%   e_x = -x
%   G_px(s) = Kp + Ki/s + Kd*s/(1 + s/wp)
%   i_ref(s) = G_px(s) * e_x = -G_px(s) * x
%
% Combine:
%   (M s^2 + ks + ki * G_i_cl(s) * G_px(s)) x = F_dist
%   => GxF(s) = x/F_dist = 1 / [M s^2 + ks + ki * G_i_cl(s) * G_px(s)]

    s = tf('s');

    % Current controller
    G_ci = gains.Kpc + gains.Kic/s;

    % Closed-loop current (i / i_ref)
    G_i_cl = G_ci / (L*s + R + G_ci);

    % Position controller with derivative filter
    G_px = gains.Kp + gains.Ki/s + gains.Kd * s/(1 + s/gains.wp);

    % Closed-loop axis dynamics denominator
    Den = M*s^2 + ks + ki * G_i_cl * G_px;

    % Transfer function from disturbance force/torque to position/angle
    GxF = 1 / Den;
end
