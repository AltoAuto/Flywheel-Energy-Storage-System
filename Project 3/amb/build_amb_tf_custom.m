function tf_struct = build_amb_tf_custom(p, geom, gains_radial, gains_tilt)
%BUILD_AMB_TF_CUSTOM  AMB TFs x/F and theta/T for arbitrary gains.
%
%   tf_struct = build_amb_tf_custom(p, geom, gains_radial, gains_tilt)
%
% Returns:
%   tf_struct.radial.GxF   : x(s)/F_dist(s)    [m/N]
%   tf_struct.tilt.GthetaT : theta(s)/T_dist(s) [rad/(N·m)]

    amb = build_amb_params(p, geom);

    m_rot = amb.m_rotor_kg;
    ks    = amb.k_s_Npm;
    ki    = amb.k_i_NpA;
    L     = amb.L_H;
    R     = amb.R_Ohm;

    R_lever = amb.rotor_diameter_m / 2;
    J_theta  = 0.5 * m_rot * R_lever^2;
    ks_theta = ks * R_lever^2;   % [N·m/rad]
    ki_theta = ki * R_lever;     % [N·m/A]

    GxF_radial = build_single_axis_tf(m_rot, ks,      ki,      L, R, gains_radial);
    GthetaT    = build_single_axis_tf(J_theta, ks_theta, ki_theta, L, R, gains_tilt);

    tf_struct = struct();
    tf_struct.radial.GxF   = GxF_radial;
    tf_struct.tilt.GthetaT = GthetaT;
end


% ---------- helper ----------
function GxF = build_single_axis_tf(M, ks, ki, L, R, gains)
    s = tf('s');

    G_ci = gains.Kpc + gains.Kic/s;
    G_i_cl = G_ci / (L*s + R + G_ci);

    G_px = gains.Kp + gains.Ki/s + gains.Kd * s/(1 + s/gains.wp);

    Den = M*s^2 + ks + ki * G_i_cl * G_px;
    GxF = 1 / Den;   % x/F or theta/T
end
