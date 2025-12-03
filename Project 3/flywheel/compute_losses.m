function [P_total, P_rotor, P_stator, tau_loss, shear_stress] = ...
    compute_losses(p, geom, I_stator, omega)
% COMPUTE_LOSSES  Wrapper around EE loss functions for rotor + stator.
%
%   [P_total, P_rotor, P_stator, tau_loss, shear_stress] = ...
%       compute_losses(p, geom, I_stator, omega)
%
% Inputs:
%   p         - parameter struct from params()
%   geom      - geometry struct from compute_geometry/compute_inertia
%   I_stator  - stator current [A] (can be scalar or vector)
%   omega     - rotor speed [rad/s] (can be scalar or vector)
%
% Outputs:
%   P_total      - total loss power [W] = P_rotor + P_stator
%   P_rotor      - rotor loss power [W]
%   P_stator     - stator loss power [W]
%   tau_loss     - equivalent loss torque [N·m] = P_total ./ omega
%                  (set to 0 where omega == 0)
%   shear_stress - magnetic shear stress [Pa] from magneticShear()
%
% Notes:
%   - Uses the EE functions provided:
%       rotorLosses(mag_thick, rotor_diam, axial_len, I_stator, omega)
%       statorLosses(mag_thick, rotor_diam, axial_len, I_stator, omega)
%       magneticShear(mag_thick, I_stator)
%   - This function is vectorized: I_stator and omega can be arrays.

    % ---- Unpack key geometry for EE functions ----
    mag_thickness = p.magnet_thickness_m;              % [m]
    D_rotor       = geom.D_shaft;      % [m]
    L_ax          = geom.rotor_axial_length_for_EE_m;  % [m]
    omega_rpm  = omega * 60 / (2*pi);
    % ---- Rotor and stator losses [W] ----
    P_rotor  = rotorLosses( mag_thickness, D_rotor, L_ax, I_stator, omega_rpm  );
    P_stator = statorLosses( mag_thickness, D_rotor, L_ax, I_stator, omega_rpm  );

    % ---- Total loss power [W] ----
    P_total = P_rotor + P_stator;

    % ---- Equivalent loss torque [N·m] ----
    % tau = P / omega, guard against omega = 0.
    eps_omega = 1e-9;
    omega_safe = omega ;
    omega_safe(abs(omega_safe) < eps_omega) = eps_omega;
    tau_loss = P_total ./ omega_safe;

    % Force tau_loss = 0 exactly where omega == 0 (no rotation).
    zero_omega_idx = (abs(omega_rpm ) < eps_omega);
    tau_loss(zero_omega_idx) = 0;

    % ---- Magnetic shear stress [Pa] ----
    % Does not depend on omega in the given function signature.
    shear_stress = magneticShear(mag_thickness, I_stator);

end
