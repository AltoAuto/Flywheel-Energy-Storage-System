function T_rated = calcRatedTorque(p, geom)
%CALCRATEDTORQUE  Compute rated torque from magnetic shear at I_pu = 1.
%
%   T_rated = calcRatedTorque(p, geom)
%
% Uses:
%   shear_rated = magneticShear(tMag, 1.0)
%   T_rated     = shear_rated * (pi/2) * D^2 * L
%
% where:
%   D = rotor (magnet) outer diameter
%   L = axial length of the magnets

    % Magnet thickness and rotor dimensions from params/geom
    tMag = p.magnet_thickness_m;           % [m]
    D    = geom.rotor_diameter_for_EE_m;   % [m]
    L    = geom.rotor_axial_length_for_EE_m; % [m]

    % Rated shear stress at I_pu = 1
    shear_rated = magneticShear(tMag, 1.0);   % [Pa = N/m^2]

    % Torque = shear * (2*pi*R*L) * R = shear * (pi/2) * D^2 * L
    T_rated = shear_rated * (pi/2) * D^2 * L; % [N·m]
end
