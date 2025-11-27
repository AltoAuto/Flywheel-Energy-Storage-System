function geom = compute_geometry(p)
%COMPUTE_GEOMETRY  Build geometry struct for flywheel + motor + housing.
%
%   geom = compute_geometry(p)
%
% Uses baseline dimensions and clearances from:
%   - Table 1 (clearances, limits, material properties)
%   - Appendix A (rotor diameters, lengths)
%
% Populates:
%   geom.R_fw_out              [m]  flywheel outer radius
%   geom.R_fw_in               [m]  flywheel inner radius
%   geom.L_fw                  [m]  flywheel axial length
%   geom.R_shaft               [m]  shaft / PM rotor radius
%   geom.motor_diameter_m      [m]  motor rotor diameter
%   geom.motor_radius_m        [m]  motor rotor radius
%   geom.motor_axial_length_m  [m]  motor rotor axial length
%   geom.A_rotor               [m^2] total radiating rotor area (flywheel + motor)
%   geom.A_housing             [m^2] inner housing area
%   geom.R_housing_m           [m]  housing inner radius
%   geom.viewFactor            [-]  rotor→housing view factor
%
%   geom.rotor_diameter_for_EE_m      [m] diameter used in EE loss models
%   geom.rotor_axial_length_for_EE_m  [m] axial length used in EE loss models
%
% Notes:
%   - Motor rotor diameter is taken as the same as shaft/PM diameter from Appendix A.
%   - Motor axial length is taken as 0.25 m (baseline from Appendix A).
%   - Radial clearance between flywheel and housing is 20 mm (Table 1).
%   - View factor 0.886 from the reference analysis (gray-body approximation).

    % ------------------------------
    % Basic radii and lengths
    % ------------------------------
    geom.R_fw_out = 0.5 * p.D_out;   % [m] flywheel outer radius
    geom.R_fw_in  = 0.5 * p.D_shaft; % [m] flywheel inner radius (ID = shaft/PM dia)
    geom.L_fw     = p.L_fw;          % [m] flywheel axial length

    geom.D_shaft  = p.D_shaft;       % [m] shaft / PM rotor diameter
    geom.R_shaft  = 0.5 * p.D_shaft; % [m] shaft / PM rotor radius

    % Motor rotor geometry (baseline from Appendix A)
    geom.motor_diameter_m     = p.D_shaft;           % [m] motor rotor diameter
    geom.motor_radius_m       = 0.5 * geom.motor_diameter_m;
    geom.motor_axial_length_m = 0.25;                % [m] baseline motor length

    % ------------------------------
    % Rotor radiating area (flywheel + motor)
    % ------------------------------
    % Flywheel surface area: barrel + two ends
    A_fw_curved = 2 * pi * geom.R_fw_out * geom.L_fw;
    A_fw_ends   = 2 * pi * geom.R_fw_out^2;

    % Motor rotor surface area: barrel + two ends
    A_motor_curved = 2 * pi * geom.motor_radius_m * geom.motor_axial_length_m;
    A_motor_ends   = 2 * pi * geom.motor_radius_m^2;

    geom.A_rotor = A_fw_curved + A_fw_ends + A_motor_curved + A_motor_ends;

    % ------------------------------
    % Housing geometry and area
    % ------------------------------
    % Radial clearance between flywheel and housing (Table 1): 20 mm
    radial_clearance_flywheel_m = 0.020;   % [m]

    % Housing inner radius = flywheel outer radius + clearance
    geom.R_housing_m = geom.R_fw_out + radial_clearance_flywheel_m;

    % Assume housing axial length equals flywheel axial length for area
    A_house_curved = 2 * pi * geom.R_housing_m * geom.L_fw;
    A_house_ends   = 2 * pi * geom.R_housing_m^2;

    geom.A_housing = A_house_curved + A_house_ends;

    % ------------------------------
    % View factor (rotor → housing)
    % ------------------------------
    % From reference analysis / provided example:
    % F_RH ≈ 0.886 for this geometry (gray-body approximation).
    geom.viewFactor = 0.886;

    % ------------------------------
    % Geometry for EE electromagnetic functions
    % ------------------------------
    % Use motor rotor dimensions as the "rotor" seen by magnetic shear/loss models.
    geom.rotor_diameter_for_EE_m      = geom.motor_diameter_m;
    geom.rotor_axial_length_for_EE_m  = geom.motor_axial_length_m;
end
