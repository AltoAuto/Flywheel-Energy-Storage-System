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
%   geom.R_fw_out, geom.R_fw_in, geom.L_fw
%   geom.D_shaft, geom.R_shaft
%   geom.D_mag_outer, geom.R_mag_outer
%   geom.motor_diameter_m, geom.motor_radius_m, geom.motor_axial_length_m
%   geom.A_rotor, geom.A_housing, geom.R_housing_m, geom.viewFactor
%   geom.rotor_diameter_for_EE_m, geom.rotor_axial_length_for_EE_m

    % ------------------------------
    % Basic flywheel radii and lengths
    % ------------------------------
    geom.R_fw_out = 0.5 * p.D_out;    % [m] flywheel outer radius
    geom.R_fw_in  = 0.5 * p.D_shaft;  % [m] inner radius ≈ shaft/PM dia
    geom.L_fw     = p.L_fw;           % [m] flywheel axial length

    % ------------------------------
    % Shaft / PM hub geometry
    % ------------------------------
    geom.D_shaft  = p.D_shaft;        % [m]
    geom.R_shaft  = 0.5 * geom.D_shaft;

    % ------------------------------
    % Magnet outer diameter (for EM rotor)
    % ------------------------------
    geom.D_mag_outer = geom.D_shaft + 2*p.tMag;   % [m] shaft + 2*mag thickness
    geom.R_mag_outer = 0.5 * geom.D_mag_outer;

    % Motor / EM rotor geometry
    geom.motor_diameter_m     = geom.D_mag_outer;  % [m]
    geom.motor_radius_m       = geom.R_mag_outer;  % [m]
    geom.motor_axial_length_m = 0.25;              % [m] baseline motor length

    % ------------------------------
    % Rotor radiating area (flywheel + a bit of shaft)
    % ------------------------------
    % Flywheel surface: barrel + two ends
    A_fw_curved = 2 * pi * geom.R_fw_out * geom.L_fw;
    A_fw_ends   = 2 * pi * geom.R_fw_out^2;

    % Simple approximation: some shaft length is exposed and radiates.
    % We don't introduce new p-fields; just pick a shaft length ~ flywheel length.
    L_shaft_exposed = geom.L_fw;  % [m] crude but self-consistent approx
    A_shaft_curved  = 2 * pi * geom.R_shaft * L_shaft_exposed;

    % You can choose whether to include motor surface or not.
    % If you want closer to your original model, keep it:
    A_motor_curved = 2 * pi * geom.motor_radius_m * geom.motor_axial_length_m;
    A_motor_ends   = 2 * pi * geom.motor_radius_m^2;

    % Option A: rotor = flywheel + shaft + motor (richer area)
    geom.A_rotor = A_fw_curved + A_fw_ends + A_shaft_curved + ...
                   A_motor_curved + A_motor_ends;

    % If you want simpler: comment the last two terms out and use only
    % flywheel + shaft.

    % ------------------------------
    % Housing geometry and area
    % ------------------------------
    gap_fw    = 0.020;   % [m] radial clearance around flywheel (Table 1)
    gap_shaft = 0.001;   % [m] radial clearance around shaft  (Table 1)

    % Housing around flywheel
    geom.R_housing_m = geom.R_fw_out + gap_fw;      % [m] inner radius
    A_house_fw_curved = 2 * pi * geom.R_housing_m * geom.L_fw;
    A_house_fw_ends   = 2 * pi * geom.R_housing_m^2;

    % Simple shaft housing (same axial length as our "exposed" shaft)
    R_housing_shaft   = geom.R_shaft + gap_shaft;
    A_house_sh_curved = 2 * pi * R_housing_shaft * L_shaft_exposed;

    % Total housing area
    geom.A_housing = A_house_fw_curved + A_house_fw_ends + A_house_sh_curved;

    % ------------------------------
    % View factor (rotor → housing)
    % ------------------------------
    % 1.0 is a reasonable first approximation for a rotor inside a close
    % cylindrical housing. If you want to soften the radiation a bit,
    % you could use something like 0.9. Keeping 1.0 is fine and honest.
    geom.viewFactor = 1.0;

    % ------------------------------
    % Geometry for EE electromagnetic functions
    % ------------------------------
    geom.rotor_diameter_for_EE_m      = geom.D_mag_outer;           % [m]
    geom.rotor_axial_length_for_EE_m  = geom.motor_axial_length_m;  % [m]
end
