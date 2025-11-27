function geom = compute_inertia(p, geom)
%COMPUTE_INERTIA  Compute masses and polar inertia of rotating group.
%
%   geom = compute_inertia(p, geom)
%
% Uses:
%   - Geometry from compute_geometry(p)
%   - Material properties from p (rho_comp, rho_steel, rho_mag)
%
% Populates:
%   geom.m_flywheel_kg       [kg]
%   geom.m_shaft_kg          [kg]
%   geom.m_magnets_kg        [kg]
%   geom.m_rotor_total_kg    [kg]   (flywheel + shaft + magnets)
%   geom.J_flywheel_kgm2     [kg·m^2]
%   geom.J_shaft_kgm2        [kg·m^2]
%   geom.J_magnets_kgm2      [kg·m^2]
%   geom.J_total_kgm2        [kg·m^2]
%
% Assumptions (explicit):
%   - Flywheel is a uniform composite ring: inner radius = R_fw_in,
%     outer radius = R_fw_out, length = L_fw.
%   - Shaft is a solid steel cylinder of radius R_shaft and axial length:
%         L_shaft_total = L_fw + 2*L_motor + 4*axial_clearance
%     matching the reference script.
%   - Magnets form a cylindrical ring on the motor rotor:
%         inner radius  = R_shaft
%         outer radius  = R_shaft + tMag
%         axial length  = motor_axial_length_m
%     This uses tMag as an added radial thickness (embedded magnets).
%   - Inertia formulas:
%       J_solid_cyl  = 0.5 * m * R^2
%       J_thick_ring = 0.5 * m * (R_o^2 + R_i^2)
%

    %% -----------------------------
    %  1) Flywheel mass & inertia
    %  -----------------------------
    R_o_fw = geom.R_fw_out;
    R_i_fw = geom.R_fw_in;
    L_fw   = geom.L_fw;

    % Volume of composite annulus
    V_fw = pi * (R_o_fw^2 - R_i_fw^2) * L_fw;     % [m^3]
    m_fw = p.rho_comp * V_fw;                     % [kg]

    % Polar inertia of thick ring
    J_fw = 0.5 * m_fw * (R_o_fw^2 + R_i_fw^2);    % [kg·m^2]

    %% -----------------------------
    %  2) Shaft mass & inertia
    %  -----------------------------
    R_sh = geom.R_shaft;

    % Axial clearance from Table 1 (20 mm) – same as reference script
    axial_clearance = 0.020;   % [m], given in Table 1

    % Total shaft length: flywheel + 2×motor + 4×clearance
    L_shaft_total = L_fw + 2*geom.motor_axial_length_m + 4*axial_clearance;  % [m]

    % Shaft volume and mass
    V_shaft = pi * R_sh^2 * L_shaft_total;        % [m^3]
    m_shaft = p.rho_steel * V_shaft;              % [kg]

    % Polar inertia of solid cylinder
    J_shaft = 0.5 * m_shaft * R_sh^2;             % [kg·m^2]

    %% -----------------------------
    %  3) Magnet mass & inertia
    %  -----------------------------
    R_i_mag = R_sh;
    R_o_mag = R_sh + p.tMag;                      % [m] ASSUMPTION: radial magnet thickness
    L_mag   = geom.motor_axial_length_m;          % [m]

    % Volume and mass of magnet ring
    V_mag   = pi * (R_o_mag^2 - R_i_mag^2) * L_mag;  % [m^3]
    m_mag   = p.rho_mag * V_mag;                     % [kg]

    % Polar inertia of thick ring
    J_mag = 0.5 * m_mag * (R_o_mag^2 + R_i_mag^2);   % [kg·m^2]

    %% -----------------------------
    %  4) Totals
    %  -----------------------------
    m_total = m_fw + m_shaft + m_mag;
    J_total = J_fw + J_shaft + J_mag;

    % Store in geom
    geom.m_flywheel_kg    = m_fw;
    geom.m_shaft_kg       = m_shaft;
    geom.m_magnets_kg     = m_mag;
    geom.m_rotor_total_kg = m_total;

    geom.J_flywheel_kgm2  = J_fw;
    geom.J_shaft_kgm2     = J_shaft;
    geom.J_magnets_kgm2   = J_mag;
    geom.J_total_kgm2     = J_total;

    % For backward compatibility with previous code using geom.J_total:
    geom.J_total = J_total;
end
