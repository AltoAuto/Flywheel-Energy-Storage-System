function amb = build_amb_params(p, geom)
%BUILD_AMB_TF  Collect AMB + rotor parameters for cascaded controller sims.
%
%   amb = build_amb_params(p, geom)
%
% This is for the *controller-based* architecture (Appendix B style):
%   - We do NOT build an open-loop transfer function here.
%   - We do NOT assume any mechanical damping.
%   - All damping and stabilization come from the AMB controllers
%     (position PID + current PI) implemented in the ODE.
%
% Inputs:
%   p    : parameter struct from params()
%   geom : geometry struct from compute_geometry/compute_inertia
%
% Output:
%   amb  : struct with fields you pass into your AMB ODE, e.g.:
%
%       amb.F_rated_N              [N]     rated AMB force (from spec)
%       amb.rotor_diameter_m       [m]     shaft / rotor diameter
%       amb.m_rotor_kg             [kg]    total rotating mass
%
%       amb.k_s_Npm                [N/m]   linearized stiffness constant
%       amb.k_i_NpA                [N/A]   force constant
%       amb.biasCurrent_A          [A]     bias current
%       amb.ratedControlCurrent_A  [A]     rated control current
%       amb.L_H                    [H]     coil inductance
%       amb.R_Ohm                  [Ohm]   coil resistance
%       amb.axialLength_m          [m]     AMB axial length
%
% All of these come DIRECTLY from:
%   - ambParameters(rotorDiameter, forceRating)
%   - your geometry/inertia model
% No extra physics is assumed here.

    % -----------------------------
    % 1) Inputs to ambParameters()
    % -----------------------------
    rotorDiameter_m = geom.D_shaft;     % shaft / PM diameter [m]
    F_rated_N       = p.AMB_rated_force; % AMB rated force [N] (Table A.1)

    % EE-supplied AMB parameters
    amb_ee = ambParameters(rotorDiameter_m, F_rated_N);

    % -----------------------------
    % 2) Rotor mass from your model
    % -----------------------------
    % Use the total rotating mass from your flywheel + shaft + magnets.
    % This matches the "baseline.rotating_mass" idea in the reference code.
    m_rotor_kg = geom.m_rotor_total_kg;

    % -----------------------------
    % 3) Pack into a clean struct
    % -----------------------------
    amb = struct();

    % High-level geometry / rating
    amb.F_rated_N        = F_rated_N;
    amb.rotor_diameter_m = rotorDiameter_m;
    amb.m_rotor_kg       = m_rotor_kg;

    % From ambParameters()
    amb.k_s_Npm               = amb_ee.stiffnessConstant;      % [N/m]
    amb.k_i_NpA               = amb_ee.forceConstant;             % [N/A]
    amb.biasCurrent_A         = amb_ee.biasCurrent;            % [A]
    amb.ratedControlCurrent_A = amb_ee.ratedControlCurrent;    % [A]
    amb.L_H                   = amb_ee.coilInductance;         % [H]
    amb.R_Ohm                 = amb_ee.coilResistance;         % [Ohm]
    amb.axialLength_m         = amb_ee.axialLength;            % [m]

    amb.ee_raw = amb_ee;

end
