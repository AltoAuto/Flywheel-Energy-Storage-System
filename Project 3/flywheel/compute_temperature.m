function T_rotor = compute_temperature(p, geom, P_rotor_W)
%COMPUTE_TEMPERATURE  Radiative steady-state rotor temperature.
%
%   T_rotor = compute_temperature(p, geom, P_rotor_W)
%
% Inputs:
%   p            - parameter struct (must contain eps_rotor, eps_housing, Tamb)
%   geom         - geometry struct (must contain A_rotor, A_housing, viewFactor)
%   P_rotor_W    - rotor losses [W] (vector or scalar)
%
% Model:
%   Two large, gray, diffuse surfaces (rotor + housing) exchanging radiation.
%   View factor F_RH is provided in geom.viewFactor.
%
%   Net heat flow:
%      Q = (T_R^4 - T_H^4) / R_rad
%
%   where:
%      R_rad = (1-ε_R)/(ε_R*A_R)
%            + 1/(A_R*F_RH)
%            + (1-ε_H)/(ε_H*A_H)
%
%   Solve for T_R:
%      T_R^4 = T_H^4 + Q * R_rad
%
% Assumptions (explicit):
%   - Steady-state radiation only (vacuum, no convection, conduction negligible)
%   - Surfaces treated as gray, diffuse
%   - View factor provided by geom.viewFactor (e.g., 0.886 from Appendix)
%   - Only rotor losses heat the rotor (stator losses heat stator)
%
% Output:
%   T_rotor  - rotor temperature [K], same shape as P_rotor_W

    % Stefan-Boltzmann constant
    sigma = 5.670374419e-8;

    % Extract parameters
    eps_r = p.rotor_emissivity;      % rotor emissivity (Table 1)
    eps_h = p.eps_housing;    % housing emissivity (Table 1)
    T_H   = p.Tamb;           % housing temperature [K]

    % Extract geometry (already precomputed in geom)
    A_r = geom.A_rotor;       % total radiating area of rotor [m^2]
    A_h = geom.A_housing;     % inner housing area [m^2]
    F   = geom.viewFactor;    % view factor rotor → housing
    % ---- Radiation resistance network ----
    R_surf_R = (1 - eps_r) / (eps_r * A_r);
    R_space  = 1 / (A_r * F);
    R_surf_H = (1 - eps_h) / (eps_h * A_h);
    R_rad    = R_surf_R + R_space + R_surf_H;

    % ---- Solve for T_R^4 in closed form ----
    Q = P_rotor_W(:);                         % [W], column vector
    T_r4 = T_H^4 + (Q .* R_rad) / sigma;      % from Q = σ (T^4 - TH^4)/R_rad

    % Guard numerical noise
    T_r4 = max(T_r4, 0);

    % Back to temperature
    T_rotor = T_r4 .^ 0.25;                   % [K]
    T_rotor = reshape(T_rotor, size(P_rotor_W));  % match input shape
