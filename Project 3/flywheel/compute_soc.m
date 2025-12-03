function soc = compute_soc(mode, value, p, geom)
% COMPUTE_SOC  Convert between rotor speed and state-of-charge (SoC).
%
%   soc = compute_soc('omega2soc', omega, p, geom)
%       -> returns SoC fraction [0..1] for given omega [rad/s]
%
%   omega = compute_soc('soc2omega', soc, p, geom)
%       -> returns omega [rad/s] for given SoC fraction [0..1]
% 
% SoC definition (from Project 3 spec):
%   - 0% SoC at half of maximum speed: omega_min = 0.5 * omega_max
%   - 100% SoC at maximum speed:       omega_max = p.omega_max
%
% Mapping is defined using stored energy:
%   E(omega) = 0.5 * J_total * omega^2
%   SoC      = (E - E_min) / (E_max - E_min)
%
% where:
%   E_min = E(omega_min), E_max = E(omega_max).
%
% Inputs:
%   mode  - 'omega2soc' or 'soc2omega'
%   value - omega [rad/s] or SoC fraction, depending on mode
%   p     - parameter struct from params()
%   geom  - geometry struct with J_total from compute_inertia()
%
% Output:
%   soc   - SoC fraction (for 'omega2soc')
%   or
%   omega - rotor speed [rad/s] (for 'soc2omega')

    % Precompute energy endpoints
    omega_max = p.omega_max;
    omega_min = 0.5 * omega_max;

    J = geom.J_total;

    E_min = 0.5 * J * omega_min^2;
    E_max = 0.5 * J * omega_max^2;

    switch lower(mode)

        case 'omega2soc'
            omega = value;

            % Ensure no negative speeds
            omega = max(0, omega);

            % Energy at this speed
            E = 0.5 * J .* (omega.^2);

            % SoC fraction (can be vectorized)
            soc = (E - E_min) ./ (E_max - E_min);

        case 'soc2omega'
            soc_in = value;

            % Clamp SoC to [0,1]
            soc_in = max(0, min(1, soc_in));

            % Energy corresponding to this SoC
            E = E_min + soc_in .* (E_max - E_min);

            % Invert E(omega) = 0.5 * J * omega^2
            omega = sqrt(2 * E ./ J);

            soc = omega;  % reuse variable name 'soc' for output

        otherwise
            error('compute_soc:InvalidMode', ...
                  'Mode must be ''omega2soc'' or ''soc2omega''.');
    end
end
