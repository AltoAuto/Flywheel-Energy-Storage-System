function spec = deliverable1b(p, geom)
%DELIVERABLE1B  Specific energy and specific power for baseline design.
%
%   spec = deliverable1b(p, geom)
%
% Outputs:
%   spec.m_rotor_total_kg
%   spec.E_max_J
%   spec.specific_energy_kWh_per_kg
%   spec.P_rated_W
%   spec.specific_power_kW_per_kg

    m_tot = geom.m_rotor_total_kg;
    J     = geom.J_total;

    omega_max = p.omega_max;
    omega_min = 0.5 * omega_max;   % from SoC definition

    % Energy capacity between omega_min and omega_max
    E_max = 0.5 * J * (omega_max^2 - omega_min^2);   % [J]

    % Rated torque & power
    T_rated = calcRatedTorque(p, geom);  % [N·m]
    P_rated = T_rated * omega_max;       % [W]


    fprintf('========== Deliverable 1b: Specific Metrics ==========\n');
    fprintf('Total rotating mass:   %.2f kg\n', m_tot);
    fprintf('Max energy capacity:   %.3f kWh\n', E_max/3.6e6);
    fprintf('Specific energy:       %.4f kWh/kg\n', E_max / (3.6e6 * m_tot));
    fprintf('Rated power:           %.2f kW\n', P_rated/1e3);
    fprintf('Specific power:        %.3f kW/kg\n', (P_rated/1e3) / m_tot);
    fprintf('======================================================\n\n');

    spec.m_rotor_total_kg          = m_tot;
    spec.E_max_J                   = E_max;
    spec.specific_energy_kWh_per_kg= E_max / (3.6e6 * m_tot);
    spec.P_rated_W                 = P_rated;
    spec.specific_power_kW_per_kg  = (P_rated/1e3) / m_tot;
end
