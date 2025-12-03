function [SoC_pct, losses, T_rotor_K] = deliverable1a(p, geom)
%DELIVERABLE1A  Losses and rotor temperature vs SoC at rated power.
%
%   [SoC_pct, losses, T_rotor_K] = deliverable1a(p, geom)
%
% Uses:
%   - Rated torque from magnetic shear (calcRatedTorque)
%   - Rated power P_rated = T_rated * omega_max
%   - EE loss functions via compute_losses
%   - Radiative thermal model via compute_temperature
%

    % SoC grid [0..100%]
    SoC_pct = linspace(0,100,200).';
    soc_frac = SoC_pct / 100;   % [0..1]

    % Map SoC → omega
    omega = zeros(size(soc_frac));
    for k = 1:numel(soc_frac)
        omega(k) = compute_soc('soc2omega', soc_frac(k), p, geom);
    end

    % Rated torque and power
    T_rated = calcRatedTorque(p, geom);          % [N·m]
    P_rated = T_rated * p.omega_max;             % [W]

    % Preallocate
    P_rotor = zeros(size(omega));
    P_stator = zeros(size(omega));
    P_total = zeros(size(omega));
    T_rotor_K = zeros(size(omega));

    for k = 1:numel(omega)
        w = max(omega(k), 1e-3);

        % Torque demand to deliver P_rated at speed w
        T_demand = P_rated / w;

        I_pu = min(T_demand / T_rated, 1.0);

        [P_tot_k, P_rot_k, P_stat_k, ~, ~] = ...
            compute_losses(p, geom, I_pu, w);

        P_rotor(k)  = P_rot_k;
        P_stator(k) = P_stat_k;
        P_total(k)  = P_tot_k;

        T_rotor_K(k) = compute_temperature(p, geom, P_rot_k);
    end

    % Plot for 1a
    figure('Name','Deliverable 1a: Losses and Temperature vs SoC');
    
    subplot(2,1,1); hold on;
    plot(SoC_pct, P_rotor/1e3,  'LineWidth', 2);
    plot(SoC_pct, P_stator/1e3, 'LineWidth', 2);
    plot(SoC_pct, P_total/1e3,  '--',        'LineWidth', 2);
    xlabel('State of Charge [%]');
    ylabel('Losses [kW]');
    legend('Rotor','Stator','Total','Location','Best');
    title('Losses vs SoC');
    
    subplot(2,1,2); hold on;
    plot(SoC_pct, T_rotor_K - 273.15, 'LineWidth', 2);
    yline(p.T_max - 273.15,'k--','Max allowed');
    xlabel('State of Charge [%]');
    ylabel('Rotor Temperature [^{\circ}C]');
    title('Rotor Temperature vs SoC');

    losses.P_rotor = P_rotor;
    losses.P_stator = P_stator;
    losses.P_total = P_total;


end
