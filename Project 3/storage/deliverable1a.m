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

    % Map SoC → omega (your compute_soc)
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
        T_oper   = min(T_demand, T_rated);    % cap at T_rated

        I_pu = min(T_oper / T_rated, 1.0);

        [P_tot_k, P_rot_k, P_stat_k, ~, ~] = ...
            compute_losses(p, geom, I_pu, w);

        P_rotor(k)  = P_rot_k;
        P_stator(k) = P_stat_k;
        P_total(k)  = P_tot_k;

        T_rotor_K(k) = compute_temperature(p, geom, P_rot_k);
    end

    losses.P_rotor = P_rotor;
    losses.P_stator = P_stator;
    losses.P_total = P_total;
end
