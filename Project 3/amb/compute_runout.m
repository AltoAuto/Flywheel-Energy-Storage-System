function out = compute_runout(p, geom)
%COMPUTE_RUNOUT  Rotor runout vs SoC due to mass imbalance (G2.5).
%
%   out = compute_runout(p, geom)
%
% Uses:
%   - build_amb_tf(p, geom)           : radial closed-loop TF x/F
%   - compute_soc('soc2omega', ...)   : SoC <-> speed mapping
%
% Approach:
%   1) ISO balance grade G2.5 → allowable eccentricity:
%        G [mm/s] = omega [rad/s] * e [mm]
%        => e = G / omega
%      with G = 2.5 mm/s (Table 1, G2.5) for the rotating group.
%   2) Unbalance force magnitude at speed omega:
%        F_ub(omega) = m_rotor * e * omega^2
%                     = m_rotor * (G/1000 / omega) * omega^2
%                     = m_rotor * (G/1000) * omega
%   3) Use closed-loop FRF x/F at that frequency:
%        |x(omega)| = |GxF(j*omega)| * F_ub(omega)
%   4) Map SoC → omega, compute |x| at each SoC, convert to microns.
%
% Returns struct 'out' with:
%   out.soc_frac        : SoC in [0,1]
%   out.soc_pct         : SoC in [%]
%   out.omega_rad_s     : rotor speed [rad/s]
%   out.f_Hz            : frequency [Hz]
%   out.runout_m        : displacement amplitude [m]
%   out.runout_um       : displacement amplitude [µm]

    % -----------------------------
    % 1) Build radial AMB transfer function x/F
    % -----------------------------
    tf_struct = build_amb_tf(p, geom);
    GxF_radial = tf_struct.radial.GxF;   % x(s)/F_dist(s)

    % -----------------------------
    % 2) SoC grid and speed mapping
    % -----------------------------
    soc_pct  = linspace(0, 100, 101).';     % 0,1,...,100 %
    soc_frac = soc_pct / 100;

    omega = zeros(size(soc_frac));          % [rad/s]

    for k = 1:numel(soc_frac)
        omega(k) = compute_soc('soc2omega', soc_frac(k), p, geom);
    end

    % Frequency in Hz
    f_Hz = omega / (2*pi);

    % -----------------------------
    % 3) Unbalance force from ISO G2.5
    % -----------------------------
    % Rotating group balance grade (ISO1940) G2.5:
    %   G = 2.5 mm/s  (Table 1)
    G_mm_per_s = 2.5;                       % [mm/s]
    G_m_per_s  = G_mm_per_s / 1000;         % [m/s]

    m_rotor = geom.m_rotor_total_kg;        % [kg] from your inertia model

    % Unbalance force magnitude:
    %   F_ub(omega) = m_rotor * G_m_per_s * omega
    F_ub = m_rotor * G_m_per_s .* omega;    % [N]

    % Guard omega = 0 → F_ub = 0
    F_ub(omega == 0) = 0;

    % -----------------------------
    % 4) Frequency response x/F at each omega
    % -----------------------------
    w_eval = omega;                          % [rad/s]
    H = squeeze(freqresp(GxF_radial, w_eval));  % complex x/F

    % |x| = |H| * |F|
    x_amp_m = abs(H) .* abs(F_ub);          % [m] displacement amplitude
    x_amp_um = x_amp_m * 1e6;               % [µm]

    % -----------------------------
    % 5) Pack outputs
    % -----------------------------
    out = struct();
    out.soc_frac    = soc_frac;
    out.soc_pct     = soc_pct;
    out.omega_rad_s = omega;
    out.f_Hz        = f_Hz;
    out.runout_m    = x_amp_m;
    out.runout_um   = x_amp_um;
end
