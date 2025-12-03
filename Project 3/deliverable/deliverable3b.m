function results = deliverable3b(p_base, choice_opt)
%DELIVERABLE3B  Compare AMB performance: baseline vs optimal design.
%
%   results = deliverable3b(p_base, choice_opt)
%
% Uses:
%   - design_amb_gains_opt
%   - simulate_amb_step
%   - build_amb_tf_custom
%   - compute_soc
%
% Generates:
%   - Step response overlay (x, i, F)
%   - Dynamic stiffness overlay (radial & tilt)
%   - Runout vs SoC overlay (baseline vs optimal)

    fprintf('========== Deliverable 3(b): AMB Performance Comparison ==========\n');

    % ----- Gains & parameters for baseline & optimal -----
    amb_gains = design_amb_gains_opt(p_base, choice_opt);

    p_base_local = amb_gains.p_base;
    p_opt        = amb_gains.p_opt;

    geom_base = amb_gains.geom_base;
    geom_opt  = amb_gains.geom_opt;

    base_rad = amb_gains.base.radial;
    base_til = amb_gains.base.tilt;
    opt_rad  = amb_gains.opt.radial;
    opt_til  = amb_gains.opt.tilt;

    %% 1) Step response (10% F_rated, 0–100 ms)

    F_step_frac = 0.10;
    t_end = 0.1;

    resp_base = simulate_amb_step(p_base_local, geom_base, base_rad, F_step_frac, t_end);
    resp_opt  = simulate_amb_step(p_opt,        geom_opt,  opt_rad,  F_step_frac, t_end);

    figure('Name','Deliverable 3b: AMB Step Response Comparison');

    subplot(3,1,1);
    plot(resp_base.t_ms, resp_base.x_um, 'LineWidth', 1.5); hold on;
    plot(resp_opt.t_ms,  resp_opt.x_um,  'LineWidth', 1.5);
    ylabel('x [\mum]');
    title('Displacement step response (10% F_{rated})');
    legend('Baseline', 'Optimal', 'Location', 'Best');
    grid on;

    subplot(3,1,2);
    plot(resp_base.t_ms, resp_base.i_A, 'LineWidth', 1.5); hold on;
    plot(resp_opt.t_ms,  resp_opt.i_A,  'LineWidth', 1.5);
    ylabel('i [A]');
    title('Coil current response');
    legend('Baseline', 'Optimal', 'Location', 'Best');
    grid on;

    subplot(3,1,3);
    plot(resp_base.t_ms, resp_base.F_N, 'LineWidth', 1.5); hold on;
    plot(resp_opt.t_ms,  resp_opt.F_N,  'LineWidth', 1.5);
    ylabel('F_{AMB} [N]');
    xlabel('Time [ms]');
    title('AMB force response (saturated at \pmF_{rated})');
    legend('Baseline', 'Optimal', 'Location', 'Best');
    grid on;

    %% 2) Dynamic stiffness (radial & tilt)

    tf_base = build_amb_tf_custom(p_base_local, geom_base, base_rad, base_til);
    tf_opt  = build_amb_tf_custom(p_opt,        geom_opt,  opt_rad,  opt_til);

    GxF_base = tf_base.radial.GxF;
    GxF_opt  = tf_opt.radial.GxF;

    Gth_base = tf_base.tilt.GthetaT;
    Gth_opt  = tf_opt.tilt.GthetaT;

    f_Hz  = logspace(0, 4, 400).';      % 1–10 kHz
    w_rad = 2*pi * f_Hz;

    H_rad_base = squeeze(freqresp(GxF_base, w_rad));
    H_rad_opt  = squeeze(freqresp(GxF_opt,  w_rad));

    H_til_base = squeeze(freqresp(Gth_base, w_rad));
    H_til_opt  = squeeze(freqresp(Gth_opt,  w_rad));

    K_rad_base = 1 ./ abs(H_rad_base);  % [N/m]
    K_rad_opt  = 1 ./ abs(H_rad_opt);
    K_til_base = 1 ./ abs(H_til_base);  % [N·m/rad]
    K_til_opt  = 1 ./ abs(H_til_opt);

    figure('Name','Deliverable 3b: Dynamic Stiffness Comparison');

    subplot(2,1,1);
    loglog(f_Hz, K_rad_base, 'LineWidth', 1.5); hold on;
    loglog(f_Hz, K_rad_opt,  'LineWidth', 1.5);
    grid on;
    xlabel('Frequency [Hz]');
    ylabel('|K_{radial}| [N/m]');
    title('Radial dynamic stiffness');
    legend('Baseline', 'Optimal', 'Location', 'Best');

    subplot(2,1,2);
    loglog(f_Hz, K_til_base, 'LineWidth', 1.5); hold on;
    loglog(f_Hz, K_til_opt,  'LineWidth', 1.5);
    grid on;
    xlabel('Frequency [Hz]');
    ylabel('|K_{tilt}| [N·m/rad]');
    title('Tilting dynamic stiffness');
    legend('Baseline', 'Optimal', 'Location', 'Best');

    %% 3) Runout vs SoC (G2.5, same method as 1f)

    % SoC grid
    soc_pct  = linspace(0,100,101).';
    soc_frac = soc_pct/100;

    % Base design
    omega_base = zeros(size(soc_frac));
    for k = 1:numel(soc_frac)
        omega_base(k) = compute_soc('soc2omega', soc_frac(k), p_base_local, geom_base);
    end

    % Optimal design
    omega_opt = zeros(size(soc_frac));
    for k = 1:numel(soc_frac)
        omega_opt(k) = compute_soc('soc2omega', soc_frac(k), p_opt, geom_opt);
    end

    % ISO G2.5 unbalance force model
    G_mm_per_s = 2.5;
    G_m_per_s  = G_mm_per_s/1000;

    m_rot_base = geom_base.m_rotor_total_kg;
    m_rot_opt  = geom_opt.m_rotor_total_kg;

    Fub_base = m_rot_base * G_m_per_s .* omega_base;
    Fub_opt  = m_rot_opt  * G_m_per_s .* omega_opt;
    Fub_base(omega_base==0) = 0;
    Fub_opt(omega_opt==0)   = 0;

    % Evaluate radial FRF at each omega
    H_base = squeeze(freqresp(GxF_base, omega_base));
    H_opt  = squeeze(freqresp(GxF_opt,  omega_opt));

    x_run_base_m = abs(H_base) .* abs(Fub_base);
    x_run_opt_m  = abs(H_opt)  .* abs(Fub_opt);

    x_run_base_um = x_run_base_m * 1e6;
    x_run_opt_um  = x_run_opt_m  * 1e6;

    figure('Name','Deliverable 3b: Runout vs SoC Comparison');
    plot(soc_pct, x_run_base_um, 'LineWidth', 1.5); hold on;
    plot(soc_pct, x_run_opt_um,  'LineWidth', 1.5);
    grid on;
    xlabel('State of Charge [%]');
    ylabel('Runout amplitude [\mum]');
    title('Rotor runout due to mass imbalance vs SoC');
    legend('Baseline', 'Optimal', 'Location', 'Best');

    %% Pack results struct (if you want to inspect numerically)
    results = struct();
    results.step.baseline = resp_base;
    results.step.opt      = resp_opt;

    results.stiffness.f_Hz       = f_Hz;
    results.stiffness.K_rad_base = K_rad_base;
    results.stiffness.K_rad_opt  = K_rad_opt;
    results.stiffness.K_til_base = K_til_base;
    results.stiffness.K_til_opt  = K_til_opt;

    results.runout.soc_pct        = soc_pct;
    results.runout.base_um        = x_run_base_um;
    results.runout.opt_um         = x_run_opt_um;

    fprintf('===============================================================\n\n');
end
