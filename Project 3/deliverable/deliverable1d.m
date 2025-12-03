function out = deliverable1d(p, geom)
%DELIVERABLE1D  AMB step response at 0 rpm and 100% SoC.
%
%   out = deliverable1d(p, geom)
%
% Uses:
%   - build_amb_params(p, geom)  (wrapper around ambParameters)
%   - amb_ode_1d(...)            (closed-loop AMB ODE with cascaded controllers)
%
% Simulates a 10% rated-force disturbance step for:
%   1) Rotor stopped (0 rpm)
%   2) Rotor at 100% SoC (conceptuambParametersal: same 1-DOF model, no gyro terms)
%
% Plots (for each case):
%   - x(t) [µm]
%   - i(t) [A]
%   - F_AMB(t) [N] (saturated at ±F_rated)
%
% Returns struct 'out' with fields:
%   out.t0,   out.x0_um,   out.i0_A,   out.F0_N
%   out.t100, out.x100_um, out.i100_A, out.F100_N
%
% Notes (explicit):
%   - This is a 1-DOF radial model with no explicit speed dependence.
%     Thus 0 rpm and 100% SoC use the same ODE parameters; in a more
%     complete model, spin speed would enter via gyro/coupling terms.

    fprintf('========== Deliverable 1d: AMB Step Response ==========\n');

    % -----------------------------
    % 1) AMB parameters from EE + geometry
    % -----------------------------
    amb = build_amb_params(p, geom);

    m_rot   = amb.m_rotor_kg;
    ks      = amb.k_s_Npm;
    ki      = amb.k_i_NpA;
    R       = amb.R_Ohm;
    L       = amb.L_H;
    F_rated = amb.F_rated_N;
    I_rated = amb.ratedControlCurrent_A;  

    % -----------------------------
    % 2) Baseline controller gains 
    % -----------------------------
    % Baseline cascaded controller gains.
    Kpx = 1.2639e8;
    Kix = 1.16868e9;
    Kdx = 252790;
    wpx = 3770;      % [rad/s] derivative filter pole
    Kpc = 345;
    Kic = 2149;

    % -----------------------------
    % 3) Disturbance + simulation settings
    % -----------------------------
    F_dist = 0.10 * F_rated;              % 10% rated force step
    tspan  = linspace(0, 0.1, 5000)';     % 100 ms

    % State vector: y = [ x; xdot; i; x_int; i_int; q ]
    % Initial conditions:
    x0      = 0;
    xdot0   = 0;
    i0      = amb.biasCurrent_A;  
    x_int0  = 0;
    i_int0  = 0;
    q0      = 0;
    y0      = [x0; xdot0; i0; x_int0; i_int0; q0];

    % -----------------------------
    % 4) Simulate 0 rpm
    % -----------------------------
    fprintf('Simulating 10%% disturbance at 0 rpm...\n');
    [t0, s0] = ode45(@(t,y) amb_ode_1d(t, y, ...
        m_rot, ks, ki, R, L, ...
        Kpx, Kix, Kdx, wpx, Kpc, Kic, ...
        F_dist, F_rated, I_rated), ...
        tspan, y0);

    x0_um = s0(:,1) * 1e6;                 % [µm]
    i0_A  = s0(:,3);                       % [A]
    F0_N  = -ks * s0(:,1) + ki * s0(:,3);  % [N]
    F0_N  = max(min(F0_N, F_rated), -F_rated);  % clip at ±F_rated

    % -----------------------------
    % 5) Simulate "100% SoC" case
    % -----------------------------
    % In this 1-DOF model, rotor speed does not enter the equations
    % explicitly. We reuse the same parameters/ODE, but label this case
    % as 100% SoC to match the deliverable.
    fprintf('Simulating 10%% disturbance at 100%% SoC...\n');
    [t100, s100] = ode45(@(t,y) amb_ode_1d(t, y, ...
        m_rot, ks, ki, R, L, ...
        Kpx, Kix, Kdx, wpx, Kpc, Kic, ...
        F_dist, F_rated, I_rated), ...
        tspan, y0);

    x100_um = s100(:,1) * 1e6;                 % [µm]
    i100_A  = s100(:,3);                       % [A]
    F100_N  = -ks * s100(:,1) + ki * s100(:,3);
    F100_N  = max(min(F100_N, F_rated), -F_rated);

    % -----------------------------
    % 6) Plots
    % -----------------------------
    figure('Name','Deliverable 1d: AMB Step Response at 0 rpm');
    subplot(3,1,1);
    plot(t0*1e3, x0_um, 'LineWidth', 1.5);
    ylabel('x [\mum]');
    title('0 rpm: Displacement');

    subplot(3,1,2);
    plot(t0*1e3, i0_A, 'LineWidth', 1.5);
    ylabel('i [A]');
    title('0 rpm: Coil Current');

    subplot(3,1,3);
    plot(t0*1e3, F0_N, 'LineWidth', 1.5);
    ylabel('F_{AMB} [N]');
    xlabel('Time [ms]');
    title('0 rpm: AMB Force');

    figure('Name','Deliverable 1d: AMB Step Response at 100% SoC');
    subplot(3,1,1);
    plot(t100*1e3, x100_um, 'LineWidth', 1.5);
    ylabel('x [\mum]');
    title('100% SoC: Displacement');

    subplot(3,1,2);
    plot(t100*1e3, i100_A, 'LineWidth', 1.5);
    ylabel('i [A]');
    title('100% SoC: Coil Current');

    subplot(3,1,3);
    plot(t100*1e3, F100_N, 'LineWidth', 1.5);
    ylabel('F_{AMB} [N]');
    xlabel('Time [ms]');
    title('100% SoC: AMB Force');

    % -----------------------------
    % 7) Basic printed summary
    % -----------------------------
    fprintf('Max |x| at 0 rpm:        %.2f microns\n', max(abs(x0_um)));
    fprintf('Max |i| at 0 rpm:        %.2f A\n',       max(abs(i0_A)));
    fprintf('Max |F| at 0 rpm:        %.1f N\n',      max(abs(F0_N)));
    fprintf('Max |x| at 100%% SoC:    %.2f microns\n', max(abs(x100_um)));
    fprintf('Max |i| at 100%% SoC:    %.2f A\n',       max(abs(i100_A)));
    fprintf('Max |F| at 100%% SoC:    %.1f N\n',      max(abs(F100_N)));
    fprintf('======================================================\n\n');

    % -----------------------------
    % 8) Pack outputs
    % -----------------------------
    out.t0        = t0;
    out.x0_um     = x0_um;
    out.i0_A      = i0_A;
    out.F0_N      = F0_N;

    out.t100      = t100;
    out.x100_um   = x100_um;
    out.i100_A    = i100_A;
    out.F100_N    = F100_N;

end
