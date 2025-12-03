function out = deliverable1c(p, geom)
%DELIVERABLE1C  Simulate baseline storage cycle and compute efficiency.
%
%   out = deliverable1c(p, geom)
%
% Uses:
%   - storage/load_cycle.m      (returns t, P_cmd, info)
%   - storage/simulate_cycle.m  (returns time histories of state and losses)
%
% Outputs (struct 'out'):
%   out.info_cycle       - struct with cycle name, etc.
%   out.result           - struct from simulate_cycle (t, omega, soc, P_cmd, P_total, ...)
%   out.E_del_J          - energy delivered to grid [J]
%   out.E_chg_J          - energy charged from grid [J]
%   out.E_loss_J         - energy lost as heat [J]
%   out.eta_cycle        - cycle efficiency [-]
%
% Also generates plots for Deliverable 1c:
%   - omega vs time
%   - SoC vs time
%   - P_cmd vs time

    % ----------------- Load storage cycle -----------------
    [t_cycle, P_cmd_W, info_cycle] = load_cycle('baseline');

    % ----------------- Simulate cycle dynamics -----------------
    result = simulate_cycle(p, geom, t_cycle, P_cmd_W);
    % Expected fields in result:
    %   result.t        [s]
    %   result.omega    [rad/s]
    %   result.soc      [-]  (0..1)
    %   result.P_cmd    [W]
    %   result.P_total  [W]  (total losses)

    % ----------------- Energy accounting -----------------
    P  = result.P_cmd;           % [W]
    t  = result.t;               % [s]

    % Energy delivered to grid: positive power
    E_del = trapz(t, max(P, 0));          % [J]

    % Energy charged from grid: negative power (take magnitude)
    E_chg = trapz(t, max(-P, 0));         % [J]

    % Loss energy
    E_loss = trapz(t, result.P_total);    % [J]

    % Cycle efficiency
    % Stored energy at start and end
    J      = geom.J_total;
    omega0 = result.omega(1);
    omegaf = result.omega(end);
    
    E0 = 0.5 * J * omega0^2;   % initial stored energy
    Ef = 0.5 * J * omegaf^2;   % final stored energy
    
    % Extra energy needed to bring SoC back to initial (self-discharge recovery)
    E_recover = max(0, E0 - Ef);
    
    % Round-trip efficiency including recovery
    eta_cycle = E_del / (E_chg + E_recover);

    % ----------------- Print summary -----------------
    fprintf('========== Deliverable 1c: Storage Cycle ==========\n');
    fprintf('Cycle name:          %s\n', info_cycle.name);
    fprintf('Cycle efficiency:    %.2f %%\n', 100*eta_cycle);
    fprintf('Energy charged:      %.3f kWh\n', E_chg/3.6e6);
    fprintf('Energy delivered:    %.3f kWh\n', E_del/3.6e6);
    fprintf('Energy losses:       %.3f kWh\n', E_loss/3.6e6);
    fprintf('End SoC:             %.1f %%\n', 100*result.soc(end));
    fprintf('===================================================\n\n');

    % ----------------- Plots for Deliverable 1c -----------------
    figure('Name','Deliverable 1c: Cycle Dynamics');

    subplot(3,1,1);
    plot(result.t, result.omega,'LineWidth',1.5);
    ylabel('\omega [rad/s]');
    title('Speed vs Time');
    grid on;

    subplot(3,1,2);
    plot(result.t, 100*result.soc,'LineWidth',1.5);
    ylabel('SoC [%]');
    grid on;

    subplot(3,1,3);
    plot(result.t, result.P_cmd/1e3,'LineWidth',1.5);
    ylabel('Power [kW]');
    xlabel('Time [s]');
    title('Grid Power Command');
    grid on;

    % ----------------- Pack outputs -----------------
    out.info_cycle = info_cycle;
    out.result     = result;
    out.E_del_J    = E_del;
    out.E_chg_J    = E_chg;
    out.E_loss_J   = E_loss;
    out.eta_cycle  = eta_cycle;
end
