%% POWER GRID FLYWHEEL STORAGE - PROJECT 3
% Created by Thomas Ritten, Aiden Wang, Ben Lahyani, Cole Lange 

clear; clc; close all;

% ----------------- Core model setup -----------------
p    = params();             % from utils/params.m
geom = compute_geometry(p);  % from flywheel/compute_geometry.m
geom = compute_inertia(p, geom);  % from flywheel/compute_inertia.m

% ----------------- Deliverable 1a -------------------
[SoC_pct, losses, T_rotor_K] = deliverable1a(p, geom);

% Plot for 1a
figure('Name','Deliverable 1a: Losses and Temperature vs SoC');

subplot(2,1,1); hold on; grid on;
plot(SoC_pct, losses.P_rotor/1e3,  'LineWidth', 2);
plot(SoC_pct, losses.P_stator/1e3, 'LineWidth', 2);
plot(SoC_pct, losses.P_total/1e3,  '--',        'LineWidth', 2);
xlabel('State of Charge [%]');
ylabel('Losses [kW]');
legend('Rotor','Stator','Total','Location','Best');
title('Losses vs SoC');

subplot(2,1,2); hold on; grid on;
plot(SoC_pct, T_rotor_K - 273.15, 'LineWidth', 2);
yline(p.T_max - 273.15,'k--','Max allowed');
xlabel('State of Charge [%]');
ylabel('Rotor Temperature [^{\circ}C]');
title('Rotor Temperature vs SoC');

% ----------------- Deliverable 1b -------------------
spec = deliverable1b(p, geom);

fprintf('========== Deliverable 1b: Specific Metrics ==========\n');
fprintf('Total rotating mass:   %.2f kg\n', spec.m_rotor_total_kg);
fprintf('Max energy capacity:   %.3f kWh\n', spec.E_max_J/3.6e6);
fprintf('Specific energy:       %.4f kWh/kg\n', spec.specific_energy_kWh_per_kg);
fprintf('Rated power:           %.2f kW\n', spec.P_rated_W/1e3);
fprintf('Specific power:        %.3f kW/kg\n', spec.specific_power_kW_per_kg);
fprintf('======================================================\n\n');

% ----------------- Deliverable 1c -------------------
[t_cycle, P_cmd_W, info_cycle] = load_cycle('baseline');   % your 2A

result = simulate_cycle(p, geom, t_cycle, P_cmd_W);        % your 2B

% Energy accounting
P = result.P_cmd;              % [W]
dt = mean(diff(result.t));

E_del = trapz(result.t, max(P, 0));      % energy delivered to grid   [J]
E_chg = trapz(result.t, max(-P, 0));     % energy absorbed from grid  [J]
E_loss = trapz(result.t, result.P_total);% loss energy                [J]

eta_cycle = E_del / (E_chg + E_loss);

fprintf('========== Deliverable 1c: Storage Cycle ==========\n');
fprintf('Cycle name:          %s\n', info_cycle.name);
fprintf('Cycle efficiency:    %.2f %%\n', 100*eta_cycle);
fprintf('Energy charged:      %.3f kWh\n', E_chg/3.6e6);
fprintf('Energy delivered:    %.3f kWh\n', E_del/3.6e6);
fprintf('Energy losses:       %.3f kWh\n', E_loss/3.6e6);
fprintf('End SoC:             %.1f %%\n', 100*result.soc(end));
fprintf('===================================================\n\n');

% Plots for 1c
figure('Name','Deliverable 1c: Cycle Dynamics');

subplot(3,1,1); grid on;
plot(result.t, result.omega,'LineWidth',1.5);
ylabel('\omega [rad/s]');
title('Speed vs Time');

subplot(3,1,2); grid on;
plot(result.t, 100*result.soc,'LineWidth',1.5);
ylabel('SoC [%]');

subplot(3,1,3); grid on;
plot(result.t, result.P_cmd/1e3,'LineWidth',1.5);
ylabel('Power [kW]');
xlabel('Time [s]');
title('Grid Power Command');
