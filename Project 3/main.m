%% POWER GRID FLYWHEEL STORAGE - PROJECT 3
% Baseline modeling and analysis for:
%   - Losses & temperature vs SoC (Deliverable 1a)
%   - Specific energy & specific power (Deliverable 1b)
%   - Storage cycle efficiency & SoC evolution (Deliverable 1c)
%
% Created by: Thomas Ritten, Aiden Wang, Ben Lahyani, Cole Lange 

clear; clc; close all;

% ----------------- Core model setup -----------------
% Build parameter and geometry structs used by all deliverables.
p    = params();                 % from utils/params.m (material props, limits, omega_max, etc.)
geom = compute_geometry(p);      % from flywheel/compute_geometry.m (radii, lengths, areas)
geom = compute_inertia(p, geom); % from flywheel/compute_inertia.m (masses, J_total)

% ----------------- Deliverable 1a -------------------
% Losses & rotor temperature vs SoC at rated operating condition.
% Computes:
%   - SoC_pct        : 0–100% SoC grid
%   - losses.* [W]   : rotor, stator, and total machine losses vs SoC
%   - T_rotor_K [K]  : steady-state rotor temperature vs SoC (radiation-only model)
[SoC_pct, losses, T_rotor_K] = deliverable1a(p, geom);

% ----------------- Deliverable 1b -------------------
% Baseline specific performance metrics of the rotating group.
% Computes:
%   - m_rotor_total_kg           : total rotating mass
%   - E_max_J                    : usable energy between 50%–100% SoC
%   - specific_energy_kWh_per_kg : E_max normalized by mass
%   - P_rated_W                  : rated power from magnetic-shear torque
%   - specific_power_kW_per_kg   : P_rated normalized by mass
spec = deliverable1b(p, geom);

% ----------------- Deliverable 1c -------------------
% Dynamic simulation of the provided storage cycle starting from 50% SoC.
% Uses:
%   - team / baseline storage cycle (P_cmd vs time)
%   - flywheel dynamics + loss model
% Returns & prints:
%   - cycle efficiency (round-trip)
%   - charged / delivered / lost energy
%   - final SoC
%   - time histories of omega(t), SoC(t), P_cmd(t) for plotting
out1c = deliverable1c(p, geom);

% ----------------- Deliverable 1d -------------------
% AMB radial-axis step response at:
%   - 0 rpm (standstill)
%   - 100% SoC (rated speed), conceptually identical for 1-DOF model
%
% Simulates the closed-loop AMB using:
%   - position PID (Appendix B gains)
%   - current PI loop
%   - magnetic force model: F = -ks x + ki i
%
% Applies a disturbance force = 10% of rated AMB force.
%
% Returns & plots:
%   - x(t)      : radial displacement [µm]
%   - i(t)      : coil current [A]
%   - F_AMB(t)  : magnetic bearing force [N]
%
% Also prints peak displacement, current, and force for each case.
out1d = deliverable1d(p, geom);

% ----------------- Deliverable 1e -------------------
% Dynamic stiffness analysis for the closed-loop AMB in:
%   - radial direction (N/m)
%   - tilting direction (N·m/rad)
%
% Builds closed-loop transfer functions (x/F and θ/T) using:
%   - mechanical model (mass or rotational inertia)
%   - stiffness & force constants (ks, ki)
%   - position PID + current PI
%   - coil electrical dynamics (L, R)
%
% Evaluates both TFs over a log-spaced frequency grid (1–10,000 Hz)
% to compute dynamic stiffness magnitude:
%       K_radial = |F / x|      [N/m]
%       K_tilt   = |T / θ|      [N·m/rad]
%
% Generates log-log plots of radial and tilt dynamic stiffness,
% and prints sample values at 10, 100, and 1000 Hz.
out1e = deliverable1e(p, geom);

% ----------------- Deliverable 1f -------------------
out1f = deliverable1f(p, geom);

% ----------------- Deliverable 2a -------------------
% Load team cycle
[t_team, P_team, info_team] = load_cycle('team');

% Call Deliverable 2a
sweep = deliverable2a(p, t_team, P_team);

% ----------------- Deliverable 2b -------------------
choice_opt   = deliverable2b(p, sweep);

% Deliverable 3(a): controller TFs
amb_gains  = deliverable3a(p, choice_opt);

% Deliverable 3(b): performance comparison
results3b  = deliverable3b(p, choice_opt);