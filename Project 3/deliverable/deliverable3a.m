function amb_gains = deliverable3a(p_base, choice_opt)
%DELIVERABLE3A  Transfer functions for baseline and optimal AMB controllers.
%
%   amb_gains = deliverable3a(p_base, choice_opt)
%
% Inputs:
%   p_base     : baseline params from params()
%   choice_opt : optimal design struct from deliverable2b()
%
% Output:
%   amb_gains  : struct from design_amb_gains_opt()

    fprintf('========== Deliverable 3(a): AMB Controller TFs ==========\n');

    amb_gains = design_amb_gains_opt(p_base, choice_opt);

    base_rad = amb_gains.base.radial;
    base_til = amb_gains.base.tilt;

    opt_rad  = amb_gains.opt.radial;
    opt_til  = amb_gains.opt.tilt;

    s = tf('s');

    % Current controller (same baseline/opt)
    G_ci = base_rad.Kpc + base_rad.Kic/s;

    fprintf('\nInner current (force) controller (baseline & optimal):\n');
    G_ci

    % Radial position controllers
    G_cp_base = base_rad.Kp + base_rad.Ki/s + base_rad.Kd * s/(1 + s/base_rad.wp);
    G_cp_opt  = opt_rad.Kp  + opt_rad.Ki/s  + opt_rad.Kd  * s/(1 + s/opt_rad.wp);

    fprintf('\nRadial position controller (baseline):\n');
    G_cp_base

    fprintf('\nRadial position controller (optimal):\n');
    G_cp_opt

    % Tilting position controllers
    G_ct_base = base_til.Kp + base_til.Ki/s + base_til.Kd * s/(1 + s/base_til.wp);
    G_ct_opt  = opt_til.Kp  + opt_til.Ki/s  + opt_til.Kd  * s/(1 + s/opt_til.wp);

    fprintf('\nTilting position controller (baseline):\n');
    G_ct_base

    fprintf('\nTilting position controller (optimal):\n');
    G_ct_opt

    fprintf('=========================================================\n\n');
end
