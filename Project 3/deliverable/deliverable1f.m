function out = deliverable1f(p, geom)
%DELIVERABLE1F  Rotor runout vs SoC due to mass imbalance.
%
%   out = deliverable1f(p, geom)
%
% Uses:
%   - amb/compute_runout(p, geom)
%
% Generates a plot of radial runout amplitude at the top AMB vs SoC,
% consistent with ISO1940 balance grade G2.5.

    fprintf('========== Deliverable 1f: Rotor Runout vs SoC ==========\n');

    out = compute_runout(p, geom);

    soc_pct   = out.soc_pct;
    runout_um = out.runout_um;

    % -----------------------------
    % Plot
    % -----------------------------
    figure('Name','Deliverable 1f: Rotor Runout vs SoC');
    plot(soc_pct, runout_um, 'LineWidth', 1.8);
    grid on;
    xlabel('State of Charge [%]');
    ylabel('Runout amplitude [\mum]');
    title('Rotor Runout due to Mass Imbalance vs SoC');

    % Print a few sample values
    soc_samples = [0, 25, 50, 75, 100];
    for s = soc_samples
        [~, idx] = min(abs(soc_pct - s));
        fprintf('SoC = %3.0f %%: runout = %.3f microns\n', ...
            soc_pct(idx), runout_um(idx));
    end

    fprintf('========================================================\n\n');
end
