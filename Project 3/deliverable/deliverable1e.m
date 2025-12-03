function out = deliverable1e(p, geom)
%DELIVERABLE1E  Plot dynamic stiffness in radial & tilting directions.
%
%   out = deliverable1e(p, geom)
%
% Uses:
%   - compute_dynamic_stiffness(p, geom)
%
% Generates a log-log plot of |K_dyn| vs frequency for:
%   - radial:   N/m
%   - tilting:  N·m/rad

    fprintf('========== Deliverable 1e: Dynamic Stiffness ==========\n');

    out = compute_dynamic_stiffness(p, geom);

    f    = out.f_Hz;
    Krad = out.K_radial_Npm;
    Ktilt= out.K_tilt_Nm_per_rad;

    % Plot
    figure('Name','Deliverable 1e: Dynamic Stiffness');
    loglog(f, Krad, 'LineWidth', 1.8); hold on;
    loglog(f, Ktilt, 'LineWidth', 1.8);
    grid on;
    xlabel('Frequency [Hz]');
    ylabel('|K_{dyn}|');
    legend('Radial (N/m)', 'Tilting (N·m/rad)', 'Location', 'Best');
    title('Dynamic Stiffness vs Frequency');

    % Optional: print some sample values
    f_samples = [10, 100, 1000];  % Hz
    for fk = f_samples
        [~, idx] = min(abs(f - fk));
        fprintf('f = %6.1f Hz:  K_radial = %.3e N/m,  K_tilt = %.3e N·m/rad\n', ...
            f(idx), Krad(idx), Ktilt(idx));
    end

    fprintf('======================================================\n\n');
end
