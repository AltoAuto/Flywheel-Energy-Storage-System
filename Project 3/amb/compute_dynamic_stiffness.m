function out = compute_dynamic_stiffness(p, geom)
%COMPUTE_DYNAMIC_STIFFNESS  Dynamic stiffness vs frequency (radial & tilt).
%
%   out = compute_dynamic_stiffness(p, geom)
%
% Uses:
%   - build_amb_tf(p, geom)
%
% Returns struct 'out' with:
%   out.f_Hz                     : frequency vector [Hz]
%   out.w_rad_s                  : angular freq. [rad/s]
%   out.K_radial_Npm             : |F/x| [N/m]
%   out.K_tilt_Nm_per_rad        : |T/theta| [N·m/rad]
%   out.GxF_radial               : tf object x/F
%   out.GthetaT_tilt             : tf object theta/T

    % Build TFs
    tf_struct = build_amb_tf(p, geom);

    GxF_radial = tf_struct.radial.GxF;   % x(s)/F(s)
    GthetaT    = tf_struct.tilt.GthetaT; % theta(s)/T(s)

    % Frequency grid (1 Hz to 10 kHz)
    f_Hz  = logspace(0, 4, 400).';      % [Hz]
    w_rad = 2*pi * f_Hz;                % [rad/s]

    % Frequency responses
    H_rad  = squeeze(freqresp(GxF_radial, w_rad));  % x/F
    H_tilt = squeeze(freqresp(GthetaT,    w_rad));  % theta/T

    % Dynamic stiffness magnitudes
    K_radial = 1 ./ abs(H_rad);         % [N/m]
    K_tilt   = 1 ./ abs(H_tilt);        % [N·m/rad]

    % Pack outputs
    out = struct();
    out.f_Hz              = f_Hz;
    out.w_rad_s           = w_rad;
    out.K_radial_Npm      = K_radial;
    out.K_tilt_Nm_per_rad = K_tilt;
    out.GxF_radial        = GxF_radial;
    out.GthetaT_tilt      = GthetaT;
end
