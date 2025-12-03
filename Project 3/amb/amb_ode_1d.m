function dy = amb_ode_1d(t, y, ...
    m_rot, ks, ki, R, L, ...
    Kpx, Kix, Kdx, wpx, Kpc, Kic, ...
    F_dist, F_rated, I_rated)
%AMB_ODE_1D  1-DOF AMB closed-loop dynamics with cascaded controllers.
%
%   dy = amb_ode_1d(t, y, m_rot, ks, ki, R, L, ...
%                   Kpx, Kix, Kdx, wpx, Kpc, Kic, ...
%                   F_dist, F_rated, I_rated)
%
% State vector:
%   y(1) = x      [m]    lateral position
%   y(2) = xdot   [m/s]  lateral velocity
%   y(3) = i      [A]    bias current
%   y(4) = x_int  [m·s]  position error integrator state
%   y(5) = i_int  [A·s]  current error integrator state
%   y(6) = q      [-]    derivative filter state
%
% Modeling notes:
%   - No physical damping term m*xddot + c*xdot; all damping via control.
%   - Outer loop: position PID -> current reference i_ref [A].
%   - Inner loop: current PI -> coil voltage.
%   - Coil:      L di/dt + R i = v_control.
%   - Magnetic force: F_amb = -ks*x + ki*i (linearized about operating point).
%   - Current reference is saturated based on rated control current:
%         I_rated = F_rated/ki in simple models, here provided explicitly
%         from ambParameters().
%   - Magnetic force is also saturated at ±F_rated.

    %#ok<*INUSD>

    % Unpack states
    x     = y(1);   % [m]
    xdot  = y(2);   % [m/s]
    i     = y(3);   % [A]
    x_int = y(4);   % [m·s]
    i_int = y(5);   % [A·s]
    q     = y(6);   % derivative filter state

    % ------------------------------
    % 1) Position loop (outer loop)
    % ------------------------------
    % Reference position = 0
    % Choose e_x = -x so that positive x → negative current command.
    e_x = -x;

    % Integrator for position error
    dx_int = e_x;

    % Derivative filter:
    %   q_dot = wpx * (e_x - q)
    %   filtered derivative ≈ q_dot
    dq   = wpx * (e_x - q);
    d_e  = dq;

    % Position PID output -> commanded force F_cmd [N]
    F_cmd  = Kpx * e_x + Kix * x_int + Kdx * d_e;

    % ------------------------------
    % 2) Saturate current reference using I_rated
    % ------------------------------
    % Unsaturated current reference from force command
    i_ref_unsat  = F_cmd  / ki;

    % Saturate based on rated control current from ambParameters()
    i_ref = max(min(i_ref_unsat, I_rated), -I_rated);

    % ------------------------------
    % 3) Current loop (inner loop)
    % ------------------------------
    e_i   = i_ref - i;    % if i < i_ref → positive error → increase voltage

    di_int = e_i;

    % PI current controller -> coil voltage
    v_control = Kpc * e_i + Kic * i_int;   % [V]

    % ------------------------------
    % 4) Coil electrical dynamics
    % ------------------------------
    %   L di/dt + R i = v_control
    di = (v_control - R * i) / L;    % [A/s]

    % ------------------------------
    % 5) Mechanical dynamics
    % ------------------------------
    F_amb = -ks * x + ki * i;
    % Saturate force at ±F_rated
    F_amb = max(min(F_amb, F_rated), -F_rated);

    % Total force:
    xddot = (F_amb + F_dist) / m_rot;  % [m/s^2]

    % ------------------------------
    % 6) Assemble derivatives
    % ------------------------------
    dy = zeros(6,1);
    dy(1) = xdot;    % x_dot
    dy(2) = xddot;   % x_ddot
    dy(3) = di;      % i_dot
    dy(4) = dx_int;  % x_int_dot
    dy(5) = di_int;  % i_int_dot
    dy(6) = dq;      % q_dot

end
