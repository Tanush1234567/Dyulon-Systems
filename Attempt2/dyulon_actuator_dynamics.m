function [act_state_next, Omega_actual, beta_actual] = dyulon_actuator_dynamics(...
    act_state, Omega_cmd, beta_cmd, dt, p)
%DYULON_ACTUATOR_DYNAMICS  Discrete-time actuator lag/rate-limit/saturation.
%  Rev 4 -- models the REAL hardware: 4x ESC+motor (speed loop) and
%  4x direct-drive gearmotor tilt actuator (NOT a servo, no aileron-style
%  actuator on tilt -- ailerons are the only separate control surface).
%
%  HARDWARE-READY BY DESIGN:
%    This function has no persistent/global state and no Simulink-specific
%    constructs. All state is explicit in (act_state) and out
%    (act_state_next), and the only external input is dt (seconds) and the
%    parameter struct p. This is the same signature a PX4 module would use
%    per control-loop tick: state_next = f(state, cmd, dt, params).
%
%    INTERNAL SUB-STEPPING: forward-Euler integration of the tilt 2nd-order
%    dynamics requires dt < p.dt_actuator_max (~0.1/tilt_omega_n) for
%    numerical stability. If the caller's dt exceeds this (e.g. a 200 Hz
%    controller with a 25 rad/s tilt model gives dt=0.005s > dt_max=0.004s),
%    this function automatically sub-steps internally at p.dt_actuator_max
%    so the OUTER control loop rate is decoupled from the actuator model's
%    numerical stability requirement -- call this once per controller tick
%    regardless of dt; it self-subdivides. Port the same sub-stepping
%    logic to firmware if the flight computer's actuator-update rate is
%    slower than this bound (i.e. run the actuator model in a faster
%    inner loop than the outer controller, exactly as done here).
%
%  ACTUATOR STATE LAYOUT (act_state, struct):
%    act_state.Omega(4)      rad/s   current motor speed (1st-order lag state)
%    act_state.beta(4)       rad     current tilt angle   (2nd-order lag state 1)
%    act_state.beta_dot(4)   rad/s   current tilt rate    (2nd-order lag state 2)
%
%  INPUTS:
%    Omega_cmd(4x1)  rad/s   commanded motor speed (from allocator, pre-sat),
%                             order [FL, FR, RL, RR] matching p.r_motor rows
%    beta_cmd(4x1)   rad     commanded tilt angle  (from allocator, REV 4
%                             convention: 0=horizontal, pi/2=vertical)
%    dt              s       CALLER's control step (any value -- this
%                             function sub-steps internally as needed, see
%                             INTERNAL SUB-STEPPING above)
%    p                       parameter struct from dyulon_init.m
%
%  OUTPUTS:
%    act_state_next         updated actuator state for next call
%    Omega_actual(4)  rad/s  actual (lagged, saturated) motor speed -- feed to plant
%    beta_actual(4)   rad    actual (lagged, rate-limited, saturated) tilt -- feed to plant

%% --- Sub-stepping setup ---
% Decouple numerical stability of this explicit-Euler model from the
% caller's control-loop rate. n_sub is always >= 1.
n_sub  = max(1, ceil(dt / p.dt_actuator_max));
dt_sub = dt / n_sub;

Omega_i    = act_state.Omega;
beta_i     = act_state.beta;
betadot_i  = act_state.beta_dot;

beta_lo_vec = [p.beta_min_front; p.beta_min_front; p.beta_min_rear; p.beta_min_rear];
beta_hi_vec = [p.beta_max_front; p.beta_max_front; p.beta_max_rear; p.beta_max_rear];

Omega_cmd_sat = min(max(Omega_cmd, p.Omega_min(:)), p.Omega_max(:));
beta_cmd_sat  = min(max(beta_cmd,  beta_lo_vec),      beta_hi_vec);

for sub = 1:n_sub
    %% --- Motor speed: 1st-order lag, discretised, per-motor saturation ---
    % Continuous: Omega_dot = omega_c * (Omega_cmd - Omega)
    Omega_i = Omega_i + dt_sub * p.motor_bandwidth * (Omega_cmd_sat - Omega_i);
    Omega_i = min(max(Omega_i, p.Omega_min(:)), p.Omega_max(:));

    %% --- Tilt actuator: 2nd-order lag + hard rate limit, discretised ---
    % Continuous state-space (per motor), states [beta; beta_dot]:
    %   beta_dot_dot = omega_n^2*(beta_cmd - beta) - 2*zeta*omega_n*beta_dot
    % Rate limit applied to the resulting rate before integrating position --
    % models a real gearmotor's max slew rate as a hard physical constraint,
    % not just a smoothed 2nd-order response. Saturation applied to position
    % last, matching the documented block order
    % (Rate Limiter -> Transfer Fcn -> Saturation).
    betadotdot_i = p.tilt_omega_n^2 * (beta_cmd_sat - beta_i) ...
                 - 2 * p.tilt_zeta * p.tilt_omega_n * betadot_i;

    betadot_i = betadot_i + dt_sub * betadotdot_i;
    betadot_i = min(max(betadot_i, -p.tilt_rate_lim), p.tilt_rate_lim);

    beta_i = beta_i + dt_sub * betadot_i;
    beta_i = min(max(beta_i, beta_lo_vec), beta_hi_vec);
end

%% --- Pack outputs ---
act_state_next.Omega    = Omega_i;
act_state_next.beta     = beta_i;
act_state_next.beta_dot = betadot_i;

Omega_actual = Omega_i;
beta_actual  = beta_i;

end
