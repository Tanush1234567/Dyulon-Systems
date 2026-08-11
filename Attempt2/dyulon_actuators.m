%% =========================================================================
%  DYULON SYSTEMS - Actuator Dynamics Reference / Analysis
%  dyulon_actuators.m   |   Rev 4
%
%  REV 4 CHANGE: rear motors now tilt (direct-drive gear+motor, not a
%  servo). There is no lateral/aileron-style actuator on the tilt axis --
%  ailerons remain the only non-motor control surface.
%
%  This script is for OFFLINE analysis (step response plots, discrete
%  coefficient inspection). The actual runtime actuator model used by the
%  plant is dyulon_actuator_dynamics.m, which is a discrete-time,
%  hardware-portable state-update function (no Simulink Transfer Fcn
%  blocks -- ports directly to embedded C as state_next = f(state,cmd,dt)).
%  Run this script to sanity-check the same physical parameters that
%  dyulon_actuator_dynamics.m consumes from the p struct.
%
%  BUG FIX vs Rev 3: this script previously referenced p.servo_damping,
%  a field that was never defined in dyulon_init.m (only p.servo_zeta
%  existed, and only for the old front-only servo model). REV 4 replaces
%  both with p.tilt_zeta / p.tilt_omega_n / p.tilt_rate_lim, defined for
%  all four motors' direct-drive tilt actuator.
%% =========================================================================

%% ---- Motor Dynamics (first-order, per motor, all 4 identical form) ----
motor_tf_num = p.motor_bandwidth;
motor_tf_den = [1, p.motor_bandwidth];

fprintf('Motor TF:  %g / (s + %g)\n', motor_tf_num, motor_tf_den(2));
fprintf('  Saturation per motor: [Omega_min(i), Omega_max(i)] -- see p.Omega_min/max\n');

%% ---- Tilt Actuator Dynamics (second-order, ALL FOUR motors, Rev 4) ----
wn   = p.tilt_omega_n;
zeta = p.tilt_zeta;
tilt_tf_num = wn^2;
tilt_tf_den = [1, 2*zeta*wn, wn^2];

fprintf('\nTilt TF (front AND rear, Rev 4):   %g / (s^2 + %g*s + %g)\n', ...
        tilt_tf_num, tilt_tf_den(2), tilt_tf_den(3));
fprintf('  Rate limit: +/- %.1f rad/s (%.0f deg/s) -- applied to COMMANDED\n', ...
        p.tilt_rate_lim, rad2deg(p.tilt_rate_lim));
fprintf('  rate before integration in dyulon_actuator_dynamics.m, matching\n');
fprintf('  the documented block order Rate Limiter -> TF -> Saturation.\n');
fprintf('  Front stops: [%.1f, %.1f] deg\n', rad2deg(p.beta_min_front), rad2deg(p.beta_max_front));
fprintf('  Rear  stops: [%.1f, %.1f] deg\n', rad2deg(p.beta_min_rear),  rad2deg(p.beta_max_rear));
fprintf('  NOTE: stops are generous PLACEHOLDER defaults -- confirm against\n');
fprintf('  the physical gear+motor mechanical limits before flight test.\n');

%% ---- Aileron Servo Dynamics (first-order -- the ONLY non-motor actuator) ----
ail_tf_num = p.aileron_bandwidth;
ail_tf_den = [1, p.aileron_bandwidth];

fprintf('\nAileron TF: %g / (s + %g)\n', ail_tf_num, ail_tf_den(2));
fprintf('  Saturation: [%.1f, %.1f] deg\n', rad2deg(p.aileron_min), rad2deg(p.aileron_max));

%% ---- Discrete-time versions (Tustin, reference only) ----
dt = 1/200;   % seconds, 200 Hz controller rate

fprintf('\n--- Discretisation check ---\n');
fprintf('Controller dt:              %.5f s (%.0f Hz)\n', dt, 1/dt);
fprintf('Actuator explicit-Euler dt_max (0.1/omega_n): %.5f s (%.0f Hz)\n', ...
        p.dt_actuator_max, 1/p.dt_actuator_max);
if dt > p.dt_actuator_max
    fprintf('  WARNING: controller dt exceeds the explicit-Euler stability bound.\n');
else
    fprintf('  OK: controller dt is within the explicit-Euler stability bound.\n');
end

s_to_z = @(num_c, den_c) c2d(tf(num_c, den_c), dt, 'tustin');

motor_tf_d   = s_to_z(motor_tf_num,   motor_tf_den);
tilt_tf_d    = s_to_z(tilt_tf_num,    tilt_tf_den);
aileron_tf_d = s_to_z(ail_tf_num,     ail_tf_den);

fprintf('\nDiscrete-time (200 Hz, Tustin -- reference only, NOT what\n');
fprintf('dyulon_actuator_dynamics.m actually runs):\n');
fprintf('Motor:   '); motor_tf_d
fprintf('Tilt:    '); tilt_tf_d
fprintf('Aileron: '); aileron_tf_d
