%% =========================================================================
%  DYULON SYSTEMS - Actuator Dynamics Models
%  dyulon_actuators.m
%
%  In Simulink, implement each of these as a subsystem sitting between
%  your controller output (commanded value) and the plant input
%  (actual physical state). This represents the real hardware lag.
%
%  How to implement in Simulink:
%   - Motor: Transfer Fcn block  (num=[omega_c], den=[1, omega_c])
%            + Saturation block  (Omega_min to Omega_max per motor)
%   - Tilt:  Transfer Fcn block  (2nd order: num=[omega_n^2], den=[1, 2*zeta*omega_n, omega_n^2])
%            + Rate Limiter block (rate_limit)
%            + Saturation block  (beta_min to beta_max)
%   - Ail:   Transfer Fcn block  (num=[omega_c], den=[1, omega_c])
%            + Saturation block  (aileron_min to aileron_max)
%
%  All transfer functions use the 'p' struct from dyulon_init.m.
%  After running dyulon_init.m, you can extract numerator/denominator
%  arrays for Simulink Transfer Fcn blocks as shown below.
%% =========================================================================

%% ---- Motor Dynamics (first-order, per motor) ----
% H_motor(s) = omega_c / (s + omega_c)
motor_tf_num = p.motor_bandwidth;
motor_tf_den = [1, p.motor_bandwidth];

fprintf('Motor TF:  %g / (s + %g)\n', motor_tf_num, motor_tf_den(2));
% Enter these into Simulink Transfer Fcn block:
%   Numerator:   [p.motor_bandwidth]
%   Denominator: [1, p.motor_bandwidth]
% Add a Saturation after: min = p.Omega_min(i), max = p.Omega_max(i)

%% ---- Tilt Servo Dynamics (second-order, for front motors only) ----
% H_tilt(s) = omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2)
wn   = p.servo_omega_n;
zeta = p.servo_damping;
tilt_tf_num = wn^2;
tilt_tf_den = [1, 2*zeta*wn, wn^2];

fprintf('Tilt TF:   %g / (s^2 + %g*s + %g)\n', tilt_tf_num, tilt_tf_den(2), tilt_tf_den(3));
% Enter into Simulink Transfer Fcn block:
%   Numerator:   [wn^2]
%   Denominator: [1, 2*zeta*wn, wn^2]
% Add Rate Limiter BEFORE: rising/falling rate = ±p.servo_rate_lim
% Add Saturation AFTER:    min = p.beta_min, max = p.beta_max

%% ---- Aileron Servo Dynamics (first-order) ----
% H_aileron(s) = omega_c / (s + omega_c)
ail_tf_num = p.aileron_bandwidth;
ail_tf_den = [1, p.aileron_bandwidth];

fprintf('Aileron TF: %g / (s + %g)\n', ail_tf_num, ail_tf_den(2));
% Add Saturation: min = p.aileron_min, max = p.aileron_max

%% ---- Discrete-time versions (for eventual C code generation) ----
% Sample time for controller (220 Hz -> dt = 0.00455 s, round to 5ms -> 200 Hz)
dt = 1/200;   % seconds

% Motor (bilinear / Tustin discretisation)
s_to_z = @(num_c, den_c) c2d(tf(num_c, den_c), dt, 'tustin');

motor_tf_d   = s_to_z(motor_tf_num,   motor_tf_den);
tilt_tf_d    = s_to_z(tilt_tf_num,    tilt_tf_den);
aileron_tf_d = s_to_z(ail_tf_num,     ail_tf_den);

fprintf('\nDiscrete-time (200 Hz, Tustin):\n');
fprintf('Motor:   '); motor_tf_d
fprintf('Tilt:    '); tilt_tf_d
fprintf('Aileron: '); aileron_tf_d
