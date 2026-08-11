%% =========================================================================
%  DYULON SYSTEMS - Hybrid VTOL Plant Model Initialization
%  dyulon_init.m   |   Rev 4 - independently-tilting rear motors + control allocation
%
%  DESIGN INTENT:
%  Every derived parameter is computed from primary inputs using explicit
%  expressions. Change a primary input at the top and re-run - all
%  dependent values update automatically.
%
%  REV 4 CHANGES:
%    - Rear motors now tilt (previously fixed). New local convention:
%        beta_rear = 0   -> straight DOWN  (pusher prop)
%        beta_rear = -90 -> forward         (same physical direction as front -90)
%        range: [-100, -20] deg  (never reaches pure-down or pure-forward-only)
%      Front motors UNCHANGED:
%        beta_front = 0   -> straight UP   (tractor prop)
%        beta_front = -90 -> forward
%        range: [-100, +5] deg
%      Both motors use the IDENTICAL thrust vector formula. No sign flip
%      needed - the only difference is each motor's own mechanical zero.
%    - Added PID (not just P/PD) structure for all three control loops.
%    - Added per-flight-stage tilt perturbation limits and DOF weighting
%      for control allocation (motor speed vs tilt angle).
%    - Added AoA/stall saturation parameters.
%
%  PRIMARY INPUTS (the only numbers you should ever change directly):
%    p.mass, p.V_cruise_kmh, p.V_stall_target, p.AR, p.CL_max_assumed,
%    p.TW_ratio, p.frac_front, p.D_front_in, p.D_rear_in,
%    p.Ct_front, p.Cm_front, p.Ct_rear, p.Cm_rear,
%    p.k_Ixx, p.k_Iyy, sensor noise values, actuator dynamics values,
%    p.gains.*, p.tilt_limits.*, p.alloc_weights.*
%
%  HOW TO USE:
%    Run dyulon_init.m, then dyulon_plot_luts.m to sanity check.
%    All Simulink blocks read from the 'p' workspace struct.
%  =========================================================================
clear; clc;


%Assume a wing ratio and derive the stall speed (could result to worse
%performance)

%% =========================================================================
%  0. PRIMARY DESIGN INPUTS  ← only edit numbers in this section
%% =========================================================================

% --- Mission / mass ---
p.mass           = 35.0;    % kg    total MTOW (25 kg operating + 10 kg payload)
p.V_cruise_kmh   = 125.0;   % km/h  cruise airspeed from spec

% --- Wing design targets ---
p.V_stall_target = 20.0;    % m/s   desired stall speed (sets wing area)
p.AR             = 7.5;     % -     aspect ratio (efficiency vs. structural trade)
p.CL_max_assumed = 1.05;    % -     assumed max lift coefficient (reflex airfoil)
p.e_oswald       = 0.82;    % -     Oswald efficiency factor

% --- Propulsion design targets ---
p.TW_ratio   = 1.40;    % -     hover thrust-to-weight ratio (T/W)
p.frac_front = 0.70;    % -     fraction of hover lift from front motors
p.frac_rear  = 0.30;    % -     fraction of hover lift from rear motors

% --- Prop geometry ---
p.D_front_in = 22;      % inch  front hover prop diameter
p.D_rear_in  = 18;      % inch  rear cruise prop diameter

% --- Prop aerodynamic coefficients (static, from UIUC database / eCalc) ---
p.Ct_front   = 0.105;   % -     thrust coefficient, front prop (APC 22x8MR class)
p.Cm_front   = 0.0065;  % -     torque coefficient, front prop
p.Ct_rear    = 0.092;   % -     thrust coefficient, rear prop (APC 18x5.5E class)
p.Cm_rear    = 0.0055;  % -     torque coefficient, rear prop

% --- Motor arm fractions of wingspan ---
p.y_front_frac = 0.28;  % -     front motor lateral position / half-wingspan
p.y_rear_frac  = 0.18;  % -     rear  motor lateral position / half-wingspan
p.x_front      = 0.60;  % m     front motor forward distance from CG
p.x_rear       = -0.80; % m     rear  motor aft     distance from CG (negative)
p.z_motor      = 0.03;  % m     motor vertical offset below CG

% --- Inertia scaling factors (k such that I = k * mass * b²) ---
% From empirical fits to similar fixed-wing UAVs in 20-50 kg class
p.k_Ixx = 0.030;   % roll  inertia factor
p.k_Iyy = 0.018;   % pitch inertia factor

% --- Hover thrust split (fraction front vs rear) ---
% These two must sum to 1.0
assert(abs(p.frac_front + p.frac_rear - 1.0) < 1e-6, ...
       'frac_front + frac_rear must equal 1.0');

%% =========================================================================
%  1. PHYSICAL CONSTANTS
%% =========================================================================
p.g   = 9.81;    % m/s²
p.rho = 1.225;   % kg/m³  ISA sea level

%% =========================================================================
%  2. DERIVED MISSION PARAMETERS
%% =========================================================================
p.V_cruise = p.V_cruise_kmh / 3.6;   % m/s
p.W        = p.mass * p.g;            % N   vehicle weight

%% =========================================================================
%  3. WING GEOMETRY  (all derived from primary inputs)
%% =========================================================================
% Wing area from stall speed requirement:
%   L = W  at stall  →  W = 0.5*rho*V_stall²*S*CL_max  →  S = 2W/(rho*V_stall²*CL_max)
p.S    = 2 * p.W / (p.rho * p.V_stall_target^2 * p.CL_max_assumed);
p.b    = sqrt(p.AR * p.S);       % m   wingspan
p.cbar = p.S / p.b;              % m   mean aerodynamic chord
% AR is already set as primary input; confirm it round-trips:
% p.AR == p.b^2 / p.S  ← always true by construction above

%% =========================================================================
%  4. MASS AND INERTIA  (derived)
%% =========================================================================
p.Ixx = p.k_Ixx * p.mass * p.b^2;         % kg·m²  roll
p.Iyy = p.k_Iyy * p.mass * p.b^2;         % kg·m²  pitch
p.Izz = p.Ixx + p.Iyy;                    % kg·m²  yaw (flat plate approx)
p.Ixy = 0; p.Ixz = 0; p.Iyz = 0;

p.I_mat = [p.Ixx, p.Ixy, p.Ixz;
           p.Ixy, p.Iyy, p.Iyz;
           p.Ixz, p.Iyz, p.Izz];

%% =========================================================================
%  5. MOTOR GEOMETRY  (derived from wingspan)
%% =========================================================================
y_front = p.y_front_frac * (p.b / 2);   % m  lateral arm, front motors
y_rear  = p.y_rear_frac  * (p.b / 2);   % m  lateral arm, rear motors

p.r_motor = [ p.x_front, -y_front,  p.z_motor;   % 1: Front-Left
              p.x_front,  y_front,  p.z_motor;   % 2: Front-Right
              p.x_rear,  -y_rear,   p.z_motor;   % 3: Rear-Left
              p.x_rear,   y_rear,   p.z_motor];  % 4: Rear-Right

p.spin_dir   = [-1, 1, 1, -1];   % CW/CCW from above, counter-rotating pairs
p.motor_type = [ 1, 1, 2,  2];   % 1=hover prop  2=cruise prop
p.has_tilt   = [ 1, 1, 1,  1];   % REV4: all four motors now tilt

%% =========================================================================
%  6. TILT SERVO  (primary inputs; dynamics from Mancinelli bench data)
%
%  REV 4 SIGN CONVENTION (confirmed, do not re-derive):
%    Both front and rear motors use the IDENTICAL thrust vector formula:
%        T_vec_i = T_i * [-sin(beta_i); 0; -cos(beta_i)]
%    The only difference between motor types is where beta=0 physically
%    points, because the rear motors are pusher props mounted backwards
%    relative to the front tractor props:
%
%      FRONT (tractor):  beta_front = 0   -> straight UP
%                         beta_front = -90 -> forward
%                         range: [-100, +5] deg   (mostly negative = useful)
%
%      REAR (pusher):    beta_rear  = 0   -> straight DOWN
%                         beta_rear  = -90 -> forward (same physical direction
%                                              as front's -90, both converge
%                                              to pure forward thrust here)
%                         range: [-100, -20] deg  (never reaches pure-down
%                                              or pure-forward-only; always
%                                              retains a vertical+forward mix)
%
%    Hover/takeoff/landing trim sits near the negative limit for rear
%    (close to -100 deg, i.e. close to forward-pointing but with a strong
%    downward component still present) and near 0 deg for front (near
%    straight-up). This gives both motor groups a net-forward thrust
%    component even in hover, by design.
%% =========================================================================

% --- Front motor tilt limits (tractor, zero = up) ---
p.beta_front_min  = deg2rad(-100);  % rad  hard stop (near-forward)
p.beta_front_max  = deg2rad(5);     % rad  hard stop (just past vertical)
p.beta_front_rest = deg2rad(0);     % rad  hover/takeoff/landing trim (straight up)

% --- Rear motor tilt limits (pusher, zero = down) ---
%
%  Range [-100, -20] deg:
%    -20 deg = hover/takeoff/landing trim
%              -> 94% upward (lift) + 34% forward. Rear motors contribute
%                 meaningfully to hover lift while retaining a forward bias.
%    -90 deg = pure forward thrust (cruise target)
%    -100 deg = negative hard stop, gives ±10 deg perturbation room around
%               -90 in cruise without crossing into net-downward territory
%               (anything past -90 pushes vehicle slightly down).
%
%  Blend schedule drives beta_rear from rest (-20) to cruise (-90):
%    beta_rear_cmd = beta_rear_rest + (-pi/2 - beta_rear_rest) * blend
%                  = -20 + (-90 - (-20)) * blend  [deg]
%                  = -20 - 70 * blend
p.beta_rear_min   = deg2rad(-100);  % rad  hard stop (10 deg past forward, perturbation limit)
p.beta_rear_max   = deg2rad(-20);   % rad  hard stop (hover trim, mostly-up with fwd bias)
p.beta_rear_rest  = deg2rad(-20);   % rad  hover/takeoff/landing trim

% Legacy combined fields (kept for any code still expecting beta_min/beta_max
% as a single front-oriented range; front values used since front is the
% historically "primary" tilting surface)
p.beta_min = p.beta_front_min;
p.beta_max = p.beta_front_max;

% Per-motor min/max/rest, indexed [FL, FR, RL, RR] - used directly by
% the control allocation / saturation logic so nothing needs an if-statement
% on motor index inside the controller blocks.
% Per-motor arrays indexed [FL, FR, RL, RR]
% Note: beta_rear_max = -20 deg (least negative = closest to hover trim = upper bound)
%       beta_rear_min = -100 deg (most negative = closest to forward/past = lower bound)
%       This is numerically correct: min < max, i.e. -100 < -20. Saturation
%       clamp in the controller uses these directly: max(min(beta, max), min).
p.beta_motor_min  = [p.beta_front_min,  p.beta_front_min,  p.beta_rear_min,  p.beta_rear_min];
p.beta_motor_max  = [p.beta_front_max,  p.beta_front_max,  p.beta_rear_max,  p.beta_rear_max];
p.beta_motor_rest = [p.beta_front_rest, p.beta_front_rest, p.beta_rear_rest, p.beta_rear_rest];

p.servo_omega_n  = 55;             % rad/s WHAT IS THIS
p.servo_zeta     = 1.5;            % overdamped
p.servo_rate_lim = 11.0;           % rad/s  max tilt rate

%% =========================================================================
%  7. PROPELLER COEFFICIENTS  (derived from Ct/Cm/D)
%% =========================================================================
D_front = p.D_front_in * 0.0254;   % m  convert inches to metres
D_rear  = p.D_rear_in  * 0.0254;   % m

% KpT = Ct * rho * D^4 / (4*pi²)    [N / (rad/s)²]    %WHAT IS THIS
% KpM = Cm * rho * D^5 / (4*pi²)    [N·m / (rad/s)²]  % WHAT IS THIS
p.KpT_front_0 = p.Ct_front * p.rho * D_front^4 / (4*pi^2);
p.KpM_front_0 = p.Cm_front * p.rho * D_front^5 / (4*pi^2);
p.KpT_rear_0  = p.Ct_rear  * p.rho * D_rear^4  / (4*pi^2);
p.KpM_rear_0  = p.Cm_rear  * p.rho * D_rear^5  / (4*pi^2);

%% =========================================================================
%  8. MOTOR SPEED LIMITS  (derived from hover thrust requirement)
%% =========================================================================
T_total_hover   = p.TW_ratio * p.W;                        % N
T_front_each    = p.frac_front * T_total_hover / 2;        % N per front motor
T_rear_each     = p.frac_rear  * T_total_hover / 2;        % N per rear motor

% Hover operating point: Omega = sqrt(T / KpT_0)
p.Omega_hover_front = sqrt(T_front_each / p.KpT_front_0);  % rad/s
p.Omega_hover_rear  = sqrt(T_rear_each  / p.KpT_rear_0);   % rad/s

% Max speed: 1.5× hover → 35% headroom for attitude control perturbations
%SAFETY FACTOR -> 50%


p.Omega_max_front = 1.5 * p.Omega_hover_front;
p.Omega_max_rear  = 1.5 * p.Omega_hover_rear;

p.Omega_min = [  80,   80,   80,   80];                          % rad/s keep-alive
p.Omega_max = [p.Omega_max_front, p.Omega_max_front, ...
               p.Omega_max_rear,  p.Omega_max_rear];             % rad/s

p.motor_bandwidth = 25;   % rad/s  first-order motor model corner frequency

%% =========================================================================
%  9. PROPELLER AIRSPEED LUTs  (scale factors on KpT_0 and KpM_0)
%% =========================================================================
p.prop_V_bp = [0,    5,    10,   15,   20,   25,   35  ];  % m/s

p.KpT_front_scale = [1.00, 0.89, 0.78, 0.67, 0.56, 0.46, 0.30]; % HOW ARE THESE DERIVED
p.KpT_rear_scale  = [1.00, 0.94, 0.88, 0.82, 0.75, 0.68, 0.55];
p.KpM_front_scale = p.KpT_front_scale;
p.KpM_rear_scale  = p.KpT_rear_scale;

%% =========================================================================
%  10. WING AERODYNAMICS LUTs
%% =========================================================================
p.aero_alpha_deg = [-8,  -6,  -4,  -2,   0,   2,   4,   6,   8,  10,  12,  14,  16,  18];

p.aero_CL = [-0.61,-0.40,-0.19, 0.00, 0.20, 0.40, 0.59, 0.76, 0.90, 1.01, 1.07, 1.00, 0.80, 0.55];
p.aero_CD = [ 0.058, 0.038, 0.025, 0.018, 0.015, 0.016, 0.020, 0.028, 0.042, 0.060, 0.086, 0.124, 0.178, 0.245];
p.aero_Cm = [-0.008,-0.010,-0.012,-0.013,-0.014,-0.016,-0.018,-0.020,-0.023,-0.027,-0.031,-0.037,-0.040,-0.042];

% Aileron
p.Cl_da         = 0.17;          % roll moment per radian aileron
p.aileron_max   = deg2rad(25);
p.aileron_min   = deg2rad(-25);
p.aileron_bandwidth = 20;        % rad/s servo

% Damping derivatives
p.Cm_q = -8.0;    % pitch rate damping
p.Cl_p = -0.45;   % roll  rate damping
p.Cn_r = -0.12;   % yaw   rate damping
p.Cl_b = -0.08;   % dihedral effect
p.Cn_b =  0.005;  % weathercock (weak - no vertical fin)  #WHAT IS THIS

%% =========================================================================
%  11. TRANSITION BLEND SCHEDULE  (derived from stall speed)
%% =========================================================================
p.V_blend_lo = 10.0;                         % m/s  (fixed: wing starts ~18% lift here)
p.V_blend_hi = p.V_stall_target * 1.10;      % m/s  derived: 10% above target stall

%% =========================================================================
%  12. SENSOR NOISE AND DELAYS
%% =========================================================================

%ALL ASSUMED VALUES

p.noise.pos_xy     = 0.50;           % m
p.noise.pos_z      = 0.80;           % m
p.noise.vel        = 0.10;           % m/s
p.noise.att_roll   = deg2rad(0.4);   % rad
p.noise.att_pitch  = deg2rad(0.4);   % rad
p.noise.att_yaw    = deg2rad(0.8);   % rad
p.noise.body_rate  = deg2rad(0.05);  % rad/s
p.noise.airspeed   = 0.30;           % m/s
p.noise.AoA        = deg2rad(0.8);   % rad
p.noise.tilt_angle = deg2rad(0.3);   % rad
p.noise.rpm        = 8.0;            % rad/s

p.delay.imu   = 0.002;  % s
p.delay.gps   = 0.10;   % s
p.delay.pitot = 0.015;  % s
p.delay.tilt  = 0.005;  % s
p.delay.rpm   = 0.003;  % s

%% =========================================================================
%  13. DERIVED AERODYNAMIC QUANTITIES
%% =========================================================================
CLCD_vec             = p.aero_CL ./ p.aero_CD; %CHANGE THIS TO HAVE MORE POINTS (SO interpolate Cl and Cd to have more points and get more points for CLCD  
[~, idx_bestLD]      = max(CLCD_vec);
p.alpha_bestLD_deg   = p.aero_alpha_deg(idx_bestLD);
p.CL_bestLD          = p.aero_CL(idx_bestLD);
p.CD_bestLD          = p.aero_CD(idx_bestLD);
p.CLCD_max           = CLCD_vec(idx_bestLD);

[p.CL_max, idx_stall] = max(p.aero_CL);
p.alpha_stall_deg     = p.aero_alpha_deg(idx_stall);

p.V_stall_actual = sqrt(2 * p.W / (p.rho * p.S * p.CL_max));
p.CL_at_cruise   = p.W / (0.5 * p.rho * p.V_cruise^2 * p.S);

% Stall AoA protection band - used by inner loop to saturate theta_ref
p.alpha_protect_lo_deg = 2.0;                  % deg  lower AoA bound (avoid negative stall)
p.alpha_protect_hi_deg = p.alpha_stall_deg - 2; % deg  stay 2 deg below stall AoA as margin

%% =========================================================================
%  14. FLIGHT STAGE DEFINITION  (purely airspeed-based, single source of truth)
%
%  Three stages, driven entirely by the existing blend factor computed from
%  V_air. No separate tilt-based staging - tilt commands and limits are an
%  OUTPUT of the stage/blend, never an input that defines it. This avoids
%  any possibility of airspeed and tilt position disagreeing about what
%  stage the vehicle is in.
%
%    STAGE 1 - HOVER/TAKEOFF/LANDING : blend = 0           (V < V_blend_lo)
%    STAGE 2 - TRANSITION            : 0 < blend < 1        (V_blend_lo..V_blend_hi)
%    STAGE 3 - CRUISE                : blend = 1            (V > V_blend_hi)
%
%  Takeoff, hover, and landing are modelled as the same stage for now
%  (per project decision) - all share blend = 0 behaviour and limits.
%% =========================================================================
p.stage.HOVER      = 1;   % hover / takeoff / landing combined
p.stage.TRANSITION = 2;
p.stage.CRUISE     = 3;

%% =========================================================================
%  15. PER-STAGE TILT PERTURBATION LIMITS
%
%  These are ADDITIONAL perturbation allowances on top of each motor's
%  rest/trim position (p.beta_motor_rest), used by the control allocation
%  logic to decide how far it may move beta away from trim for attitude
%  correction at a given blend value. Independent from the hard mechanical
%  stops in Section 6 - perturbation limits are always tighter than or
%  equal to the hard stops, and are interpolated continuously vs blend
%  using the breakpoints below (so behaviour matches the continuous
%  blending philosophy even though limits are specified "per stage").
%
%  Rationale (per project decision):
%   - Hover/takeoff: REAR motors get LARGE perturbation freedom (they
%     contribute little to lift here, so swinging them doesn't risk
%     losing vertical thrust). FRONT motors get a SMALL perturbation -
%     they are the primary lift contributors in hover and must stay
%     close to their vertical trim.
%   - Cruise: FRONT motors get a LARGER (but still partial) perturbation
%     allowance since they become the secondary/pitch-trim actuator.
%     REAR motors get a SMALL perturbation - they are the primary thrust
%     contributors in cruise and must not be disturbed much.
%   - Transition: linear interpolation between the two endpoints, same
%     breakpoints as the existing V_blend_lo/V_blend_hi schedule.
%% =========================================================================

% Breakpoints reuse the existing transition schedule exactly (single
% source of truth - no separate stage breakpoints to keep in sync)
p.tilt_limits.V_bp = [p.V_blend_lo, p.V_blend_hi];   % m/s, [hover_end, cruise_start]

% Front motor perturbation allowance (deg off trim), at [hover, cruise]
p.tilt_limits.front_perturb_deg_bp = [2, 10];

% Rear motor perturbation allowance (deg off trim), at [hover, cruise]
p.tilt_limits.rear_perturb_deg_bp  = [8, 2];

% Convenience pre-converted radian versions (interp1 at runtime still uses
% the _deg_bp arrays directly against p.tilt_limits.V_bp; these are for
% any block that wants the hover/cruise endpoints directly without an
% interp1 call)
p.tilt_limits.front_perturb_rad_bp = deg2rad(p.tilt_limits.front_perturb_deg_bp);
p.tilt_limits.rear_perturb_rad_bp  = deg2rad(p.tilt_limits.rear_perturb_deg_bp);

%% =========================================================================
%  16. CONTROL ALLOCATION DOF WEIGHTING (speed vs tilt split)
%
%  When a correction (e.g. extra lift demand under AoA saturation) can be
%  met by EITHER increasing motor speed (Omega) OR adjusting tilt (beta),
%  this weight decides how much of the correction goes to each DOF.
%  Currently flat 50/50 across all motors and stages - project owner to
%  revisit and tune per-stage/per-motor weighting later.
%% =========================================================================
p.alloc_weights.omega_share = 0.5;   % fraction of correction via motor speed
p.alloc_weights.tilt_share  = 0.5;   % fraction of correction via tilt angle
assert(abs(p.alloc_weights.omega_share + p.alloc_weights.tilt_share - 1.0) < 1e-9, ...
       'omega_share + tilt_share must equal 1.0');

%% =========================================================================
%  17. PID GAINS - ALL THREE LOOPS
%
%  Full PID structure provided for every loop. Any gain can be set to 0
%  to recover P/PD/PI behaviour without changing block structure.
%  Attitude loops use direct rate feedback for the derivative term
%  (Kd * rate_measured) rather than differentiating the angle error, to
%  avoid derivative noise and to keep gains directly portable to
%  ArduPilot's rate-feedback architecture.
%% =========================================================================

% --- Outer loop: position error -> velocity reference ---
p.gains.outer.Kp_xy = 0.5;    % 1/s
p.gains.outer.Ki_xy = 0.0;    % 1/s² (start at 0 - position integrator risks windup
                               %       through transition; raise only if steady-state
                               %       position error is observed in sim)
p.gains.outer.Kp_z  = 0.6;    % 1/s
p.gains.outer.Ki_z  = 0.0;    % 1/s²

% --- Mid loop: velocity error -> acceleration reference ---
p.gains.mid.Kp_xy = 1.0;      % (m/s²)/(m/s)
p.gains.mid.Ki_xy = 0.15;     % (m/s²)/(m/s)/s
p.gains.mid.Kp_z  = 1.5;      % (m/s²)/(m/s)
p.gains.mid.Ki_z  = 0.20;     % (m/s²)/(m/s)/s

% --- Inner loop: attitude ---
p.gains.inner.Kp_theta = 8.0;    % (rad/s²)/rad
p.gains.inner.Ki_theta = 0.5;    % (rad/s²)/rad/s
p.gains.inner.Kd_theta = 3.0;    % (rad/s²)/(rad/s)   - multiplies q_rate directly

p.gains.inner.Kp_phi   = 6.0;    % (rad/s²)/rad
p.gains.inner.Ki_phi   = 0.5;    % (rad/s²)/rad/s
p.gains.inner.Kd_phi   = 2.5;    % (rad/s²)/(rad/s)   - multiplies p_rate directly

p.gains.inner.Kp_psi   = 3.0;    % (rad/s)/rad
p.gains.inner.Ki_psi   = 0.1;    % (rad/s)/rad/s
p.gains.inner.Kd_psi   = 0.0;    % (rad/s)/(rad/s)    - yaw authority weak, Kd off by default

% Integrator anti-windup limits (symmetric, applied in Simulink at the
% Discrete Integrator block level via the "Limit output" setting - listed
% here so the value lives in one place)
p.gains.windup_limit_outer = 5.0;     % m/s   (velocity reference clamp from outer I-term)
p.gains.windup_limit_mid   = 5.0;     % m/s²  (accel reference clamp from mid I-term)
p.gains.windup_limit_inner = deg2rad(15);  % rad/s (rate command clamp from inner I-term)

%% =========================================================================
%  18. SUMMARY PRINT
%% =========================================================================
fprintf('\n=============================================\n');
fprintf(' DYULON SYSTEMS - Model Initialised (Rev 4)\n');
fprintf('=============================================\n');
fprintf('Mass:                  %.1f kg\n',   p.mass);
fprintf('Weight:                %.2f N\n',    p.W);
fprintf('\n--- Wing ---\n');
fprintf('Area S:                %.4f m²\n',  p.S);
fprintf('Wingspan b:            %.4f m\n',   p.b);
fprintf('Mean chord:            %.4f m\n',   p.cbar);
fprintf('Aspect ratio:          %.2f\n',     p.AR);
fprintf('Stall speed (actual):  %.2f m/s  (%.1f km/h)\n', p.V_stall_actual, p.V_stall_actual*3.6);
fprintf('Cruise speed:          %.3f m/s  (%.1f km/h)\n', p.V_cruise, p.V_cruise*3.6);
fprintf('CL at cruise:          %.4f\n',     p.CL_at_cruise);
fprintf('Cruise / stall margin: %.2fx\n',    p.V_cruise / p.V_stall_actual);
fprintf('Best L/D:              %.1f  at %.1f deg AoA\n', p.CLCD_max, p.alpha_bestLD_deg);
fprintf('AoA protect band:      [%.1f, %.1f] deg\n', p.alpha_protect_lo_deg, p.alpha_protect_hi_deg);
fprintf('\n--- Inertia ---\n');
fprintf('Ixx (roll):            %.3f kg·m²\n', p.Ixx);
fprintf('Iyy (pitch):           %.3f kg·m²\n', p.Iyy);
fprintf('Izz (yaw):             %.3f kg·m²\n', p.Izz);
fprintf('\n--- Props ---\n');
fprintf('Front prop:            %d in  KpT=%.4e  KpM=%.4e\n', p.D_front_in, p.KpT_front_0, p.KpM_front_0);
fprintf('Rear  prop:            %d in  KpT=%.4e  KpM=%.4e\n', p.D_rear_in,  p.KpT_rear_0,  p.KpM_rear_0);
fprintf('\n--- Hover (T/W = %.2f) ---\n', p.TW_ratio);
T_front_check = p.KpT_front_0 * p.Omega_hover_front^2;
T_rear_check  = p.KpT_rear_0  * p.Omega_hover_rear^2;
T_total_check = 2*T_front_check + 2*T_rear_check;
fprintf('Front: %.1f N each  @  %.0f rad/s  (%.0f RPM)\n', ...
        T_front_check, p.Omega_hover_front, p.Omega_hover_front*60/(2*pi));
fprintf('Rear:  %.1f N each  @  %.0f rad/s  (%.0f RPM)\n', ...
        T_rear_check,  p.Omega_hover_rear,  p.Omega_hover_rear*60/(2*pi));
fprintf('Total: %.1f N  |  Weight: %.1f N  |  T/W: %.3f\n', ...
        T_total_check, p.W, T_total_check/p.W);
fprintf('\n--- Tilt Convention (Rev 4) ---\n');
fprintf('Front (tractor, 0=up):    rest=%.1f deg  range=[%.1f, %.1f] deg\n', ...
        rad2deg(p.beta_front_rest), rad2deg(p.beta_front_min), rad2deg(p.beta_front_max));
fprintf('Rear  (pusher, 0=down):   rest=%.1f deg  range=[%.1f, %.1f] deg\n', ...
        rad2deg(p.beta_rear_rest), rad2deg(p.beta_rear_min), rad2deg(p.beta_rear_max));
fprintf('Front perturb (hover->cruise): %.1f -> %.1f deg\n', ...
        p.tilt_limits.front_perturb_deg_bp(1), p.tilt_limits.front_perturb_deg_bp(2));
fprintf('Rear  perturb (hover->cruise): %.1f -> %.1f deg\n', ...
        p.tilt_limits.rear_perturb_deg_bp(1), p.tilt_limits.rear_perturb_deg_bp(2));
fprintf('\n--- Transition ---\n');
fprintf('Blend lo:              %.1f m/s\n', p.V_blend_lo);
fprintf('Blend hi:              %.1f m/s  (%.0f%% above stall)\n', ...
        p.V_blend_hi, (p.V_blend_hi/p.V_stall_actual - 1)*100);
fprintf('\n--- Motor Arms ---\n');
fprintf('Front: x=%.2f m, y=±%.3f m  (%.0f%% semi-span)\n', ...
        p.x_front, abs(p.r_motor(1,2)), p.y_front_frac*100);
fprintf('Rear:  x=%.2f m, y=±%.3f m  (%.0f%% semi-span)\n', ...
        p.x_rear,  abs(p.r_motor(3,2)), p.y_rear_frac*100);
fprintf('=============================================\n\n');

%% =========================================================================
%  19. STATE VECTOR DEFINITION
%  Order here must match the Mux block in Simulink exactly.
%  This vector serves two purposes:
%    (a) defines the interface contract between all blocks
%    (b) provides initial conditions for the 6DOF block
%% =========================================================================

% --- Hover initial conditions ---
ic.V_air          = 0;
ic.alpha          = 0;
ic.beta_side      = 0;
ic.p_rate         = 0;
ic.q_rate         = 0;
ic.r_rate         = 0;
ic.phi            = 0;
ic.theta          = 0;
ic.psi            = 0;
ic.pos_N          = 0;
ic.pos_E          = 0;
ic.pos_D          = -10;     % 10 m altitude, NED so negative
ic.vel_N          = 0;
ic.vel_E          = 0;
ic.vel_D          = 0;

% REV4: all four motors have tilt feedback now (was only front)
ic.beta_tilt_FL   = p.beta_front_rest;
ic.beta_tilt_FR   = p.beta_front_rest;
ic.beta_tilt_RL   = p.beta_rear_rest;
ic.beta_tilt_RR   = p.beta_rear_rest;

ic.Omega_FL       = p.Omega_hover_front;
ic.Omega_FR       = p.Omega_hover_front;
ic.Omega_RL       = p.Omega_hover_rear;
ic.Omega_RR       = p.Omega_hover_rear;

% --- Pack into flat vector (order is the contract) ---
%  [1]  V_air        m/s
%  [2]  alpha        rad
%  [3]  beta_side    rad
%  [4]  p_rate       rad/s
%  [5]  q_rate       rad/s
%  [6]  r_rate       rad/s
%  [7]  phi          rad
%  [8]  theta        rad
%  [9]  psi          rad
%  [10] pos_N        m
%  [11] pos_E        m
%  [12] pos_D        m
%  [13] vel_N        m/s
%  [14] vel_E        m/s
%  [15] vel_D        m/s
%  [16] beta_tilt_FL rad   (REV4: front-left, was already present)
%  [17] beta_tilt_FR rad   (REV4: front-right, was already present)
%  [18] beta_tilt_RL rad   (REV4: NEW - rear-left now tilts)
%  [19] beta_tilt_RR rad   (REV4: NEW - rear-right now tilts)
%  [20] Omega_FL     rad/s (REV4: index shifted from 18 -> 20)
%  [21] Omega_FR     rad/s (REV4: index shifted from 19 -> 21)
%  [22] Omega_RL     rad/s (REV4: index shifted from 20 -> 22)
%  [23] Omega_RR     rad/s (REV4: index shifted from 21 -> 23)
%
%  NOTE: vector length changed from 21 to 23 elements due to the two
%  additional rear tilt feedback channels. Any existing Mux/Demux block
%  in Simulink with hardcoded port counts must be updated to 23.

p.state0_vec = [
    ic.V_air;
    ic.alpha;
    ic.beta_side;
    ic.p_rate;
    ic.q_rate;
    ic.r_rate;
    ic.phi;
    ic.theta;
    ic.psi;
    ic.pos_N;
    ic.pos_E;
    ic.pos_D;
    ic.vel_N;
    ic.vel_E;
    ic.vel_D;
    ic.beta_tilt_FL;
    ic.beta_tilt_FR;
    ic.beta_tilt_RL;
    ic.beta_tilt_RR;
    ic.Omega_FL;
    ic.Omega_FR;
    ic.Omega_RL;
    ic.Omega_RR
];

% --- 6DOF block needs these separately ---
p.ic_pos    = [ic.pos_N; ic.pos_E; ic.pos_D];
p.ic_vel    = [ic.V_air; 0; 0];
p.ic_euler  = [ic.phi; ic.theta; ic.psi];
p.ic_rates  = [ic.p_rate; ic.q_rate; ic.r_rate];

%% =========================================================================
%  20. REFERENCE / DESIRED STATE
%% =========================================================================
p.ref.pos_N   = 0;      % m
p.ref.pos_E   = 0;      % m
p.ref.pos_D   = -10;    % m   (10m altitude)
p.ref.psi     = 0;      % rad (heading)
p.ref.V_cmd   = 0;      % m/s (airspeed command, 0 = hover)

% [1] pos_N  [2] pos_E  [3] pos_D  [4] psi_cmd  [5] V_cmd
p.ref_vec = [p.ref.pos_N; p.ref.pos_E; p.ref.pos_D;
             p.ref.psi;   p.ref.V_cmd];

fprintf('State vector defined: %d elements\n', length(p.state0_vec));
fprintf('Initial altitude: %.1f m\n', -ic.pos_D);
fprintf('Rear motors now have independent tilt - state vector grew from 21 to 23 elements.\n\n');
