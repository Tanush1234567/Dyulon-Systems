%% =========================================================================
%  DYULON SYSTEMS - Hybrid VTOL Plant Model Initialization
%  dyulon_init.m   |   Rev 3 - fully expression-driven
%
%  DESIGN INTENT:
%  Every derived parameter is computed from primary inputs using explicit
%  expressions. Change a primary input at the top and re-run - all
%  dependent values update automatically.
%
%  PRIMARY INPUTS (the only numbers you should ever change directly):
%    p.mass, p.V_cruise_kmh, p.V_stall_target, p.AR, p.CL_max_assumed,
%    p.TW_ratio, p.frac_front, p.D_front_in, p.D_rear_in,
%    p.Ct_front, p.Cm_front, p.Ct_rear, p.Cm_rear,
%    p.k_Ixx, p.k_Iyy, sensor noise values, actuator dynamics values.
%
%  HOW TO USE:
%    Run dyulon_init.m, then dyulon_plot_luts.m to sanity check.
%    All Simulink blocks read from the 'p' workspace struct.
%  =========================================================================
clear; clc;

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
p.has_tilt   = [ 1, 1, 0,  0];   % front tilt, rear fixed

p.beta_rear_fixed = deg2rad(0);   % rear motors vertical (no lean)

%% =========================================================================
%  6. TILT SERVO  (primary inputs; dynamics from Mancinelli bench data)
%% =========================================================================
p.beta_min       = deg2rad(-100);  % rad  hard stop
p.beta_max       = deg2rad(5);     % rad  hard stop
p.servo_omega_n  = 55;             % rad/s
p.servo_zeta     = 1.5;            % overdamped
p.servo_rate_lim = 11.0;           % rad/s  max tilt rate

%% =========================================================================
%  7. PROPELLER COEFFICIENTS  (derived from Ct/Cm/D)
%% =========================================================================
D_front = p.D_front_in * 0.0254;   % m  convert inches to metres
D_rear  = p.D_rear_in  * 0.0254;   % m

% KpT = Ct * rho * D^4 / (4*pi²)    [N / (rad/s)²]
% KpM = Cm * rho * D^5 / (4*pi²)    [N·m / (rad/s)²]
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
p.Omega_max_front = 1.5 * p.Omega_hover_front;
p.Omega_max_rear  = 1.5 * p.Omega_hover_rear;

p.Omega_min = [  80,   80,   80,   80];                          % rad/s keep-alive
p.Omega_max = [p.Omega_max_front, p.Omega_max_front, ...
               p.Omega_max_rear,  p.Omega_max_rear];             % rad/s

p.motor_bandwidth = 25;   % rad/s  first-order motor model corner frequency

%% =========================================================================
%  9. PROPELLER AIRSPEED LUTs  (scale factors on KpT_0 and KpM_0)
%% =========================================================================
% Field name used consistently everywhere: p.prop_V_bp
% Front props lose thrust faster with airspeed (large hover-optimised diameter)
% Rear props retain thrust better (smaller cruise-optimised diameter)
%
% Linear decrease approx: scale ≈ 1 - k_v * V
%   Front: k_v ≈ 0.022 (matches ~30% retention at 35 m/s)
%   Rear:  k_v ≈ 0.013 (matches ~55% retention at 35 m/s)
% Breakpoints cover 0 to cruise + 10%:

p.prop_V_bp = [0,    5,    10,   15,   20,   25,   35  ];  % m/s

p.KpT_front_scale = [1.00, 0.89, 0.78, 0.67, 0.56, 0.46, 0.30];
p.KpT_rear_scale  = [1.00, 0.94, 0.88, 0.82, 0.75, 0.68, 0.55];
p.KpM_front_scale = p.KpT_front_scale;
p.KpM_rear_scale  = p.KpT_rear_scale;

%% =========================================================================
%  10. WING AERODYNAMICS LUTs
%% =========================================================================
% Reflex airfoil class (TL54 / MH60 / AG35).  Re ≈ 500k-1M.
% Replace with XFLR5 output for chosen airfoil.

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
p.Cn_b =  0.005;  % weathercock (weak - no vertical fin)

%% =========================================================================
%  11. TRANSITION BLEND SCHEDULE  (derived from stall speed)
%% =========================================================================
% Blend starts when wing contributes meaningfully (roughly q*S*CL ≈ 15-20% W)
% Blend ends at 10% above stall to leave motor backup margin
p.V_blend_lo = 10.0;                         % m/s  (fixed: wing starts ~18% lift here)
p.V_blend_hi = p.V_stall_target * 1.10;      % m/s  derived: 10% above target stall

%% =========================================================================
%  12. SENSOR NOISE AND DELAYS
%% =========================================================================
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
CLCD_vec             = p.aero_CL ./ p.aero_CD;
[~, idx_bestLD]      = max(CLCD_vec);
p.alpha_bestLD_deg   = p.aero_alpha_deg(idx_bestLD);
p.CL_bestLD          = p.aero_CL(idx_bestLD);
p.CD_bestLD          = p.aero_CD(idx_bestLD);
p.CLCD_max           = CLCD_vec(idx_bestLD);

[p.CL_max, idx_stall] = max(p.aero_CL);
p.alpha_stall_deg     = p.aero_alpha_deg(idx_stall);

p.V_stall_actual = sqrt(2 * p.W / (p.rho * p.S * p.CL_max));
p.CL_at_cruise   = p.W / (0.5 * p.rho * p.V_cruise^2 * p.S);

%% =========================================================================
%  14. SUMMARY PRINT
%% =========================================================================
fprintf('\n=============================================\n');
fprintf(' DYULON SYSTEMS - Model Initialised (Rev 3)\n');
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
