function [F_body, M_body, aux] = dyulon_forces_moments(state, actuators, p)
%DYULON_FORCES_MOMENTS  Compute total forces and moments on the Dyulon airframe.
%  Rev 3 - field name bug fixed (prop_V_bp), expressions consistent with init Rev 3.
%
%  INPUTS:
%    state.V_air, .alpha, .beta_side, .p_rate, .q_rate, .r_rate, .phi, .theta, .psi
%    actuators.Omega(4), .beta(4), .delta_ail
%    p  - parameter struct from dyulon_init.m

V      = state.V_air;
alpha  = state.alpha;
beta_s = state.beta_side;
p_rate = state.p_rate; %Roll Rate
q_rate = state.q_rate; %Pitch Rate
r_rate = state.r_rate; %Yaw Rate

q_bar = 0.5 * p.rho * max(V, 0.1)^2;

%% A. PROPELLER FORCES AND MOMENTS
F_prop = zeros(3,1);
M_prop = zeros(3,1);
M_gyro = zeros(3,1);

% FIX Rev3: use p.prop_V_bp (was p.prop_V_breakpoints in Rev2 - crash bug)
V_clamped = min(V, p.prop_V_bp(end));

scale_front = interp1(p.prop_V_bp, p.KpT_front_scale, V_clamped, 'linear');
KpT_front   = p.KpT_front_0 * scale_front;
KpM_front   = p.KpM_front_0 * scale_front;

scale_rear  = interp1(p.prop_V_bp, p.KpT_rear_scale,  V_clamped, 'linear');
KpT_rear    = p.KpT_rear_0  * scale_rear;
KpM_rear    = p.KpM_rear_0  * scale_rear;

I_rotor = 1.5e-4;   % kg·m²  approximate per rotor (spinner + blades)
omega_body = [p_rate; q_rate; r_rate];

for i = 1:4
    Omega_i = actuators.Omega(i);

    if p.motor_type(i) == 1
        KpT_i = KpT_front;  KpM_i = KpM_front;
    else
        KpT_i = KpT_rear;   KpM_i = KpM_rear;
    end

    T_i = KpT_i * Omega_i^2;
    Q_i = KpM_i * Omega_i^2;

    if p.has_tilt(i)
        beta_i = actuators.beta(i);
    else
        beta_i = p.beta_rear_fixed;
    end

    % Thrust vector: beta=0 → [0;0;-T] (up), beta=-pi/2 → [T;0;0] (fwd)
    T_vec = T_i * [-sin(beta_i); 0; -cos(beta_i)];

    % Reaction torque along thrust axis
    Q_vec = -p.spin_dir(i) * Q_i * [-sin(beta_i); 0; -cos(beta_i)];

    r_i = p.r_motor(i,:)';
    F_prop = F_prop + T_vec;
    M_prop = M_prop + cross(r_i, T_vec) + Q_vec;

    % Gyroscopic moment
    spin_axis = [-sin(beta_i); 0; -cos(beta_i)];
    H_rotor   = p.spin_dir(i) * I_rotor * Omega_i * spin_axis;
    M_gyro    = M_gyro + cross(omega_body, H_rotor);
end

%% B. WING AERODYNAMICS
if V > 2.0
    alpha_deg = rad2deg(alpha);
    alpha_clamped = max(p.aero_alpha_deg(1), min(p.aero_alpha_deg(end), alpha_deg));

    CL = interp1(p.aero_alpha_deg, p.aero_CL, alpha_clamped, 'linear');
    CD = interp1(p.aero_alpha_deg, p.aero_CD, alpha_clamped, 'linear');
    Cm = interp1(p.aero_alpha_deg, p.aero_Cm, alpha_clamped, 'linear');

    Lift = q_bar * p.S * CL;
    Drag = q_bar * p.S * CD;

    Fx_aero =  Lift * sin(alpha) - Drag * cos(alpha);
    Fz_aero = -Lift * cos(alpha) - Drag * sin(alpha);
    Fy_aero =  q_bar * p.S * (-0.15) * beta_s;   % side force, no fin % CONFIRM THE VALUE OF -0.15. It is a coefficient for C_y which is approximately
                                                    % Cy * beta for low
                                                    % beta and Cy is a const

    F_aero = [Fx_aero; Fy_aero; Fz_aero];

    Cm_total = Cm + p.Cm_q * (q_rate * p.cbar / (2 * max(V,0.1)));
    My_aero  = q_bar * p.S * p.cbar * Cm_total;

    Cl_total = p.Cl_b * beta_s ...
             + p.Cl_p * (p_rate * p.b / (2 * max(V,0.1))) ...
             + p.Cl_da * actuators.delta_ail;
    Mx_aero  = q_bar * p.S * p.b * Cl_total;

    Cn_total = p.Cn_b * beta_s + p.Cn_r * (r_rate * p.b / (2 * max(V,0.1)));
    Mz_aero  = q_bar * p.S * p.b * Cn_total;

    M_aero = [Mx_aero; My_aero; Mz_aero];
else
    F_aero = zeros(3,1);  M_aero = zeros(3,1);
    CL = 0; CD = 0; Cm = 0; Lift = 0; Drag = 0;
end

% Gravity handled by Aerospace Blockset 6DOF block - do not add here
F_body = F_prop + F_aero;
M_body = M_prop + M_aero + M_gyro;

aux.F_prop = F_prop; aux.F_aero = F_aero;
aux.M_prop = M_prop; aux.M_aero = M_aero; aux.M_gyro = M_gyro;
aux.KpT_front = KpT_front; aux.KpT_rear = KpT_rear;
aux.q_bar = q_bar; aux.CL = CL; aux.CD = CD; aux.Lift = Lift; aux.Drag = Drag;
end
