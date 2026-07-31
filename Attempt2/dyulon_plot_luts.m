%% =========================================================================
%  DYULON SYSTEMS - Parameter Validation and LUT Plots
%  dyulon_plot_luts.m
%
%  Run dyulon_init.m first, then this script.
%  Generates all the key aerodynamic and propulsion LUT plots so you can
%  visually sanity-check the assumed values before building the Simulink model.
%% =========================================================================

dyulon_init;   % load all parameters into p

figure('Name','Dyulon - Vehicle LUTs','NumberTitle','off','Color','white');
set(gcf,'Position',[50 50 1400 900]);

%% ---- 1. CL vs Alpha ----
subplot(3,4,1);
plot(p.aero_alpha_deg, p.aero_CL, 'b-o','LineWidth',2,'MarkerSize',4);
hold on;
xline(p.alpha_bestLD_deg,'g--','LineWidth',1.5);
xline(p.alpha_stall_deg, 'r--','LineWidth',1.5);
xlabel('AoA  [deg]'); ylabel('C_L  [-]');
title('Lift Coefficient vs AoA');
legend('C_L','Best L/D AoA','Stall AoA','Location','northwest');
grid on; ylim([-0.8 1.3]);

%% ---- 2. CD vs Alpha ----
subplot(3,4,2);
plot(p.aero_alpha_deg, p.aero_CD, 'r-o','LineWidth',2,'MarkerSize',4);
xlabel('AoA  [deg]'); ylabel('C_D  [-]');
title('Drag Coefficient vs AoA');
grid on;

%% ---- 3. L/D vs Alpha ----
subplot(3,4,3);
CLCD = p.aero_CL ./ p.aero_CD;
plot(p.aero_alpha_deg, CLCD, 'm-o','LineWidth',2,'MarkerSize',4);
hold on;
xline(p.alpha_bestLD_deg,'g--','LineWidth',1.5);
yline(p.CLCD_max,'g:','LineWidth',1.2);
text(p.alpha_bestLD_deg+0.3, p.CLCD_max-0.5, sprintf('L/D_{max} = %.1f',p.CLCD_max),'Color','g');
xlabel('AoA  [deg]'); ylabel('L/D  [-]');
title('Lift-to-Drag Ratio');
grid on;

%% ---- 4. Cm vs Alpha (pitching moment) ----
subplot(3,4,4);
plot(p.aero_alpha_deg, p.aero_Cm, 'k-o','LineWidth',2,'MarkerSize',4);
yline(0,'k:');
xlabel('AoA  [deg]'); ylabel('C_m  [-]');
title('Pitching Moment Coeff. (stable if slope < 0)');
grid on;

%% ---- 5. Thrust vs RPM (front hover props, V=0) ----
subplot(3,4,5);
RPM_vec  = 1000:200:9000;
Omega_vec = RPM_vec * 2*pi/60;
T_front  = p.KpT_front_0 .* Omega_vec.^2;
T_rear   = p.KpT_rear_0  .* Omega_vec.^2;
plot(RPM_vec, T_front, 'b-','LineWidth',2); hold on;
plot(RPM_vec, T_rear,  'r-','LineWidth',2);
yline(p.mass*p.g/4, 'k--','LineWidth',1.2);   % thrust needed per motor at hover
text(1500, p.mass*p.g/4+0.3, 'Hover per-motor thrust','FontSize',8);
xlabel('Motor speed  [RPM]'); ylabel('Thrust  [N]');
title('Static Thrust vs RPM');
legend('Front (hover) prop','Rear (cruise) prop','Location','northwest');
grid on;

%% ---- 6. KpT vs Airspeed (both prop types) ----
subplot(3,4,6);
V_range = 0:0.5:20;
scale_f = interp1(p.prop_V_bp, p.KpT_front_scale, V_range, 'linear', 'extrap');
scale_r = interp1(p.prop_V_bp, p.KpT_rear_scale,  V_range, 'linear', 'extrap');
KpT_f_V = p.KpT_front_0 * scale_f;
KpT_r_V = p.KpT_rear_0  * scale_r;
plot(V_range, KpT_f_V*1e5, 'b-','LineWidth',2); hold on;
plot(V_range, KpT_r_V*1e5, 'r-','LineWidth',2);
xlabel('Airspeed  [m/s]'); ylabel('KpT  [×10^{-5} N/(rad/s)²]');
title('Thrust Coefficient vs Airspeed');
legend('Front (hover) prop','Rear (cruise) prop','Location','northeast');
grid on;

%% ---- 7. Thrust vs Airspeed (at fixed Omega = 700 rad/s) ----
subplot(3,4,7);
Omega_test = 700;  % rad/s (~6700 RPM)
T_f_V = KpT_f_V * Omega_test^2;
T_r_V = KpT_r_V * Omega_test^2;
plot(V_range, T_f_V, 'b-','LineWidth',2); hold on;
plot(V_range, T_r_V, 'r-','LineWidth',2);
xlabel('Airspeed  [m/s]'); ylabel('Thrust  [N]');
title(sprintf('Thrust vs Airspeed  (\\Omega = %d rad/s ≈ %d RPM)', Omega_test, round(Omega_test*60/(2*pi))));
legend('Front (hover)','Rear (cruise)','Location','northeast');
grid on;

%% ---- 8. Motor Actuator Step Response ----
subplot(3,4,8);
t_sim = 0:0.001:0.3;
sys_motor  = tf(p.motor_bandwidth,  [1, p.motor_bandwidth]);
[y_motor, t_out] = step(sys_motor, t_sim);
plot(t_out*1000, y_motor, 'b-','LineWidth',2); hold on;
yline(0.63,'k:'); yline(0.90,'r:');
text(20, 0.64, '63% (time constant)','FontSize',8);
text(20, 0.91, '90% rise','FontSize',8,'Color','r');
xlabel('Time  [ms]'); ylabel('Normalised Response  [-]');
title(sprintf('Motor Step Response (\\omega_c = %g rad/s)', p.motor_bandwidth));
grid on; ylim([0 1.1]);

%% ---- 9. Tilt Actuator Step Response (REV 4: direct-drive gearmotor, ----
%% ---- shared model for front AND rear -- BUG FIXED: was p.servo_damping ----
%% ---- (undefined field in Rev 3), now p.tilt_zeta/p.tilt_omega_n) ----
subplot(3,4,9);
sys_tilt = tf(p.tilt_omega_n^2, [1, 2*p.tilt_zeta*p.tilt_omega_n, p.tilt_omega_n^2]);
[y_tilt, t_out] = step(sys_tilt, t_sim);
plot(t_out*1000, y_tilt, 'g-','LineWidth',2);
yline(0.63,'k:'); yline(1.02,'r:');
xlabel('Time  [ms]'); ylabel('Normalised Response  [-]');
title(sprintf('Tilt Actuator Step Response (\\omega_n = %g, \\zeta = %.1f)', p.tilt_omega_n, p.tilt_zeta));
grid on; ylim([0 1.15]);

%% ---- 10. Transition Blending vs Airspeed (REV 4: illustrative only -- ----
%% ---- actual per-motor tilt allocation is decided by dyulon_controller.m, ----
%% ---- not a single blend*beta formula. This shows the front nominal ----
%% ---- tilt sweeping hover->cruise per the role split in Sec 6.) ----
subplot(3,4,10);
V_blend = linspace(0, 20, 200);
blend_val = min(1, max(0, (V_blend - p.V_blend_lo)/(p.V_blend_hi - p.V_blend_lo)));
beta_front_deg = rad2deg(p.beta_nom_front + (0 - p.beta_nom_front) * blend_val);
yyaxis left;
plot(V_blend, blend_val, 'b-','LineWidth',2);
ylabel('Blend factor  [0=hover, 1=cruise]');
yyaxis right;
plot(V_blend, beta_front_deg, 'r--','LineWidth',2);
ylabel('Front nominal tilt angle  [deg]');
xlabel('Airspeed  [m/s]');
title('Illustrative Front Tilt Envelope (see dyulon\_controller.m for actual law)');
xline(p.V_blend_lo,'k:'); xline(p.V_blend_hi,'k:');
legend('Blend','Tilt angle','Location','east');
grid on;

%% ---- 11. Wing Lift at Cruise Speed ----
subplot(3,4,11);
V_range2 = 0:0.5:25;
% At best-L/D AoA
Lift_vec = 0.5 * p.rho * V_range2.^2 * p.S * p.CL_bestLD;
Weight_line = p.mass * p.g * ones(size(V_range2));
plot(V_range2, Lift_vec,    'b-','LineWidth',2); hold on;
plot(V_range2, Weight_line, 'k--','LineWidth',1.5);
[~, idx_cross] = min(abs(Lift_vec - Weight_line));
xline(V_range2(idx_cross), 'g--','LineWidth',1.5);
text(V_range2(idx_cross)+0.3, p.mass*p.g*0.6, ...
    sprintf('V_{cruise} ≈ %.1f m/s', p_cross = V_range2(idx_cross)),'Color','g');
xlabel('Airspeed  [m/s]'); ylabel('Force  [N]');
title(sprintf('Wing Lift at \\alpha_{bestLD} = %.1f°  vs Weight', p.alpha_bestLD_deg));
legend('Wing Lift','Vehicle Weight','Cruise speed','Location','northwest');
grid on;

%% ---- 12. Polar (CL vs CD) ----
subplot(3,4,12);
plot(p.aero_CD, p.aero_CL, 'b-o','LineWidth',2,'MarkerSize',4);
hold on;
% Best L/D tangent line from origin
CD_line = linspace(0, max(p.aero_CD), 100);
CL_line = p.CLCD_max * CD_line;
plot(CD_line, CL_line, 'g--','LineWidth',1.5);
plot(p.CD_bestLD, p.CL_bestLD, 'ro','MarkerSize',8,'MarkerFaceColor','r');
xlabel('C_D  [-]'); ylabel('C_L  [-]');
title(sprintf('Drag Polar  (L/D_{max} = %.1f at \\alpha = %.1f°)', p.CLCD_max, p.alpha_bestLD_deg));
legend('Polar','Best L/D tangent','Best L/D point','Location','northwest');
grid on;

sgtitle('Dyulon Systems - Vehicle LUT Validation Plots', 'FontSize',14,'FontWeight','bold');

fprintf('\nAll LUT plots generated. Replace approximated values with:\n');
fprintf('  - aero_CL/CD/Cm: XFLR5 analysis of your chosen airfoil\n');
fprintf('  - KpT/KpM LUTs: eCalc prop selection + wind tunnel/bench test\n');
fprintf('  - Inertia: CAD model with component masses\n');
fprintf('  - Motor bandwidth: step-response bench test with ESC\n');
fprintf('  - Servo bandwidth: step-response bench test\n');
