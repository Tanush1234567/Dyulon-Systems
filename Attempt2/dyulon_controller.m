function [Omega_cmd, beta_cmd, delta_ail_cmd, ctrl_state_next] = dyulon_controller(...
    rate_cmd, rate_meas, thrust_cmd, V_air, ctrl_state, dt, p) %#codegen
%DYULON_CONTROLLER  Rate-loop attitude controller + control allocation.
%  Rev 4, v1 scope: RATE CONTROL ONLY (roll/pitch/yaw rate + collective
%  thrust). No angle hold, no position hold -- those are outer loops that
%  wrap around this, not yet built. This is the innermost loop, matching
%  what a real flight controller runs in acro/rate mode.
%
%  CODEGEN-COMPATIBLE BY CONSTRUCTION:
%    - Fixed-size arrays only (4x1 throughout), no cell arrays, no
%      dynamic-size anything, no try/catch, no string formatting.
%    - All state explicit in (ctrl_state) / out (ctrl_state_next) --
%      same f(state, cmd, dt, params) -> (state_next, output) shape as
%      dyulon_actuator_dynamics.m, for the same hardware-porting reason.
%    - #codegen pragma at the top so this can be dropped into a MATLAB
%      Function block and Simulink/Embedded Coder can generate C from it
%      directly, or ported by hand into a PX4/custom module with the same
%      function signature.
%
%  CONTROL ALLOCATION STRATEGY (fixed policy, not an optimised solve --
%  deterministic and debuggable on embedded, per the earlier design
%  decision to avoid a per-timestep pseudo-inverse solve):
%    Roll  (p) : differential Omega, FRONT pair only (FL vs FR).
%                Front = vertical-heavy = most roll-efficient near hover
%                tilt, per the role split in dyulon_init.m Sec 6.
%    Pitch (q) : differential Omega, FRONT pair vs REAR pair (mean-based),
%                using both moment arms -- rear's long arm gives it real
%                pitch authority even at partial (cruise-nominal) thrust.
%    Yaw   (r) : differential TILT ANGLE (beta), either pair -- NOT
%                differential Omega. This is the REV 4 authority Rev 3
%                did not have (Rev 3's rear motors were fixed-mount).
%                Reaction-torque yaw (via Omega) is ALSO applied as a
%                secondary contribution since it's small but free.
%    Collective: mean Omega across all 4 motors, split front/rear by the
%                same p.frac_front/frac_rear hover thrust ratio used to
%                size the motors in dyulon_init.m, so nominal (zero-rate-
%                error) commands reproduce the designed hover trim.
%    Tilt schedule (nominal beta before control trim is added): blended
%    by airspeed exactly like the aero blend schedule in dyulon_init.m
%    Sec 11, using the SAME p.V_blend_lo/hi breakpoints so the tilt
%    schedule and the aero-blend the physics model already assumes are
%    the same schedule, not two schedules that could disagree.
%    Aileron: NOT part of this v1 -- zero-commanded until an outer loop
%    or cruise-mode aileron trim is added. Wing lift-dependent, so it is
%    intentionally left at 0 while V_air is near hover regime (no dynamic
%    pressure to authority on anyway).
%
%  CONTROLLER STATE LAYOUT (ctrl_state, struct):
%    ctrl_state.I_roll   rad     roll  rate-error integral
%    ctrl_state.I_pitch  rad     pitch rate-error integral
%    ctrl_state.I_yaw    rad     yaw   rate-error integral
%
%  INPUTS:
%    rate_cmd(3x1)   [p_cmd; q_cmd; r_cmd]     rad/s   commanded body rates
%    rate_meas(3x1)  [p; q; r]                  rad/s   measured body rates
%    thrust_cmd      scalar, 0..1 normalised    -       commanded collective
%                     (0 = min keep-alive thrust, 1 = full authority)
%    V_air           scalar                    m/s     measured airspeed,
%                     drives the tilt-schedule blend (same schedule the
%                     aero model already blends on)
%    ctrl_state              integrator state (see above)
%    dt              scalar                    s       controller time step
%    p                       parameter struct from dyulon_init.m
%
%  OUTPUTS:
%    Omega_cmd(4x1)   rad/s   commanded motor speed, order [FL,FR,RL,RR],
%                              PRE-actuator-dynamics (feed into
%                              dyulon_actuator_dynamics.m as Omega_cmd)
%    beta_cmd(4x1)    rad     commanded tilt angle, same order/convention
%                              (feed into dyulon_actuator_dynamics.m)
%    delta_ail_cmd    rad     commanded aileron deflection (v1: always 0)
%    ctrl_state_next          updated integrator state for next call

%% --- Unpack ---
p_cmd = rate_cmd(1); q_cmd = rate_cmd(2); r_cmd = rate_cmd(3);
p_meas = rate_meas(1); q_meas = rate_meas(2); r_meas = rate_meas(3);

e_roll  = p_cmd - p_meas;
e_pitch = q_cmd - q_meas;
e_yaw   = r_cmd - r_meas;

%% --- PI rate loops (roll, pitch via differential Omega; yaw via ---
%% --- differential tilt, see allocation notes above) ---
I_roll_next  = ctrl_state.I_roll  + dt * e_roll;
I_pitch_next = ctrl_state.I_pitch + dt * e_pitch;
I_yaw_next   = ctrl_state.I_yaw   + dt * e_yaw;

% Anti-windup: clamp integrator magnitude. Limits are placeholders (v1) --
% size these against actual actuator saturation once bench-tested.
I_clamp = 5.0;
I_roll_next  = min(max(I_roll_next,  -I_clamp), I_clamp);
I_pitch_next = min(max(I_pitch_next, -I_clamp), I_clamp);
I_yaw_next   = min(max(I_yaw_next,   -I_clamp), I_clamp);

domega_roll  = p.Kp_roll_rate  * e_roll  + p.Ki_roll_rate  * I_roll_next;
domega_pitch = p.Kp_pitch_rate * e_pitch + p.Ki_pitch_rate * I_pitch_next;

% Yaw uses differential TILT, not differential Omega (REV 4 authority).
% Gain derivation: this is NOT part of the linear Kp_yaw_rate derivation
% in dyulon_init.m (that was reaction-torque-only) -- this is a SEPARATE,
% currently-placeholder gain for the nonlinear tilt-differential effect.
% FLAG: p.Kp_yaw_tilt is a rough placeholder, not derived from a linearised
% tilt-yaw sensitivity the way the Omega-based gains were. Tune this against
% simulated or bench yaw-rate step response before trusting it.
Kp_yaw_tilt = 0.5;   % rad tilt-differential per rad/s yaw rate error, PLACEHOLDER
dbeta_yaw   = Kp_yaw_tilt * e_yaw + (p.Ki_yaw_rate/p.Kp_yaw_rate) * Kp_yaw_tilt * I_yaw_next;

% Small secondary reaction-torque yaw contribution via differential Omega
% (uses the linearised Kp_yaw_rate/Ki_yaw_rate gains derived in
% dyulon_init.m Sec 8b -- kept small/secondary since that derivation
% showed reaction-torque yaw authority is weak).
%
% PAIRING NOTE: p.spin_dir = [FL:-1, FR:+1, RL:+1, RR:-1] (counter-
% rotating pairs, set in dyulon_init.m Sec 5). Reaction torque from
% dyulon_forces_moments.m scales as -spin_dir(i)*Omega_i^2 about the
% thrust axis. Speeding up FL+RL together while slowing FR+RR (i.e.
% front-vs-rear pairing) puts a -1 and a +1 spin-dir motor on the SAME
% side of the differential, so their reaction-torque contributions
% CANCEL (net zero yaw authority -- verified numerically before writing
% this). The pairing that actually reinforces net yaw reaction torque is
% by MATCHING spin direction: FL(-1) + RR(-1) vs FR(+1) + RL(+1).
domega_yaw_reaction = p.Kp_yaw_rate * e_yaw + p.Ki_yaw_rate * I_yaw_next;

%% --- Collective (heave) ---
% thrust_cmd in [0,1] maps onto the hover-trim Omega split already sized
% in dyulon_init.m (p.Omega_hover_front/rear), scaled by thrust_cmd/nominal.
% v1: simple linear scaling around hover trim, not a true thrust-to-Omega
% inverse (that would need q_bar-dependent KpT, adding V_air-dependence
% here -- deferred to keep v1 minimal per "no fancy simulations" scope).
Omega_base_front = p.Omega_hover_front * (0.5 + thrust_cmd);
Omega_base_rear  = p.Omega_hover_rear  * (0.5 + thrust_cmd);

%% --- Tilt nominal schedule (airspeed-blended, same breakpoints as the ---
%% --- aero blend so the two schedules cannot disagree) ---
blend = (V_air - p.V_blend_lo) / (p.V_blend_hi - p.V_blend_lo);
blend = min(max(blend, 0.0), 1.0);   % 0 = hover, 1 = cruise

beta_nom_front_sched = p.beta_nom_front + (0 - p.beta_nom_front) * blend;
beta_nom_rear_sched  = p.beta_nom_rear  + (0 - p.beta_nom_rear)  * blend;

%% --- Allocation: combine base + control trim per motor ---
% Order: [FL, FR, RL, RR]
Omega_cmd = zeros(4,1);
beta_cmd  = zeros(4,1);

% Roll: differential Omega, FRONT pair only. FL gets +, FR gets - (sign
% convention: positive roll rate = right wing down = FL speeds up).
% Yaw reaction-torque pairing is FL+RR vs FR+RL (matching spin direction,
% see note above) -- NOT front-vs-rear, which cancels to zero.
Omega_cmd(1) = Omega_base_front + domega_roll/2 + domega_pitch/2 + domega_yaw_reaction/4;  % FL
Omega_cmd(2) = Omega_base_front - domega_roll/2 + domega_pitch/2 - domega_yaw_reaction/4;  % FR
% Rear pair: pitch trim only (no roll authority assigned to rear per the
% fixed allocation policy), plus its share of yaw reaction torque, paired
% by spin direction (RR matches FL's -1, RL matches FR's +1).
Omega_cmd(3) = Omega_base_rear                  - domega_pitch/2 - domega_yaw_reaction/4;  % RL (spin +1, pairs with FR)
Omega_cmd(4) = Omega_base_rear                  - domega_pitch/2 + domega_yaw_reaction/4;  % RR (spin -1, pairs with FL)

% Tilt: nominal schedule + yaw differential (front pair carries the yaw
% tilt-differential; rear stays on its nominal schedule in v1 -- adding
% rear-side yaw-tilt authority is a straightforward extension once front-
% only yaw-tilt is validated).
beta_cmd(1) = beta_nom_front_sched + dbeta_yaw/2;
beta_cmd(2) = beta_nom_front_sched - dbeta_yaw/2;
beta_cmd(3) = beta_nom_rear_sched;
beta_cmd(4) = beta_nom_rear_sched;

% Saturate commands to actuator limits (belt-and-braces -- 
% dyulon_actuator_dynamics.m also saturates, but saturating here keeps
% the controller's own integrator anti-windup logic consistent with what
% will actually reach the plant).
Omega_cmd(1:2) = min(max(Omega_cmd(1:2), p.Omega_min(1)), p.Omega_max_front);
Omega_cmd(3:4) = min(max(Omega_cmd(3:4), p.Omega_min(3)), p.Omega_max_rear);
beta_cmd(1:2)  = min(max(beta_cmd(1:2),  p.beta_min_front), p.beta_max_front);
beta_cmd(3:4)  = min(max(beta_cmd(3:4),  p.beta_min_rear),  p.beta_max_rear);

%% --- Aileron: v1 always zero (see allocation note above) ---
delta_ail_cmd = 0.0;

%% --- Pack state ---
ctrl_state_next.I_roll  = I_roll_next;
ctrl_state_next.I_pitch = I_pitch_next;
ctrl_state_next.I_yaw   = I_yaw_next;

end