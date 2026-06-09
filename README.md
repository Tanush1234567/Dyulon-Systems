# Dyulon Systems — Full Technical Context Document

### For AI assistants continuing this project

**Version:** Rev 3 | **Last updated:** June 2026  
**Team:** Tanush Gupta, Jishnu Mehra, Divye Yadav — TU Delft Impact Contest (Ideation category)

---

## 1. What Dyulon Is

Dyulon Systems is building a **hybrid VTOL (Vertical Take-Off and Landing) delivery UAV** that combines a multirotor's ability to take off and land vertically with a fixed-wing aircraft's efficiency in cruise. The core insight driving the design is simple: conventional drones use rotors for both lift and forward thrust simultaneously, which is aerodynamically inefficient. By separating these functions — rotors for vertical flight, a fixed wing for cruise lift — we can achieve roughly 50–70% better energy efficiency and potentially triple the range compared to a pure multirotor of the same mass.

**Target application:** Long-range autonomous delivery of time-sensitive goods (medicine, documents, emergency supplies) to locations that are difficult to reach by road — islands, remote villages, disaster zones.

**Key performance targets (from spec sheet):**

- MTOW: 35 kg (25 kg operating + 10 kg payload)
- Cruise speed: 125 km/h (34.72 m/s)
- Range: 100 km one-way
- Operating altitude: 100 m AGL
- Rate of climb: 2 m/s

---

## 2. Vehicle Architecture

### 2.1 Airframe Concept

The vehicle is a **flying wing** — no fuselage, no tail, no vertical fin. The payload and avionics sit in the wing centre-section. This was chosen because:

- Flying wings have the best lift-to-drag ratio for a given structural mass (no tail drag, no fuselage drag)
- A reflex airfoil (slightly S-shaped camber line) provides pitch stability without a tail by generating a self-correcting pitching moment
- The configuration is compact for a given wing area

The wing uses a **reflex airfoil** (TL54/MH60/AG35 class). The key property of reflex airfoils is that their pitching moment coefficient (Cm) is slightly negative and becomes more negative with increasing angle of attack — this creates a natural nose-down restoring force, making the aircraft statically stable in pitch without any tail surface.

**Known weakness:** Flying wings have very weak yaw stability. Without a vertical fin, the weathercock stability derivative Cn_β ≈ 0.005/rad (near zero). Yaw must be actively controlled via differential motor torque. This is a significant design constraint.

### 2.2 Propulsion Layout — 4 Motors

```
        FRONT
    [FL]       [FR]
     ↑  WING   ↑
    22"        22"
    TILT       TILT
    hover      hover
    props      props
         ___
        |PAY|
        |LOD|
         ---
    [RL]       [RR]
    18"        18"
    FIXED      FIXED
    cruise     cruise
    props      props
        REAR
```

**Motor 1 — Front-Left (FL):** 22-inch hover-optimised prop. Has longitudinal tilt servo. In hover, points straight up. In cruise, tilts to point fully forward (-90°). Carries 35% of hover lift.

**Motor 2 — Front-Right (FR):** Mirror of FL. Same tilt servo. 35% of hover lift.

**Motor 3 — Rear-Left (RL):** 18-inch cruise-optimised prop. Fixed mount (no tilt servo), permanently vertical (0° lean). Carries 15% of hover lift.

**Motor 4 — Rear-Right (RR):** Mirror of RL. 15% of hover lift.

**Why different prop sizes?**  
A propeller's efficiency is governed by its advance ratio J = V/(n·D). Hover props (22 inch, high pitch) are optimised for J ≈ 0 — maximum static thrust per watt. At cruise speeds (35 m/s) they become very inefficient, losing ~70% of their static thrust coefficient by 35 m/s. Cruise props (18 inch, lower pitch) retain ~55% of their static KpT at 35 m/s, making them suitable for providing forward thrust in cruise while also assisting hover.

**Why rear motors further from CG?**  
The rear motors are positioned 0.80 m aft of CG vs 0.60 m forward for the front. This longer moment arm gives the lower-powered rear motors (30% of hover lift) useful pitch control authority through differential thrust. Even at reduced RPM, their torque generates meaningful pitch moments due to the longer arm.

**Tilt mechanism (front motors only):**  
A longitudinal tilt servo rotates each front motor from vertical (β = 0°, hover) to horizontal (β = -90°, cruise). The servo is second-order with natural frequency 55 rad/s, damping ratio 1.5 (overdamped), and a rate limit of 11 rad/s (~630°/s). Physical stops at -100° and +5°.

**No lateral tilt.** Unlike the Mancinelli dissertation reference design (dual-axis tilting), Dyulon's front motors only tilt longitudinally (elevation only, no azimuth). This was a deliberate simplification that eliminates the gimbal-lock problem that caused most of the control complexity in Mancinelli's design.

### 2.3 Wing Geometry (all derived, current values)

|Parameter|Value|How derived|
|---|---|---|
|Wing area S|1.335 m²|S = 2mg / (ρ · V_stall² · CL_max)|
|Wingspan b|3.164 m|b = √(AR · S)|
|Mean chord|0.422 m|c̄ = S/b|
|Aspect ratio AR|7.5|Primary input (efficiency vs. structure trade)|
|Stall speed|20.0 m/s (72 km/h)|Target; gives 1.74× cruise/stall margin|
|Cruise/stall margin|1.74×|V_cruise / V_stall = 34.72 / 20.0|
|CL at cruise|0.348|Well below stall — low induced drag|
|Best L/D|~37.5|At ~6° AoA from LUT|

### 2.4 Motor Positions (body frame: x-fwd, y-right, z-down)

|Motor|x (m)|y (m)|z (m)|Type|Tilts?|
|---|---|---|---|---|---|
|Front-Left|+0.60|-0.444|+0.03|Hover (22")|Yes|
|Front-Right|+0.60|+0.444|+0.03|Hover (22")|Yes|
|Rear-Left|-0.80|-0.285|+0.03|Cruise (18")|No|
|Rear-Right|-0.80|+0.285|+0.03|Cruise (18")|No|

Spin directions (view from above): FL=CCW, FR=CW, RL=CW, RR=CCW (counter-rotating pairs cancel net gyroscopic torque).

### 2.5 Inertia Estimates (derived from mass × wingspan²)

|Parameter|Value|Expression|
|---|---|---|
|Ixx (roll)|10.51 kg·m²|0.030 · m · b²|
|Iyy (pitch)|6.31 kg·m²|0.018 · m · b²|
|Izz (yaw)|16.82 kg·m²|Ixx + Iyy|

These are rough estimates from empirical scaling. Replace with CAD-derived values when component layout is finalised.

---

## 3. Propulsion Parameters

### 3.1 Thrust Model

```
T_i = KpT_i(V) · Ω_i²     [N]
Q_i = KpM_i(V) · Ω_i²     [N·m]

KpT = Ct · ρ · D⁴ / (4π²)
KpM = Cm · ρ · D⁵ / (4π²)
```

### 3.2 Static Coefficients

||Front (22")|Rear (18")|
|---|---|---|
|Diameter D|0.559 m|0.457 m|
|Ct (static)|0.105|0.092|
|Cm (static)|0.0065|0.0055|
|KpT₀|3.18×10⁻⁴ N/(rad/s)²|1.25×10⁻⁴ N/(rad/s)²|
|KpM₀|1.10×10⁻⁵ N·m/(rad/s)²|3.40×10⁻⁶ N·m/(rad/s)²|

### 3.3 Hover Operating Points (T/W = 1.40)

||Front|Rear|
|---|---|---|
|Thrust target|168.2 N each|72.1 N each|
|Hover Ω|727 rad/s (6944 RPM)|761 rad/s (7267 RPM)|
|Max Ω (1.5× hover)|1091 rad/s|1141 rad/s|

### 3.4 Airspeed Correction LUTs (KpT scales with forward speed)

|V (m/s)|Front scale|Rear scale|
|---|---|---|
|0|1.00|1.00|
|5|0.89|0.94|
|10|0.78|0.88|
|15|0.67|0.82|
|20|0.56|0.75|
|25|0.46|0.68|
|35|0.30|0.55|

Front props lose 70% of static thrust by cruise speed — expected for hover-optimised large prop at high advance ratio. Rear props retain 55% — the efficiency argument of the dual-prop design.

---

## 4. Aerodynamic Model

### 4.1 Lift / Drag / Pitching Moment LUTs

Reflex airfoil class (TL54/MH60/AG35). Re ≈ 500k–1M at cruise. **Replace with XFLR5 output once airfoil is chosen.**

|α (°)|CL|CD|Cm|
|---|---|---|---|
|-8|-0.61|0.058|-0.008|
|-6|-0.40|0.038|-0.010|
|-4|-0.19|0.025|-0.012|
|-2|0.00|0.018|-0.013|
|0|0.20|0.015|-0.014|
|2|0.40|0.016|-0.016|
|4|0.59|0.020|-0.018|
|6|0.76|0.028|-0.020|
|8|0.90|0.042|-0.023|
|10|1.01|0.060|-0.027|
|**12**|**1.07**|**0.086**|**-0.031**|
|14|1.00|0.124|-0.037|
|16|0.80|0.178|-0.040|
|18|0.55|0.245|-0.042|

Zero-lift angle of attack ≈ −2°. Minimum drag at ≈ 2°. Best L/D at ≈ 6°.

### 4.2 Aerodynamic Damping Derivatives

|Derivative|Value|Meaning|
|---|---|---|
|Cm_q|-8.0|Pitch rate damping (per q·c̄/2V)|
|Cl_p|-0.45|Roll rate damping (per p·b/2V)|
|Cn_r|-0.12|Yaw rate damping (per r·b/2V)|
|Cl_β|-0.08|Dihedral (roll from sideslip)|
|Cn_β|+0.005|Weathercock — near neutral, no fin|
|Cl_δa|0.17/rad|Aileron roll effectiveness|

### 4.3 Equations of Motion (Forces)

All forces resolved in body frame (x-forward, y-right, z-down):

**Propeller thrust vector (motor i, longitudinal tilt β_i only):**

```
F_x_i = -T_i · sin(β_i)      % forward when β < 0
F_y_i =  0                    % no lateral tilt
F_z_i = -T_i · cos(β_i)      % upward (negative z)

T_i   = KpT_i(V) · Ω_i²
```

**Reaction torque (along thrust axis):**

```
Q_vec_i = -spin_dir_i · KpM_i · Ω_i² · [-sin(β_i); 0; -cos(β_i)]
```

**Aerodynamic forces (wind-to-body rotation):**

```
Lift = 0.5·ρ·V²·S·CL(α)
Drag = 0.5·ρ·V²·S·CD(α)

Fx_aero =  Lift·sin(α) - Drag·cos(α)
Fz_aero = -Lift·cos(α) - Drag·sin(α)
Fy_aero =  0.5·ρ·V²·S·CY_β·β_sideslip
```

**Moments:**

```
M_pitch = Σ(r_i × F_i)_y + 0.5·ρ·V²·S·c̄·[Cm(α) + Cm_q·(q·c̄/2V)]
M_roll  = Σ(r_i × F_i)_x + 0.5·ρ·V²·S·b·[Cl_β·β + Cl_p·(p·b/2V) + Cl_δa·δ_ail]
M_yaw   = Σ(Q_i)_z        + 0.5·ρ·V²·S·b·[Cn_β·β + Cn_r·(r·b/2V)]
M_gyro  = ω_body × H_rotors    % gyroscopic coupling
```

**6DOF integration:** Implemented using the MATLAB Aerospace Blockset 6DOF (Euler Angles) block, which handles gravity in NED frame internally. Do NOT add gravity to F_body in the forces function.

---

## 5. Controls Architecture

### 5.1 Overview

The control system is a cascaded 3-loop structure with an airspeed-based blending scheme that smoothly transitions between hover and cruise physics. The key philosophy is: **no mode switching**. Instead, a single `blend` scalar (0 = hover, 1 = cruise) continuously reshapes which actuators are responsible for which forces.

```
[Desired Position/Yaw]
         |
         v
  ┌─────────────┐
  │ OUTER LOOP  │  Position → Velocity reference
  │  Kp = 0.5   │  (proportional; velocity integrators in Simulink)
  └──────┬──────┘
         |  vel_ref [3]
         v
  ┌─────────────┐
  │  MID LOOP   │  Velocity error → Desired acceleration [ax, ay, az]
  │  Kp = 1.0   │  PI structure (integrators as Simulink Discrete blocks)
  └──────┬──────┘
         |  [ax_ref, ay_ref, az_ref]  + airspeed V
         v
  ┌──────────────────────────────────────────┐
  │       INNER LOOP — TRANSITION-AWARE      │
  │                                          │
  │  blend = sat((V - 10)/(22 - 10), 0, 1)  │
  │                                          │
  │  Vertical:  F_z_motors = F_z_needed      │
  │             + blend × Lift_available     │
  │                                          │
  │  Tilt:      β_cmd = -90° × blend         │
  │  (0° hover → -90° cruise, continuously) │
  │                                          │
  │  Pitch ref: blend × α_bestLD             │
  │  Roll ref:  bank angle from ay_ref       │
  │                                          │
  │  Motor Ω:  collective + pitch diff       │
  │             + roll diff (signed sqrt)    │
  │  Ailerons: effective at high q_bar       │
  └──────┬───────────────────────────────────┘
         |  [Ω_cmd×4, β_cmd×2, δ_ail]
         v
  ┌─────────────────┐
  │ ACTUATOR MODELS │  Transfer functions + rate limits + saturation
  │  Motor: 1st ord │  Motor ω_c = 25 rad/s
  │  Tilt:  2nd ord │  Servo ω_n = 55 rad/s, ζ = 1.5, rate ≤ 11 rad/s
  │  Aileron: 1st   │  Aileron ω_c = 20 rad/s
  └──────┬──────────┘
         |  [Ω_actual×4, β_actual×2, δ_ail_actual]
         v
  ┌─────────────┐
  │    PLANT    │  dyulon_forces_moments.m → 6DOF (Euler, Aerospace Blockset)
  └──────┬──────┘
         |  True state [pos, vel, att, rates, V_air, α, β]
         v
  ┌─────────────────┐
  │  SENSOR MODELS  │  Band-limited white noise + transport delays
  └──────┬──────────┘
         |  state_est  (feeds back to all loops)
         v
```

### 5.2 Transition Blending Logic

The `blend` factor is the single key variable that drives everything:

```
blend = saturate((V - V_blend_lo) / (V_blend_hi - V_blend_lo), 0, 1)

V_blend_lo = 10.0 m/s   (wing starts contributing ~18% of lift)
V_blend_hi = 22.0 m/s   (10% above stall, transition must complete here)
```

**What blend does:**

|blend = 0 (hover)|blend = 1 (cruise)|
|---|---|
|Front motors point straight up|Front motors point fully forward|
|All vertical force from motors|All vertical force from wing|
|Pitch control: differential Ω (front vs rear)|Pitch control: pitch attitude → AoA → wing lift|
|Roll control: differential L/R motor Ω|Roll control: ailerons|
|Aileron command = 0 (too slow to be effective)|Motor roll differential = 0 (blend suppressed)|

**Vertical force allocation during transition:**

```matlab
Fz_needed    = mass × (az_ref - g)          % total upward force required
Lift_avail   = 0.5·ρ·V²·S·CL(α_current)    % what the wing provides right now
F_z_motors   = Fz_needed + blend × Lift_avail  % motor must cover the rest
```

At V = 16 m/s (blend ≈ 0.5): motors carry ~50% of lift, wing carries ~50%. No discrete handover — purely continuous.

**Tilt angle schedule:**

```matlab
β_cmd = β_min × blend    % β_min = -100° (physical stop, effectively -90° cruise)
```

**Pitch reference during transition:**

```matlab
θ_hover  = 0 rad
θ_cruise = α_bestLD + γ (flight path angle)
θ_ref    = (1-blend)·θ_hover + blend·θ_cruise
```

AoA protection active above 6 m/s: θ clamped so α stays between +2° and +14°.

### 5.3 Motor Command Assembly

Collective command per motor group:

```matlab
F_front_mag = sqrt(Fz_front² + Fx_front²)   % total thrust magnitude
Ω_front_col = sqrt(F_front_mag / KpT_front(V))

F_rear_mag  = sqrt(Fz_rear² + Fx_rear²)
Ω_rear_col  = sqrt(F_rear_mag  / KpT_rear(V))
```

Attitude corrections applied as signed increments (using `signed_sqrt = sign(x)·√|x|` to preserve direction):

```matlab
% Pitch (front up = nose up):
dΩ_pitch_front = signed_sqrt( delta_Fz_pitch/2 / KpT_front)
dΩ_pitch_rear  = signed_sqrt(-delta_Fz_pitch/2 / KpT_rear)

% Roll (right motors up = roll left):
dΩ_roll = signed_sqrt(delta_Fz_roll / KpT_front)

% Final per-motor commands:
Ω_FL = Ω_front_col + dΩ_pitch_front - dΩ_roll
Ω_FR = Ω_front_col + dΩ_pitch_front + dΩ_roll
Ω_RL = Ω_rear_col  + dΩ_pitch_rear  - dΩ_roll
Ω_RR = Ω_rear_col  + dΩ_pitch_rear  + dΩ_roll
```

**Important note on signed_sqrt:** An earlier version of the code used `sqrt(max(x, 0))` which clipped negative corrections to zero, making attitude correction one-directional. This was fixed in Rev 3 — all attitude corrections are now bidirectional.

### 5.4 Attitude Gains (current values, require flight-test tuning)

|Loop|Gain|Value|Units|
|---|---|---|---|
|Position|Kp_pos_xy|0.5|1/s|
|Position|Kp_pos_z|0.6|1/s|
|Velocity|Kp_vel_xy|1.0|(m/s²)/(m/s)|
|Velocity|Kp_vel_z|1.5|(m/s²)/(m/s)|
|Pitch|Kp_theta|8.0|(rad/s²)/rad|
|Pitch|Kd_theta|3.0|(rad/s²)/(rad/s)|
|Roll|Kp_phi|6.0|(rad/s²)/rad|
|Roll|Kd_phi|2.5|(rad/s²)/(rad/s)|
|Yaw|Kp_psi|3.0|(rad/s)/(rad)|

### 5.5 Known Control Weaknesses

1. **Yaw authority is weak.** No rudder, no lateral motor tilt. Yaw control relies entirely on differential motor reaction torques (KpM × ΔΩ² differential). At cruise, motors are lightly loaded, so yaw torque is minimal. Will likely need to accept slow yaw response in cruise or add a small drag rudder.
    
2. **Transition band is tight.** With stall at 20 m/s and cruise at 34.7 m/s, transition must be substantially complete by 22 m/s. At this speed the wing is only generating ~63% of required lift. The remaining motor backup is critical.
    
3. **Propeller-wing interaction not modelled.** At partial tilt, the front motor prop wash hits the wing, changing its effective AoA and lift. This is flagged in Mancinelli's dissertation as important but neglected in the Simulink model for now.
    
4. **Velocity PI integrators are external to the MATLAB function.** The `dyulon_controller.m` shows only the proportional part of the velocity loop. The integral terms must be implemented as Simulink Discrete Integrator blocks connected via input/output ports to the controller MATLAB Function block, with anti-windup saturation.
    

---

## 6. Software Stack

### 6.1 Simulation Layer (Simulink)

**Purpose:** Physics modelling, control law design, gain tuning, transition validation.

**Files:**

|File|Role|
|---|---|
|`dyulon_init.m`|Run first. Populates the `p` struct with all parameters. Fully expression-driven — change primary inputs and all derived values update.|
|`dyulon_forces_moments.m`|Plant physics core. Takes actuator state + vehicle state → F_body, M_body. Used as MATLAB Function block in Simulink.|
|`dyulon_controller.m`|Blended transition controller. Takes desired state + estimated state → actuator commands.|
|`dyulon_actuators.m`|Prints Transfer Function coefficients for Simulink blocks (motor, tilt servo, aileron).|
|`dyulon_plot_luts.m`|Generates 12-panel validation plot of all LUTs and dynamics. Run after init to sanity-check.|
|`dyulon_simulink_guide.m`|Prints step-by-step Simulink build instructions with exact block types and parameters.|

**Simulink build order:**

1. Plant only (manual force inputs → 6DOF → states). Verify hover equilibrium.
2. Add actuator dynamics (Transfer Fcn + Rate Limiter + Saturation blocks).
3. Add sensor noise (Band-Limited White Noise + Transport Delay).
4. Add inner attitude loop. Test hover stability.
5. Add outer position loop. Test position hold.
6. Add transition blending. Test full hover → cruise → hover manoeuvre.

**Simulink solver settings:**

- Solver: Fixed-step, ode4 (Runge-Kutta 4th order)
- Plant step size: 0.001 s (1 kHz)
- Controller step size: 0.005 s (200 Hz, via Rate Transition blocks)

### 6.2 Autonomous Mission Layer (ArduPilot)

**Purpose:** Waypoint navigation, RC input handling, failsafe logic, GPS-based position control.

**Configuration:** ArduPilot ArduPlane with QuadPlane mode enabled. Tilt-rotor support built in.

**Key parameters:**

```
Q_ENABLE      = 1      % enable quadplane mode
Q_TILT_ENABLE = 1      % enable tilt-rotor
Q_TILT_TYPE   = 1      % continuous tilt (not binary)
Q_TILT_MASK   = 3      % front two motors tilt (bitmask)
Q_TILT_RATE_UP = 40    % deg/s tilt rate
Q_TILT_MAX    = 90     % maximum tilt angle
Q_TRANSITION_MS = 8000 % transition duration target (ms)
Q_VFWD_GAIN   = 0.08   % forward velocity feed-forward
```

**SITL testing:**

```bash
# ArduPlane SITL with tilt-rotor frame
sim_vehicle.py -v ArduPlane --frame tilttrivcopter

# Connect Mission Planner or QGroundControl to localhost:14550
```

**Workflow (Simulink to ArduPilot):**

1. Tune PID gains in Simulink model.
2. Transfer tuned gains into ArduPilot parameter file.
3. Validate autonomous mission logic in ArduPilot SITL.
4. Fly hardware with ArduPilot controlling autonomy, Simulink-tuned gains loaded.

---

## 7. Key Academic Reference

**Mancinelli, A. (2025).** _Hybrid lift UAV design and control for precision landing on a moving vessel in high sea state._ PhD Dissertation, TU Delft. DOI: 10.4233/uuid:e2003e6a-6410-45bb-bf82-e36ef727d25c

**Why it matters to Dyulon:**  
Mancinelli built a 2.3 kg dual-axis tilting rotor quad-plane for autonomous ship deck landing. His vehicle is physically similar to ours (flying wing, 4 tilting motors, fixed-wing cruise). Key takeaways we adopted:

- **INDI (Incremental Nonlinear Dynamic Inversion)** as the control foundation. Mancinelli used a full Nonlinear Control Allocation optimizer (SQP) running at 220 Hz on a Raspberry Pi. We took inspiration from the physics but simplified to a blended allocation scheme without the nonlinear optimizer.
- **Equations of motion structure** — propeller thrust model (T = KpT · Ω²), airspeed correction LUTs, gyroscopic terms, wing aerodynamics. We adopted this framework directly.
- **Transition dynamics** he encountered: (a) pitch oscillation from gain mismatch between hover/cruise actuator bandwidths — fixed with gain scheduling; (b) altitude loss during transition — fixed by coupling pitch increase to motor tilt; (c) smooth transition without mode switching via continuous blending.
- **Propeller-wing interaction** flagged as an unmodelled effect worth characterising early.

**Key differences from Mancinelli:**

- We have forward-only tilt (no lateral). Eliminates gimbal lock entirely.
- We have dual prop types (hover vs cruise optimised). Mancinelli used one prop type throughout.
- Rear motors further from CG (deliberate for pitch authority). Mancinelli had symmetric placement.
- We target delivery efficiency as the primary metric. Mancinelli targeted landing precision.
- We use Simulink + ArduPilot. Mancinelli used Paparazzi UAV autopilot throughout.

---

## 8. Assumptions and Sensitivity

The full assumptions register is in `dyulon_assumptions.xlsx` (47 items across 9 categories). Top-level summary:

|Sensitivity|Parameters|Consequence of being wrong|
|---|---|---|
|**VERY HIGH**|KpT_front_0, KpT_rear_0, D_front_in, D_rear_in|Wrong hover RPM, wrong cruise thrust. Replace with bench test data ASAP.|
|**HIGH**|p.mass, p.S (wing area), CL_max, airspeed correction LUTs, Ixx/Iyy|All force equations, stall speed, transition timing affected.|
|**MEDIUM**|Motor arm positions, Cm_q, motor bandwidth, V_blend_hi, frac_front/rear|Attitude gains, transition quality, pitch stability affected. Tunable in flight test.|
|**LOW**|Oswald efficiency, roll/yaw damping, sensor noise levels|Range estimates, open-loop handling qualities. Fine-tune last.|

**Parameters that auto-update when you change primary inputs (expression chain in `dyulon_init.m`):**

```
p.mass → p.W → p.S → p.b → p.cbar
                          → p.Ixx, p.Iyy, p.Izz
                          → p.r_motor (all 4 positions)
                          → p.V_stall_actual
                          → p.V_blend_hi

p.D_front_in → p.KpT_front_0 → p.Omega_hover_front → p.Omega_max_front
p.D_rear_in  → p.KpT_rear_0  → p.Omega_hover_rear  → p.Omega_max_rear
```

---

## 9. What Has Not Been Decided / Open Questions

1. **Exact airfoil** — TL54/MH60/AG35 class assumed. Run XFLR5 on candidate airfoils and replace `p.aero_CL/CD/Cm` LUTs.
2. **Specific motor models** — Props sized (22"/18"), but actual motor model/KV not selected. Use eCalc with KpT target to find suitable motors.
3. **Wing structure** — Foam/carbon composite assumed but not designed. Structural analysis at 35 kg and 3.16 m span needed before build.
4. **Yaw control mechanism** — Currently only differential torque. May need a small drag rudder or differential rear motor cant angle.
5. **Propeller-wing interaction** — Front prop wash effect on wing AoA during partial tilt not modelled. Wind tunnel or CFD study recommended before tuning transition gains.
6. **Payload integration** — Geometry, mass distribution, and CG shift from payload not yet modelled. Battery + payload CG location directly affects Iyy and pitch trim.
7. **Fail-safe logic** — Motor failure modes not designed. ArduPilot has basic motor failure detection but the redundancy strategy for a 35 kg delivery vehicle needs explicit design.

---

## 10. Coordinate Systems and Sign Conventions

**Body frame:**

- x: forward (positive = nose direction)
- y: right (positive = right wing)
- z: down (positive = toward ground)

**Earth/NED frame:**

- x: North
- y: East
- z: Down (positive toward ground)

**Motor tilt angle β:**

- β = 0: motor thrust pointing straight up (hover)
- β = -90° (−π/2 rad): motor thrust pointing fully forward (cruise)
- β negative = motor tilted forward (normal operating range)

**Euler angles (ZYX convention):**

- φ (phi): roll — positive = right wing down
- θ (theta): pitch — positive = nose up
- ψ (psi): yaw — positive = nose right (clockwise from above)

**Positive moments (right-hand rule):**

- Mx: roll right
- My: pitch up (nose up)
- Mz: yaw right

---

_End of context document. For questions on the Simulink model structure, refer to `dyulon_simulink_guide.m`. For parameter definitions, refer to `dyulon_init.m`. For the assumptions register, refer to `dyulon_assumptions.xlsx`._
