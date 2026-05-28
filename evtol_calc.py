#!/usr/bin/env python3
"""
eVTOL Calc - Electric Drive & Aircraft Design Calculator
Equivalent to eCalc.ch calculators, tailored for eVTOL tilt-rotor design.
Covers: propCalc, xcopterCalc, heliCalc, bladeCalc, perfCalc, cgCalc, w&bCalc, fanCalc, torqueCalc

All formulae and assumptions are documented inline.
"""

import math
import sys

# ─── CONSTANTS ────────────────────────────────────────────────────────────────
RHO_SEA = 1.225          # kg/m³  — ISA sea-level air density
RHO_DEFAULT = 1.225
G = 9.80665              # m/s²
NU_AIR = 1.5e-5          # m²/s  — kinematic viscosity of air at 20 °C

# ─── HELPERS ──────────────────────────────────────────────────────────────────

def header(title: str):
    w = 62
    print("\n" + "═" * w)
    print(f"  {title}")
    print("═" * w)

def sub(title: str):
    print(f"\n── {title} " + "─" * max(0, 55 - len(title)))

def val(label, value, unit=""):
    print(f"  {label:<40} {value:>10.3f}  {unit}")

def sep():
    print("  " + "─" * 58)

def ask(prompt, default=None):
    suffix = f" [{default}]" if default is not None else ""
    raw = input(f"  {prompt}{suffix}: ").strip()
    if raw == "" and default is not None:
        return default
    try:
        return float(raw)
    except ValueError:
        print("  ⚠  Invalid input, using default.")
        return default if default is not None else 0.0

def ask_int(prompt, default=None):
    suffix = f" [{default}]" if default is not None else ""
    raw = input(f"  {prompt}{suffix}: ").strip()
    if raw == "" and default is not None:
        return default
    try:
        return int(raw)
    except ValueError:
        return default if default is not None else 0

def isa_density(altitude_m: float) -> float:
    """
    International Standard Atmosphere (ISA) density vs altitude.
    Uses the troposphere lapse rate: T = 288.15 - 0.0065*h [K]
    ρ = ρ0 * (T/T0)^(g/(R*L) - 1)  where R=287.05, L=0.0065 K/m
    Valid up to 11 000 m (tropopause).
    """
    T0 = 288.15; L = 0.0065; R = 287.05
    T = T0 - L * altitude_m
    T = max(T, 216.65)
    rho = RHO_SEA * (T / T0) ** (G / (R * L) - 1)
    return rho

def rpm_to_rads(rpm):
    return rpm * math.pi / 30.0

# ─── 1. PROP CALC ─────────────────────────────────────────────────────────────

def prop_calc():
    """
    propCalc — Propeller / Motor / Battery Drive Simulation
    ─────────────────────────────────────────────────────────
    Models an electric brushless motor driving a fixed-pitch propeller.

    KEY FORMULAE & ASSUMPTIONS
    • Motor model (no-load speed):
        RPM_no_load = Kv × V_batt_effective
        where V_batt_effective = V_batt – I × R_total
        R_total = R_motor + R_ESC + R_cable + R_battery/n_cells (estimate)

    • Propeller thrust (actuator / empirical):
        T ≈ Ct × ρ × n² × D⁴
        Ct estimated via UIUC/APC empirical fit for typical flat-blade props:
        Ct ≈ 0.109 × (pitch/dia)^0.05   (rough; use bladeCalc for precision)

    • Propeller torque / power:
        Q = Cp × ρ × n² × D⁵ / (2π)    but simpler:
        P_shaft = T × v_e               where v_e = J × n × D (effective velocity)
        For static (hover):  P_shaft = T^(3/2) / sqrt(2 × ρ × A_disk)
        (Momentum theory — ideal induced power)
        Add figure-of-merit η_prop ≈ 0.55–0.75 for real prop losses.

    • Motor shaft power:
        P_mech = (I - I0) × (V_back_emf)  where
        V_back_emf = RPM / Kv
        I0 = no-load current (friction/iron losses)

    • Efficiency chain:
        η_total = η_motor × η_ESC × η_prop
        η_motor = P_mech / P_elec
        η_ESC ≈ 0.95 (assumed; varies 0.92–0.98)

    • Flight time:
        t = (C_bat × η_discharge) / (I_avg × 60)  minutes
        I_avg assumed = I_hover (conservative for eVTOL hover-heavy mission)

    ASSUMPTIONS:
    – Static (hover) condition; no forward airspeed correction.
    – Uniform inflow; no tip losses.
    – Battery voltage is treated as constant (use mid-discharge voltage).
    – ESC efficiency = 0.95.
    """
    header("1. propCalc — Motor + Prop + Battery Simulation")
    print("  For a single rotor (scale up by number of rotors for eVTOL).\n")

    sub("Motor Parameters")
    Kv       = ask("Motor Kv (RPM/V)",               default=400.0)
    R_motor  = ask("Motor winding resistance (Ω)",    default=0.05)
    I0       = ask("No-load current I0 (A)",           default=1.5)
    I_max    = ask("Motor max continuous current (A)", default=80.0)

    sub("Propeller")
    D_in     = ask("Prop diameter (inches)",           default=18.0)
    pitch_in = ask("Prop pitch (inches)",               default=6.0)
    eta_prop = ask("Prop figure of merit (0.5–0.75)",  default=0.60)

    sub("Battery / ESC")
    V_batt   = ask("Battery voltage (V) — use nominal",default=44.4)
    R_batt   = ask("Battery internal resistance (Ω)", default=0.02)
    C_bat    = ask("Battery capacity (mAh)",           default=16000.0)
    eta_ESC  = ask("ESC efficiency (0.92–0.98)",       default=0.95)

    sub("Environment")
    alt_m    = ask("Altitude (m)",                     default=0.0)

    # ── derived ──
    rho   = isa_density(alt_m)
    D     = D_in * 0.0254            # m
    pitch = pitch_in * 0.0254        # m
    A     = math.pi * D**2 / 4.0    # disk area m²
    R_total = R_motor + R_batt

    # Iterative solution: solve for current I that satisfies:
    #   RPM_motor(I) == RPM_prop_at_that_thrust
    # Simplified: use motor RPM at given throttle voltage, find prop power demand.
    # We iterate over throttle 0→1 and report hover operating point.

    sub("Hover Operating Point (finding throttle for T = thrust_needed)")
    MTOW     = ask("Expected thrust needed from THIS rotor (N)", default=50.0)

    # Ct empirical (UIUC-based flat blade approximation):
    J_hover  = 0.0   # static
    pitch_ratio = pitch / D
    Ct = 0.109 * pitch_ratio**0.05 * 1.0   # very rough empirical
    # Cp ≈ Ct * sqrt(Ct/2) / FM  from momentum theory
    # P = Ct * (RPM/60)^2 * D^4 / rho_correction  — approach from T directly

    # Momentum theory: P_ideal = T^1.5 / sqrt(2*rho*A)
    P_ideal = MTOW**1.5 / math.sqrt(2.0 * rho * A)
    P_shaft = P_ideal / eta_prop       # real shaft power needed

    # At this P_shaft, find RPM:
    # P_shaft = Cp * rho * n^3 * D^5  with Cp ≈ Ct * pitch_ratio / (2*pi) * pi^3 (approx)
    # Simpler: from T = Ct*rho*n^2*D^4  →  n = sqrt(T/(Ct*rho*D^4))
    n = math.sqrt(MTOW / (Ct * rho * D**4))   # rev/s
    RPM_needed = n * 60.0

    # Back-emf at that RPM:
    V_emf = RPM_needed / Kv
    # Motor current: V_batt*eta_ESC = V_emf + I*(R_total) → ignore R_batt here
    # V_applied = V_batt * throttle
    # V_applied = V_emf + I*R_total  and  P_mech = (I - I0)*V_emf
    # Solve: I = (P_shaft/V_emf) + I0  (from P_mech = (I-I0)*V_emf)
    I_motor = P_shaft / max(V_emf, 0.1) + I0
    V_applied = V_emf + I_motor * R_total
    throttle  = V_applied / (V_batt * eta_ESC)
    throttle  = min(throttle, 1.0)
    I_batt    = I_motor / eta_ESC
    P_elec    = V_batt * I_batt
    eta_motor = P_shaft / max(P_elec, 0.01)
    eta_total = eta_motor * eta_ESC * eta_prop

    # Thrust more precisely via actual RPM:
    T_check = Ct * rho * (RPM_needed/60.0)**2 * D**4

    # Flight time (hover):
    flight_min = (C_bat / 1000.0) / I_batt * 60.0

    # ── output ──
    sub("Results")
    val("Air density at altitude",      rho,          "kg/m³")
    val("Disk area",                    A,            "m²")
    val("Required RPM",                 RPM_needed,   "RPM")
    val("Prop speed (n)",               n,            "rev/s")
    val("Back-EMF",                     V_emf,        "V")
    val("Motor current",                I_motor,      "A")
    val("Battery current",              I_batt,       "A")
    val("Applied voltage",              V_applied,    "V")
    val("Throttle",                     throttle*100, "%")
    val("Ideal induced power",          P_ideal,      "W")
    val("Shaft power",                  P_shaft,      "W")
    val("Electrical power (battery)",   P_elec,       "W")
    val("Motor efficiency",             eta_motor*100,"%")
    val("Total drive efficiency",       eta_total*100,"%")
    val("Thrust (check)",               T_check,      "N")
    val("Hover flight time",            flight_min,   "min")
    sep()
    if I_motor > I_max:
        print(f"  ⚠  Motor current {I_motor:.1f} A exceeds max {I_max:.1f} A — overloaded!")
    if throttle > 0.95:
        print("  ⚠  Throttle > 95% — little headroom for control!")

# ─── 2. XCOPTER CALC ──────────────────────────────────────────────────────────

def xcopter_calc():
    """
    xcopterCalc — Multirotor / eVTOL Hover Performance
    ─────────────────────────────────────────────────────
    KEY FORMULAE & ASSUMPTIONS
    • Total thrust required (hover):
        T_total = MTOW × g × (1 + hover_margin)
        Each rotor provides T_rotor = T_total / n_rotors

    • Disk loading:
        DL = T_rotor / A_disk   [N/m²]
        Low DL → higher efficiency, larger disks, better endurance.

    • Figure of Merit (FM):
        FM = P_ideal / P_actual   (ideal = momentum theory)
        P_ideal_rotor = T_rotor^1.5 / sqrt(2*ρ*A)
        Typical FM: 0.55 (small cheap prop) – 0.80 (optimised blade)

    • Power loading:
        PL = T_total / P_total   [N/W or g/W]
        Target for good efficiency: > 8 g/W for copters, > 12 g/W for large eVTOL.

    • Specific endurance:
        Hover time = (E_bat × η_discharge) / P_total  [h]
        E_bat = V_nom × C_bat [Wh]

    • Induced velocity (hover):
        v_i = sqrt(T_rotor / (2*ρ*A))   [m/s]

    • Torque balance for yaw (quadcopter-style):
        Counter-rotating pairs cancel torque.
        For tilt-rotor: yaw via differential rotor tilt, not torque cancellation.

    ASSUMPTIONS:
    – Uniform inflow, no mutual interference between rotors (conservative).
    – Battery at nominal voltage (use 3.7 V/cell LiPo nominal or 3.85 V mean).
    – No ground effect (if hover height > 0.5×D, ground effect is < 5%).
    – Rotor interference factor not applied (add ~5–10% power penalty for
      closely spaced rotors; eVTOL tilt-rotor blades are usually separated).
    """
    header("2. xcopterCalc — Multirotor / eVTOL Hover")

    sub("Airframe")
    MTOW        = ask("Total MTOW (kg)",                  default=25.0)
    n_rotors    = ask_int("Number of rotors",              default=4)
    hover_margin= ask("Hover thrust margin (fraction, e.g. 0.2 = 20%)", default=0.20)

    sub("Rotor Geometry")
    D_in        = ask("Rotor diameter (inches)",           default=24.0)
    FM          = ask("Figure of merit (0.55–0.80)",       default=0.65)

    sub("Battery / Power")
    V_nom       = ask("Battery nominal voltage (V)",       default=44.4)
    C_bat_Ah    = ask("Battery capacity (Ah)",             default=20.0)
    eta_dis     = ask("Battery discharge efficiency",      default=0.90)
    P_avionics  = ask("Avionics / aux power (W)",          default=50.0)

    sub("Environment")
    alt_m       = ask("Altitude (m)",                      default=0.0)

    # ── derived ──
    rho   = isa_density(alt_m)
    D     = D_in * 0.0254
    A     = math.pi * D**2 / 4.0
    W     = MTOW * G
    T_tot = W * (1 + hover_margin)
    T_r   = T_tot / n_rotors
    DL    = T_r / A                              # N/m²
    P_ideal_r  = T_r**1.5 / math.sqrt(2 * rho * A)
    P_actual_r = P_ideal_r / FM
    P_rotor_tot= P_actual_r * n_rotors
    P_total    = P_rotor_tot + P_avionics
    v_i        = math.sqrt(T_r / (2 * rho * A))
    PL_gW      = (T_tot * 1000 / G) / P_total   # g/W
    E_bat      = V_nom * C_bat_Ah                # Wh
    hover_h    = (E_bat * eta_dis) / P_total     # hours
    hover_min  = hover_h * 60
    I_hover    = P_total / V_nom

    sub("Results")
    val("Air density",                  rho,            "kg/m³")
    val("Disk area per rotor",          A,              "m²")
    val("Total disk area",              A*n_rotors,     "m²")
    val("Weight (N)",                   W,              "N")
    val("Total thrust required",        T_tot,          "N")
    val("Thrust per rotor",             T_r,            "N")
    val("Disk loading per rotor",       DL,             "N/m²")
    val("Ideal power per rotor",        P_ideal_r,      "W")
    val("Actual power per rotor",       P_actual_r,     "W")
    val("Total rotor power",            P_rotor_tot,    "W")
    val("Total power incl avionics",    P_total,        "W")
    val("Induced velocity (hover)",     v_i,            "m/s")
    val("Power loading",                PL_gW,          "g/W")
    val("Battery current (hover)",      I_hover,        "A")
    val("Hover endurance",              hover_min,      "min")
    sep()
    if DL > 200:
        print("  ⚠  Disk loading > 200 N/m²: high power demand. Consider larger rotors.")
    if PL_gW < 8:
        print("  ⚠  Power loading < 8 g/W: poor efficiency. Increase disk area or FM.")
    if hover_min < 15:
        print("  ⚠  Hover time < 15 min. Increase battery or reduce MTOW.")

# ─── 3. HELI CALC ─────────────────────────────────────────────────────────────

def heli_calc():
    """
    heliCalc — Single Main Rotor Helicopter / Tilt-Rotor Hover
    ─────────────────────────────────────────────────────────────
    KEY FORMULAE & ASSUMPTIONS
    • Tail rotor power fraction:
        P_tail ≈ 0.08–0.12 × P_main   (typical RC/small heli; larger aircraft ~0.10)
        For tilt-rotor (no tail rotor): P_tail = 0

    • Collective pitch → thrust (simplified blade element):
        T ≈ ρ × a × σ × A × (Ω×R)² × (θ_0/3 + θ_tw/4 - λ_i/2)
        where:
          a   = blade lift-curve slope ≈ 5.7 /rad (flat plate ≈ 2π, real ≈ 5.7)
          σ   = solidity = N_b × c / (π × R)
          θ_0 = collective pitch angle (rad)
          θ_tw= blade twist (negative nose-down, typically –8° to –12°)
          λ_i = v_i / (Ω×R) — inflow ratio

    • Profile drag power:
        P_profile = σ × Cd0 × ρ × A × (Ω×R)³ / 8
        Cd0 ≈ 0.010–0.015 for typical aerofoils

    • Total shaft power:
        P_shaft = P_induced + P_profile + P_tail

    ASSUMPTIONS:
    – Uniform inflow (Glauert correction not applied).
    – No forward flight — hover only.
    – Blade twist and collective pitch entered as inputs.
    – No compressibility effects (tip Mach assumed < 0.6).
    """
    header("3. heliCalc — Single Main Rotor / Tilt-Rotor Hover")

    sub("Rotor Geometry")
    R       = ask("Rotor radius (m)",                   default=0.60)
    N_b     = ask_int("Number of blades",               default=2)
    c       = ask("Blade chord (m)",                    default=0.045)
    theta0  = ask("Collective pitch angle (degrees)",   default=10.0)
    theta_tw= ask("Blade twist (degrees, –ve = washout)",default=-8.0)
    Cd0     = ask("Profile drag coefficient Cd0",       default=0.012)

    sub("Drive")
    RPM     = ask("Rotor RPM",                          default=2200.0)
    tail_fraction = ask("Tail rotor power fraction (0 for tilt-rotor)", default=0.0)

    sub("Aircraft & Environment")
    MTOW    = ask("MTOW (kg)",                          default=5.0)
    alt_m   = ask("Altitude (m)",                       default=0.0)

    rho     = isa_density(alt_m)
    A       = math.pi * R**2
    Omega   = rpm_to_rads(RPM)
    VT      = Omega * R                       # tip speed m/s
    sigma   = N_b * c / (math.pi * R)        # solidity
    a_slope = 5.73                            # lift-curve slope /rad
    theta0_r= math.radians(theta0)
    tw_r    = math.radians(theta_tw)

    # Iterative λ_i (inflow ratio) from momentum theory + blade element:
    W = MTOW * G
    CT_target = W / (rho * A * VT**2)
    # First-order λ_i from momentum: CT = 2*λ_i^2 → λ_i = sqrt(CT/2)
    lam_i = math.sqrt(CT_target / 2.0)

    # Blade element CT with linear twist:
    CT_be = 0.5 * a_slope * sigma * (theta0_r / 3.0 + tw_r / 4.0 - lam_i / 2.0)
    # iterate once:
    lam_i = math.sqrt(abs(CT_be) / 2.0)
    CT_be = 0.5 * a_slope * sigma * (theta0_r / 3.0 + tw_r / 4.0 - lam_i / 2.0)
    T_calc = CT_be * rho * A * VT**2

    # Power
    v_i      = lam_i * VT
    P_induced= W * v_i
    P_profile= sigma * Cd0 * rho * A * VT**3 / 8.0
    P_main   = P_induced + P_profile
    P_tail   = tail_fraction * P_main
    P_total  = P_main + P_tail
    tip_mach = VT / 340.0

    sub("Results")
    val("Air density",              rho,        "kg/m³")
    val("Disk area",                A,          "m²")
    val("Tip speed",                VT,         "m/s")
    val("Tip Mach number",          tip_mach,   "")
    val("Rotor solidity",           sigma,      "")
    val("Inflow ratio λ_i",         lam_i,      "")
    val("Induced velocity",         v_i,        "m/s")
    val("Thrust coefficient CT",    CT_be,      "")
    val("Calculated thrust",        T_calc,     "N")
    val("Required weight",          W,          "N")
    val("Induced power",            P_induced,  "W")
    val("Profile drag power",       P_profile,  "W")
    val("Tail rotor power",         P_tail,     "W")
    val("Total shaft power",        P_total,    "W")
    sep()
    if tip_mach > 0.7:
        print("  ⚠  Tip Mach > 0.7: compressibility drag significant. Reduce RPM or radius.")
    if abs(T_calc - W) / W > 0.20:
        print(f"  ⚠  Blade element thrust {T_calc:.1f} N ≠ weight {W:.1f} N "
              f"— adjust collective pitch.")

# ─── 4. BLADE CALC ────────────────────────────────────────────────────────────

def blade_calc():
    """
    bladeCalc — Propeller / Rotor Blade Performance
    ─────────────────────────────────────────────────
    KEY FORMULAE & ASSUMPTIONS
    Uses Blade Element Momentum Theory (BEMT):

    • Each annular element dr at radius r:
        dT = N_b × 0.5 × ρ × W²(r) × c(r) × (Cl × cos(φ) − Cd × sin(φ)) × dr
        dQ = N_b × 0.5 × ρ × W²(r) × c(r) × (Cl × sin(φ) + Cd × cos(φ)) × r × dr

        where W = resultant velocity, φ = inflow angle

    • Inflow angle: φ = arctan(v_i / (Ω × r))
        Local AoA: α = θ(r) − φ
        θ(r) = θ_root + twist × (r/R)

    • Cl = a × α   (linear; Cl_max limited by stall)
        Cd ≈ Cd0 + Cl² / (π × e × AR_blade)  (not used for strip; use Cd0+k*Cl^2)

    • Integration (N strips):
        T = ∫₀ᴿ dT,  Q = ∫₀ᴿ dQ,  P = Ω × Q

    • Thrust & Torque coefficients:
        CT = T / (ρ × n² × D⁴)
        CQ = Q / (ρ × n² × D⁵)
        CP = 2π × CQ

    ASSUMPTIONS:
    – Prandtl tip-loss factor B = 1 − (1/N_b)^0.5 applied at tip (simplified form).
    – Linear chord distribution (constant chord option available).
    – Hover (J=0) — Vinfty = 0 for eVTOL sizing.
    – Stall limited at Cl_max = 1.2.
    """
    header("4. bladeCalc — Blade Element Momentum Theory")

    sub("Blade Geometry")
    R       = ask("Rotor radius (m)",                   default=0.30)
    N_b     = ask_int("Number of blades",               default=2)
    c_root  = ask("Chord at root (m)",                  default=0.030)
    c_tip   = ask("Chord at tip (m)",                   default=0.020)
    theta_root= ask("Pitch at root (degrees)",          default=15.0)
    theta_tip = ask("Pitch at tip (degrees)",           default=8.0)
    Cd0     = ask("Profile drag Cd0",                   default=0.012)
    a_slope = ask("Lift-curve slope a (/rad)",          default=5.73)
    Cl_max  = ask("Maximum Cl (stall limit)",           default=1.2)

    sub("Operating Condition")
    RPM     = ask("Rotor RPM",                          default=4000.0)
    alt_m   = ask("Altitude (m)",                       default=0.0)

    rho   = isa_density(alt_m)
    Omega = rpm_to_rads(RPM)
    N     = 100           # number of radial strips
    T_total = 0.0; Q_total = 0.0
    r_start = 0.10 * R    # hub cutout

    for i in range(N):
        r = r_start + (R - r_start) * (i + 0.5) / N
        dr = (R - r_start) / N
        frac = r / R
        c    = c_root + (c_tip - c_root) * frac
        theta= math.radians(theta_root + (theta_tip - theta_root) * frac)
        Vt   = Omega * r
        # Prandtl tip-loss
        f    = (N_b / 2.0) * (R - r) / max(r * 0.01, r)
        F    = 2.0 / math.pi * math.acos(min(1.0, math.exp(-f)))

        # Inflow from momentum + BE coupling (one iteration):
        # Glauert-simplified: dCT/dr = 4*F*lam_i*(lam_i+lam_c)*r_hat
        # Solved with linear BE assumption:
        lam_c = 0.0   # hover: no climb
        # initial guess:
        phi   = math.radians(5.0)
        for _ in range(20):
            alpha = theta - phi
            alpha = max(min(alpha, math.radians(15)), math.radians(-5))
            Cl    = min(a_slope * alpha, Cl_max)
            Cl    = max(Cl, -0.2)
            Cd    = Cd0 + 0.02 * Cl**2
            # momentum inflow:
            sin_phi = math.sin(phi); cos_phi = math.cos(phi)
            W_sq  = Vt**2 / (cos_phi**2 + 1e-9)
            sigma_r = N_b * c / (2 * math.pi * r)
            lam_i_iter = (sigma_r * a_slope / 16.0 / F) * (
                math.sqrt(1 + (32 * F * theta * r / (R * sigma_r * a_slope))) - 1)
            phi_new = math.atan2(lam_i_iter * Omega * R, Vt)
            phi = 0.5 * phi + 0.5 * phi_new

        alpha = theta - phi
        Cl    = min(a_slope * alpha, Cl_max)
        Cd    = Cd0 + 0.02 * Cl**2
        W_res = math.sqrt((Omega * r)**2 + (Omega * R * math.sin(phi))**2)
        dT    = N_b * 0.5 * rho * W_res**2 * c * (Cl * math.cos(phi) - Cd * math.sin(phi)) * dr
        dQ    = N_b * 0.5 * rho * W_res**2 * c * (Cl * math.sin(phi) + Cd * math.cos(phi)) * r * dr
        T_total += max(dT, 0)
        Q_total += max(dQ, 0)

    P_shaft = Omega * Q_total
    D       = 2 * R
    n       = RPM / 60.0
    CT      = T_total / max(rho * n**2 * D**4, 1e-9)
    CQ      = Q_total / max(rho * n**2 * D**5, 1e-9)
    CP      = 2 * math.pi * CQ
    A       = math.pi * R**2
    FM      = (T_total**1.5 / math.sqrt(2*rho*A)) / max(P_shaft, 1e-6)

    sub("Results")
    val("Total thrust",         T_total,    "N")
    val("Total torque",         Q_total,    "N·m")
    val("Shaft power",          P_shaft,    "W")
    val("Thrust coeff CT",      CT,         "")
    val("Torque coeff CQ",      CQ,         "")
    val("Power coeff CP",       CP,         "")
    val("Figure of merit FM",   FM,         "")

# ─── 5. PERF CALC ─────────────────────────────────────────────────────────────

def perf_calc():
    """
    perfCalc — Fixed-Wing / eVTOL Cruise Performance
    ──────────────────────────────────────────────────
    KEY FORMULAE & ASSUMPTIONS
    Tilt-rotor aircraft in cruise: treated as fixed-wing with propulsive thrust.

    • Lift = Weight in level cruise: L = W = 0.5 × ρ × V² × S × CL
    • Drag polar: CD = CD0 + CL²/(π × e × AR)
        CD0 ≈ 0.025 (clean eVTOL airframe); e ≈ 0.80 (Oswald)
    • Thrust required: T_req = D = 0.5 × ρ × V² × S × CD
    • Power required: P_req = T_req × V
    • Minimum power speed (best endurance):
        V_mp = (2W / (ρ S sqrt(3 CD0 π e AR)))^0.5
    • Best L/D speed (best range):
        V_md = sqrt(2W / (ρ S)) × (CD0 / (π e AR))^0.25   (approx)
    • Rate of climb: RC = (P_avail − P_req) × η_total / W
    • Range (Breguet): R = η_total × E_spec / g × (L/D) × ln(W0/W1)
        E_spec = battery specific energy (Wh/kg)
        η_total = η_prop × η_motor × η_ESC

    ASSUMPTIONS:
    – Level, unaccelerated flight.
    – No wind.
    – Battery fully discharged (W1 = W0 − m_bat×g for range calc).
    – Electric Breguet ignores weight change (battery mass fraction small).
    """
    header("5. perfCalc — Cruise / Fixed-Wing Performance")

    sub("Airframe")
    MTOW    = ask("MTOW (kg)",                          default=25.0)
    S       = ask("Wing area (m²)",                     default=1.20)
    AR      = ask("Wing aspect ratio",                  default=8.0)
    e       = ask("Oswald efficiency factor",           default=0.80)
    CD0     = ask("Zero-lift drag coefficient CD0",     default=0.025)

    sub("Propulsion")
    P_max   = ask("Max available shaft power (W)",      default=3000.0)
    eta_total= ask("Total drivetrain efficiency",       default=0.80)

    sub("Battery")
    E_bat   = ask("Battery energy (Wh)",                default=740.0)
    m_bat   = ask("Battery mass (kg)",                  default=4.0)

    sub("Environment")
    alt_m   = ask("Cruise altitude (m)",                default=120.0)
    V_cruise= ask("Cruise speed (m/s)",                 default=25.0)

    rho     = isa_density(alt_m)
    W       = MTOW * G
    q       = 0.5 * rho * V_cruise**2
    CL      = W / (q * S)
    CD      = CD0 + CL**2 / (math.pi * e * AR)
    LD      = CL / CD
    D       = q * S * CD
    P_req   = D * V_cruise
    P_avail = P_max * eta_total
    RC      = max(0, (P_avail - P_req) / W)
    # Min power speed
    V_mp    = math.sqrt(W / (0.5 * rho * S) * math.sqrt(CD0 / (3 * CD0 * math.pi * e * AR + 1e-9)))
    V_md    = math.sqrt(2*W / (rho * S)) * (CD0 / (math.pi * e * AR))**0.25
    # Range (electric Breguet approximation):
    E_use   = E_bat * 0.90 * 3600    # J (90% usable)
    Range   = eta_total * E_use / W * LD  # m
    Endur   = E_use / max(P_req, 1) / 60  # min

    sub("Results")
    val("Air density",              rho,        "kg/m³")
    val("Dynamic pressure",         q,          "Pa")
    val("Lift coefficient CL",      CL,         "")
    val("Drag coefficient CD",      CD,         "")
    val("L/D ratio",                LD,         "")
    val("Drag force",               D,          "N")
    val("Power required (cruise)",  P_req,      "W")
    val("Power available",          P_avail,    "W")
    val("Rate of climb",            RC,         "m/s")
    val("Best endurance speed Vmp", V_mp,       "m/s")
    val("Best range speed Vmd",     V_md,       "m/s")
    val("Estimated cruise range",   Range/1000, "km")
    val("Hover endurance",          Endur,      "min")
    sep()
    if CL > 1.4:
        print("  ⚠  CL > 1.4 at cruise: approaching stall. Increase speed or wing area.")
    if P_req > P_avail:
        print("  ⚠  Power required exceeds available — cannot maintain cruise speed!")

# ─── 6. CG CALC ───────────────────────────────────────────────────────────────

def cg_calc():
    """
    cgCalc — Center of Gravity, Neutral Point & Static Margin
    ──────────────────────────────────────────────────────────
    KEY FORMULAE & ASSUMPTIONS
    • CG position from datum:
        x_cg = Σ(m_i × x_i) / Σ(m_i)
        z_cg = Σ(m_i × z_i) / Σ(m_i)

    • Neutral Point (NP) for tailed aircraft (Perkins & Hage):
        x_NP = x_AC_wing + (a_h / a_w) × (1 − dε/dα) × V_H × (x_wing / MAC)
        Simplified for input: user provides estimated NP as % MAC.

    • Static margin (SM):
        SM = (x_NP − x_CG) / MAC × 100%
        Positive SM → statically stable.
        Typical target: 5–15% MAC for stable aircraft; 0–5% for agile/FBW eVTOL.

    • Mean Aerodynamic Chord (MAC):
        For trapezoidal wing: MAC = (2/3) × c_root × (1 + λ + λ²)/(1 + λ)
        where λ = c_tip / c_root (taper ratio)

    ASSUMPTIONS:
    – CG measured from nose datum (forward = positive x, up = positive z).
    – Component masses entered individually; battery and motor are dominant.
    – NP calculation is simplified; use XFLR5 or AVL for full aerodynamic NP.
    – Tail contribution to NP estimated via V_H (horizontal tail volume).
    """
    header("6. cgCalc — Center of Gravity & Static Margin")

    sub("Wing Geometry (for MAC and NP)")
    c_root  = ask("Root chord (m)",                     default=0.25)
    c_tip   = ask("Tip chord (m)",                      default=0.15)
    b       = ask("Wing span (m)",                      default=2.0)
    x_wing_LE= ask("Wing LE position from nose datum (m)",default=0.50)

    lam     = c_tip / c_root
    MAC     = (2/3) * c_root * (1 + lam + lam**2) / (1 + lam)
    AR      = b**2 / (0.5*(c_root+c_tip)*b)   # trapezoidal area

    sub("Horizontal Tail (leave 0 for tailless / pure copter)")
    S_h     = ask("Horizontal tail area (m²)",          default=0.15)
    x_tail  = ask("Tail AC from nose datum (m)",        default=1.40)
    S_wing  = 0.5 * (c_root + c_tip) * b

    sub("Components — enter mass & CG x-position from nose datum")
    print("  (Enter 0 for mass to skip a component)")
    components = [
        ("Airframe / fuselage",        5.0,  0.60),
        ("Wing structure",             2.0,  0.65),
        ("Battery",                    4.0,  0.55),
        ("Motors (all rotors)",        2.0,  0.45),
        ("ESCs + wiring",              0.5,  0.50),
        ("Flight controller",          0.2,  0.40),
        ("Payload",                    3.0,  0.50),
    ]

    total_mass = 0.0; total_moment = 0.0
    for name, m_def, x_def in components:
        print(f"\n  Component: {name}")
        m = ask(f"  Mass (kg)", default=m_def)
        x = ask(f"  x from nose (m)", default=x_def)
        total_mass   += m
        total_moment += m * x

    x_cg = total_moment / max(total_mass, 0.001)

    sub("Neutral Point (simplified)")
    # AC of wing typically at 25% MAC:
    x_AC_wing = x_wing_LE + 0.25 * MAC
    # Tail contribution (simple V_H approach):
    l_t   = x_tail - x_AC_wing
    V_H   = (S_h * l_t) / (S_wing * MAC) if S_wing > 0 else 0
    dEda  = ask("dε/dα downwash gradient (typ 0.3–0.5)", default=0.40)
    a_rat = ask("Tail/wing lift-curve slope ratio (typ 0.9)", default=0.90)
    x_NP  = x_AC_wing + a_rat * (1 - dEda) * V_H * MAC

    SM_percent = (x_NP - x_cg) / MAC * 100.0

    sub("Results")
    val("Mean aerodynamic chord MAC",   MAC,            "m")
    val("Wing AR",                      AR,             "")
    val("Total mass",                   total_mass,     "kg")
    val("CG position from nose",        x_cg,           "m")
    val("Wing AC from nose",            x_AC_wing,      "m")
    val("Neutral Point from nose",      x_NP,           "m")
    val("Static margin",                SM_percent,     "% MAC")
    val("CG as % MAC (from LE)",        (x_cg - x_wing_LE)/MAC*100, "% MAC")
    sep()
    if SM_percent < 0:
        print("  ⚠  Negative static margin — statically UNSTABLE. Requires FBW control.")
    elif SM_percent < 3:
        print("  ℹ  SM < 3%: marginally stable. Suitable for FBW-controlled eVTOL.")
    elif SM_percent > 20:
        print("  ⚠  SM > 20%: very stable — elevator authority may be insufficient.")

# ─── 7. W&B CALC ──────────────────────────────────────────────────────────────

def wb_calc():
    """
    w&bCalc — Weight & Balance Envelope
    ──────────────────────────────────────
    KEY FORMULAE & ASSUMPTIONS
    • Checks CG envelope under various loading scenarios:
      forward CG limit, aft CG limit (from SM requirements).
    • Loading cases: min payload, max payload, battery positions.
    • CG excursion: Δx_cg = Δm × (x_item − x_cg_current) / m_total_new

    ASSUMPTIONS:
    – CG limits defined as % MAC.
    – Payload placed at user-defined station.
    – Battery slides fore/aft: useful for CG trimming on eVTOL.
    """
    header("7. w&bCalc — Weight & Balance Envelope")

    sub("Reference Geometry")
    MAC     = ask("MAC (m)",                            default=0.22)
    x_LEMAC = ask("LE of MAC from nose datum (m)",      default=0.50)
    CG_fwd  = ask("Forward CG limit (% MAC)",           default=15.0)
    CG_aft  = ask("Aft CG limit (% MAC)",               default=35.0)

    sub("Empty Aircraft")
    m_empty = ask("Empty mass (kg)",                    default=12.0)
    x_empty = ask("Empty CG from nose (m)",             default=0.58)

    sub("Battery")
    m_bat   = ask("Battery mass (kg)",                  default=4.0)
    x_bat   = ask("Battery CG from nose (m)",           default=0.55)

    sub("Payload Cases")
    m_pl_min= ask("Min payload (kg)",                   default=0.0)
    m_pl_max= ask("Max payload (kg)",                   default=5.0)
    x_pl    = ask("Payload CG from nose (m)",           default=0.60)

    cases = [
        ("Min payload, battery fwd",   m_empty, x_empty, m_bat, x_bat-0.05, m_pl_min, x_pl),
        ("Min payload, battery aft",   m_empty, x_empty, m_bat, x_bat+0.05, m_pl_min, x_pl),
        ("Max payload, battery fwd",   m_empty, x_empty, m_bat, x_bat-0.05, m_pl_max, x_pl),
        ("Max payload, battery aft",   m_empty, x_empty, m_bat, x_bat+0.05, m_pl_max, x_pl),
    ]

    sub("Results")
    print(f"  {'Case':<38} {'MTOW(kg)':>9} {'CG(m)':>8} {'%MAC':>7} {'OK':>4}")
    print("  " + "─" * 68)
    for name, me, xe, mb, xb, mp, xp in cases:
        mt = me + mb + mp
        xg = (me*xe + mb*xb + mp*xp) / max(mt, 0.001)
        pct= (xg - x_LEMAC) / MAC * 100
        ok = "✓" if CG_fwd <= pct <= CG_aft else "✗"
        print(f"  {name:<38} {mt:>9.2f} {xg:>8.3f} {pct:>7.1f} {ok:>4}")

# ─── 8. FAN CALC (EDF) ────────────────────────────────────────────────────────

def fan_calc():
    """
    fanCalc — Electric Ducted Fan (EDF) Performance
    ──────────────────────────────────────────────────
    KEY FORMULAE & ASSUMPTIONS
    For tilt-rotor eVTOL: EDFs are rarely used, but included for completeness
    (e.g. auxiliary fan thrusters or boundary layer ingestion concepts).

    • Duct pressure ratio:
        ΔP = 0.5 × ρ × (V_exit² − V_inlet²)
    • Ideal fan thrust:
        T_ideal = ṁ × (V_exit − V_inlet)
        ṁ = ρ × A_fan × V_exit  (conservation of mass, exit area = fan area)
    • Fan efficiency (isentropic):
        η_fan = ΔP × Q_vol / P_shaft
    • Static thrust (V_inlet = 0):
        T_static = sqrt(2 × ρ × A × P_shaft³)^(1/3) — from momentum theory
        More practically: T = 2 × ρ × A × v_exit²  where v_exit = sqrt(P_shaft/(ρ×A))

    ASSUMPTIONS:
    – Uniform exit velocity profile.
    – No duct lip loss (η_duct ≈ 0.90 applied).
    – Inlet total pressure recovery ≈ 1.0 (no ram drag in hover).
    – Fan diameter = duct inner diameter.
    """
    header("8. fanCalc — Electric Ducted Fan")

    sub("Fan Geometry")
    D_fan   = ask("Fan diameter (mm)",              default=70.0)
    eta_fan = ask("Fan aerodynamic efficiency",     default=0.80)
    eta_duct= ask("Duct efficiency (typ 0.85–0.95)",default=0.90)

    sub("Drive")
    P_shaft = ask("Shaft power (W)",                default=1500.0)
    RPM     = ask("Fan RPM",                        default=40000.0)

    sub("Environment")
    alt_m   = ask("Altitude (m)",                   default=0.0)

    rho     = isa_density(alt_m)
    D       = D_fan / 1000.0
    A       = math.pi * D**2 / 4.0
    # Static thrust from momentum theory:
    # P = T * v_exit,  T = 2*rho*A*v_exit^2  → v_exit = (T/(2*rho*A))
    # Combined: P_fluid = T^(3/2)/sqrt(2*rho*A)
    # T = (P_fluid * sqrt(2*rho*A))^(2/3)
    P_fluid = P_shaft * eta_fan * eta_duct
    T_ideal = (P_fluid * math.sqrt(2 * rho * A))**(2/3)
    v_exit  = T_ideal / (2 * rho * A * max(1e-6, 1))   # corrected
    v_exit  = math.sqrt(P_fluid / max(rho * A, 1e-9))  # simpler form
    T_static= 2 * rho * A * v_exit**2 / 2              # = rho*A*v_exit^2 (momentum)
    T_static= math.sqrt(2 * rho * A * P_fluid**2 / P_fluid) if P_fluid > 0 else 0
    # Correct formula: T = (2*rho*A*P^2)^(1/3)
    T_static= (2 * rho * A * P_fluid**2)**(1/3)
    # Thrust/weight ratio sense:
    PL      = T_static / P_shaft * 1000   # g/W

    sub("Results")
    val("Air density",          rho,        "kg/m³")
    val("Fan disk area",        A*1e4,      "cm²")
    val("Fluid power",          P_fluid,    "W")
    val("Exit velocity",        v_exit,     "m/s")
    val("Static thrust",        T_static,   "N")
    val("Thrust (grams)",       T_static/G*1000, "g")
    val("Power loading",        PL,         "g/W")

# ─── 9. TORQUE CALC ───────────────────────────────────────────────────────────

def torque_calc():
    """
    torqueCalc — Motor / Drivetrain Torque & Power
    ──────────────────────────────────────────────
    KEY FORMULAE & ASSUMPTIONS
    • Motor torque: Q = P_mech / ω = (V − I×R) × I / ω
        where ω = RPM × π/30
    • Torque constant: Kt = 1/(Kv × π/30) = 60/(2π×Kv)  [N·m/A]
        Physical relation: Q = Kt × (I − I0)
    • Mechanical power: P_mech = Q × ω
    • Winding loss: P_loss = I² × R_winding
    • Motor efficiency: η = P_mech / (V × I) = 1 − (I²R + I×V_friction)/(VI)

    ASSUMPTIONS:
    – Brushless DC motor modelled as ideal back-EMF source + resistance.
    – Iron losses lumped into I0 (no-load current).
    – No gearbox (direct drive), or efficiency of gearbox applied as η_gear.
    """
    header("9. torqueCalc — Motor Torque & Power")

    sub("Motor")
    Kv      = ask("Motor Kv (RPM/V)",           default=400.0)
    R_wind  = ask("Winding resistance (Ω)",     default=0.05)
    I0      = ask("No-load current I0 (A)",     default=1.5)

    sub("Operating Point")
    V_supply= ask("Supply voltage (V)",         default=44.4)
    I_op    = ask("Operating current (A)",      default=60.0)
    eta_gear= ask("Gearbox efficiency (1=direct drive)", default=1.0)

    Kt      = 60.0 / (2 * math.pi * Kv)
    V_emf   = V_supply - I_op * R_wind
    RPM     = V_emf * Kv
    omega   = rpm_to_rads(RPM)
    Q_motor = Kt * max(I_op - I0, 0)
    P_mech  = Q_motor * omega
    P_elec  = V_supply * I_op
    P_loss  = I_op**2 * R_wind + I0 * V_emf
    eta_mot = P_mech / max(P_elec, 0.001)
    Q_out   = Q_motor * eta_gear
    P_out   = P_mech * eta_gear

    sub("Results")
    val("Torque constant Kt",       Kt,         "N·m/A")
    val("Back-EMF",                 V_emf,      "V")
    val("Operating RPM",            RPM,        "RPM")
    val("Angular velocity ω",       omega,      "rad/s")
    val("Motor torque",             Q_motor,    "N·m")
    val("Output shaft torque",      Q_out,      "N·m")
    val("Mechanical power",         P_mech,     "W")
    val("Output shaft power",       P_out,      "W")
    val("Electrical power in",      P_elec,     "W")
    val("Winding + iron losses",    P_loss,     "W")
    val("Motor efficiency",         eta_mot*100,"%")

# ─── 10. EV CALC (Battery Range) ──────────────────────────────────────────────

def ev_calc():
    """
    evCalc / chargeCalc — Battery Energy & eVTOL Mission Range
    ───────────────────────────────────────────────────────────
    KEY FORMULAE & ASSUMPTIONS
    • Energy budget per mission segment:
        E_segment = P_segment × t_segment
    • State of charge (SoC) tracking:
        SoC_remaining = E_bat_usable − Σ E_segments
    • Specific energy vs pack voltage:
        E_bat = V_nom × C_bat (Wh)
        Usable = E_bat × DoD (depth of discharge, typ 0.80–0.90)

    • Range (cruise segment):
        R = V_cruise × t_cruise = V_cruise × E_cruise / P_cruise

    • Charge time (constant current):
        t_charge = C_bat / I_charge  (hours)
        For CC-CV: t_actual ≈ 1.2 × t_CC  (20% overhead for CV taper)

    ASSUMPTIONS:
    – Peukert effect not modelled (relevant for high-C-rate discharge).
    – Battery weight included in MTOW.
    – Hover segments dominate power draw for eVTOL.
    – Reserves: 20% battery held back (EASA SC-VTOL requirement).
    """
    header("10. evCalc — Battery Energy & eVTOL Mission Planner")

    sub("Battery Pack")
    V_nom   = ask("Nominal pack voltage (V)",       default=44.4)
    C_bat   = ask("Pack capacity (Ah)",             default=20.0)
    DoD     = ask("Usable depth of discharge",      default=0.80)
    reserve = ask("Reserve fraction (e.g. 0.20)",   default=0.20)

    sub("Mission Profile")
    print("  Define 4 mission segments: hover takeoff, climb, cruise, hover landing")
    P_h1    = ask("Hover takeoff power (W)",        default=4000.0)
    t_h1    = ask("Hover takeoff duration (s)",     default=30.0)
    P_climb = ask("Climb power (W)",                default=3500.0)
    t_climb = ask("Climb duration (s)",             default=60.0)
    P_cruise= ask("Cruise power (W)",               default=1200.0)
    t_cruise= ask("Cruise duration (s)",            default=600.0)
    V_cruise= ask("Cruise speed (m/s)",             default=25.0)
    P_h2    = ask("Hover landing power (W)",        default=3800.0)
    t_h2    = ask("Hover landing duration (s)",     default=30.0)

    sub("Charging")
    I_charge= ask("Charge current (A)",             default=20.0)
    V_charge= ask("Charger voltage (V)",            default=50.4)

    E_total   = V_nom * C_bat              # Wh
    E_usable  = E_total * DoD * (1-reserve)

    segs = [
        ("Hover Takeoff", P_h1,    t_h1),
        ("Climb",         P_climb, t_climb),
        ("Cruise",        P_cruise,t_cruise),
        ("Hover Landing", P_h2,    t_h2),
    ]

    sub("Mission Energy Budget")
    print(f"  {'Segment':<20} {'Power(W)':>9} {'Time(s)':>8} {'Energy(Wh)':>11}")
    print("  " + "─" * 52)
    E_used = 0.0
    for name, P, t in segs:
        E = P * t / 3600.0
        E_used += E
        print(f"  {name:<20} {P:>9.0f} {t:>8.0f} {E:>11.2f}")
    print("  " + "─" * 52)
    print(f"  {'TOTAL':<20} {' ':>9} {sum(s[2] for s in segs):>8.0f} {E_used:>11.2f}")
    margin  = E_usable - E_used
    SoC_end = (margin / E_total) * 100

    cruise_range = V_cruise * t_cruise / 1000.0   # km
    t_charge_h   = C_bat / I_charge * 1.2          # hours (CC-CV)

    sub("Results")
    val("Total battery energy",     E_total,       "Wh")
    val("Usable energy",            E_usable,      "Wh")
    val("Mission energy used",      E_used,        "Wh")
    val("Energy margin",            margin,        "Wh")
    val("SoC at end of mission",    SoC_end,       "%")
    val("Cruise range",             cruise_range,  "km")
    val("Charge time (CC-CV est.)", t_charge_h*60, "min")
    sep()
    if margin < 0:
        print("  ✗  MISSION NOT FEASIBLE — insufficient battery energy!")
    elif SoC_end < 20:
        print("  ⚠  Less than 20% SoC remaining — within reserve boundary.")
    else:
        print("  ✓  Mission feasible with adequate energy margin.")

# ─── 11. TILT-ROTOR TRANSITION ────────────────────────────────────────────────

def tiltrotor_transition():
    """
    eVTOL Tilt-Rotor Transition Analysis
    ──────────────────────────────────────
    Unique to tilt-rotor: analyses the conversion corridor
    (range of tilt angles and speeds for which the aircraft can fly safely).

    KEY FORMULAE & ASSUMPTIONS
    • Rotor tilt angle θ_t (0° = hover, 90° = cruise):
        T_vertical   = T_total × cos(θ_t)  ≥ W − L_wing
        T_horizontal = T_total × sin(θ_t)  ≥ D_aircraft
        L_wing       = 0.5 × ρ × V² × S × CL_wing

    • Minimum conversion speed (rotor stall avoidance):
        RPM must stay above stall: V_tip / (Ω × R) < 0.7  (advance ratio μ < 0.7)
        μ = V × cos(θ_t) / (Ω × R)

    • Power during transition (simplified):
        P_trans = (T_total × v_i) / FM + D × V
        v_i varies with tilt (edgewise flow reduces inflow).

    ASSUMPTIONS:
    – Wings produce lift only from airspeed (no contribution from prop wake).
    – No rotor flapping model — simplified rigid rotor.
    – Power available constant = max motor power.
    – Transition is symmetric (both rotors tilt together).
    """
    header("11. eVTOL Tilt-Rotor Transition Corridor")

    sub("Aircraft")
    MTOW    = ask("MTOW (kg)",                          default=25.0)
    S       = ask("Wing area (m²)",                     default=1.20)
    AR      = ask("Wing aspect ratio",                  default=8.0)
    CD0     = ask("Zero-lift drag CD0",                 default=0.025)
    e_osw   = ask("Oswald efficiency",                  default=0.80)
    CL_max  = ask("Wing CLmax",                         default=1.4)

    sub("Rotor (per rotor, 2 tilting rotors assumed)")
    R       = ask("Rotor radius (m)",                   default=0.40)
    P_max_r = ask("Max power per rotor (W)",            default=3000.0)
    FM      = ask("Figure of merit",                    default=0.65)
    RPM_nom = ask("Nominal hover RPM",                  default=3000.0)

    sub("Environment")
    alt_m   = ask("Altitude (m)",                       default=0.0)
    n_tilt  = ask_int("Number of tilting rotors",       default=2)

    rho     = isa_density(alt_m)
    W       = MTOW * G
    A       = math.pi * R**2
    Omega   = rpm_to_rads(RPM_nom)
    V_tip   = Omega * R
    P_avail = P_max_r * n_tilt

    sub("Transition Corridor (speed vs tilt angle)")
    print(f"\n  {'Speed(m/s)':>11} {'Tilt(°)':>8} {'L_wing(N)':>10} "
          f"{'T_req(N)':>9} {'P_req(W)':>9} {'μ':>6} {'Feas':>5}")
    print("  " + "─" * 70)

    speeds = [0, 5, 10, 15, 20, 25, 30]
    for V in speeds:
        q = 0.5 * rho * V**2
        # Find CL for level flight:
        # L_wing = q*S*CL,  L_wing + T*cos(tilt) = W
        # Minimize tilt angle subject to T <= T_max from power
        # Approach: sweep tilt angles
        best_tilt = None
        for tilt_deg in range(0, 91, 5):
            tilt = math.radians(tilt_deg)
            # Lift from wing:
            if V < 1:
                CL_wing = 0; L_wing = 0
            else:
                CL_wing = min(W / max(q*S, 0.001), CL_max)
                L_wing  = q * S * CL_wing
            # Thrust needed:
            T_vert = max(W - L_wing, 0)
            T_horiz= q * S * (CD0 + CL_wing**2/(math.pi*e_osw*AR))
            T_need = math.sqrt(T_vert**2 + T_horiz**2) if tilt > 0.01 else T_vert
            # Check if tilt geometry satisfies:
            if tilt > 0.01:
                T_from_tilt_v = T_need * math.cos(tilt)
                T_from_tilt_h = T_need * math.sin(tilt)
            else:
                T_from_tilt_v = T_need
                T_from_tilt_h = 0
            # Power:
            v_i_r = math.sqrt(T_need / n_tilt / (2*rho*A))
            P_req  = T_need * v_i_r / FM + T_horiz * V
            mu     = V * math.cos(tilt) / max(V_tip, 1)
            feasible = (P_req <= P_avail) and (mu < 0.70) and (T_need/n_tilt <= 2*rho*A*V_tip**2)
            if feasible and best_tilt is None:
                best_tilt = (tilt_deg, L_wing, T_need, P_req, mu)
                break

        if best_tilt:
            td, Lw, Tn, Pr, mu = best_tilt
            print(f"  {V:>11.1f} {td:>8.0f} {Lw:>10.1f} {Tn:>9.1f} {Pr:>9.0f} {mu:>6.3f}  ✓")
        else:
            print(f"  {V:>11.1f} {'N/A':>8}  — no feasible tilt angle found —")

    print("\n  Note: Minimum tilt shown for each speed. μ < 0.7 required (rotor stall limit).")

# ─── MAIN MENU ────────────────────────────────────────────────────────────────

MENU = [
    ("propCalc",        "Motor + Propeller + Battery simulation",           prop_calc),
    ("xcopterCalc",     "Multirotor / eVTOL hover performance",             xcopter_calc),
    ("heliCalc",        "Single main rotor / tilt-rotor hover",             heli_calc),
    ("bladeCalc",       "Blade element momentum theory (BEMT)",             blade_calc),
    ("perfCalc",        "Fixed-wing / cruise performance",                  perf_calc),
    ("cgCalc",          "Center of gravity & static margin",                cg_calc),
    ("w&bCalc",         "Weight & balance envelope",                        wb_calc),
    ("fanCalc",         "Electric ducted fan (EDF) performance",            fan_calc),
    ("torqueCalc",      "Motor torque & drivetrain power",                  torque_calc),
    ("evCalc",          "Battery energy & mission planner",                 ev_calc),
    ("tiltrotorCalc",   "eVTOL tilt-rotor transition corridor",             tiltrotor_transition),
]

def main():
    print("""
╔══════════════════════════════════════════════════════════════╗
║        eVTOL Calc — Electric Drive & Aircraft Design         ║
║        Equivalent to eCalc.ch — Python Terminal Edition      ║
║        Designed for eVTOL Tilt-Rotor Development            ║
╚══════════════════════════════════════════════════════════════╝
""")
    while True:
        print("\n  CALCULATORS")
        print("  " + "─" * 58)
        for i, (name, desc, _) in enumerate(MENU, 1):
            print(f"  {i:>2}. {name:<18} {desc}")
        print("   0. Exit")
        print()
        choice = input("  Select calculator [0–11]: ").strip()
        if choice == "0":
            print("\n  Goodbye.\n")
            sys.exit(0)
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(MENU):
                try:
                    MENU[idx][2]()
                except (KeyboardInterrupt, EOFError):
                    print("\n  (Cancelled)")
            else:
                print("  Invalid selection.")
        except ValueError:
            print("  Please enter a number.")

if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print("\n\n  Exiting.\n")
