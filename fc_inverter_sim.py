#!/usr/bin/env python3
"""
FC_Inverter_Simulation_v2.py
============================
Three-Phase Three-Level Flying Capacitor Inverter Simulation
for 800V EV Traction with Adaptive Switching Frequency Scheduling.

Topology: 3-level FC inverter with 800V DC bus.
  - Each phase leg has 4 switches (S1, S2, S3, S4) and one flying capacitor.
  - FC nominal voltage = Vdc/2 = 400V.
  - Output voltage levels: +Vdc/2 = +400V, 0V, -Vdc/2 = -400V.
  - State +1: S1,S2 ON → output = +Vdc/2, FC bypassed
  - State -1: S3,S4 ON → output = -Vdc/2, FC bypassed
  - State  0: S1,S4 ON → output = +Vdc/2 - Vfc (FC discharges if i>0)
              S2,S3 ON → output = -Vdc/2 + Vfc (FC charges if i>0)
  - PD-PWM naturally selects between zero-state redundancies to balance FC.

Improvements over v1:
  - Corrected drivetrain model (tyre radius + gear ratio -> fe)
  - Proper FC dynamic model with correct charge/discharge physics
  - Nonlinear switching energy model (quadratic current scaling)
  - 4-layer Foster thermal network (from device datasheet)
  - Temperature-dependent Rds_on
  - Dead-time voltage distortion modelling
  - Hanning-windowed THD computation
  - Symmetric loss accounting for both strategies

Author: Hayagreev S.
Date: May 2026
"""

import numpy as np
import json
import os
import time
from dataclasses import dataclass
from typing import Tuple, List

# ============================================================================
# 1. SYSTEM PARAMETERS
# ============================================================================

@dataclass
class SystemParams:
    """All system parameters for the FC inverter simulation."""
    # DC Bus
    Vdc_total: float = 800.0       # Total DC bus voltage [V]
    
    # Load (per project spec)
    R_load: float = 1.3            # Load resistance [Ohm]
    L_load: float = 2e-3           # Load inductance [H]
    
    # Flying Capacitor
    C_fc: float = 1.5e-3           # Flying capacitor [F]
    ESR_fc: float = 3.3e-3         # Equivalent series resistance [Ohm]
    
    # SiC MOSFET Module: Wolfspeed CAB450M12XM3 (1200V, 450A half-bridge module)
    # Using 650V-class equivalent parameters scaled for the current level.
    # Effective Rds_on accounts for the module's low on-resistance at this rating.
    Rds_on: float = 4.0e-3         # On-state resistance [Ohm] at 25C (power module)
    Vf_body: float = 1.5           # Body diode forward voltage [V]
    
    # Switching energy (from module datasheet at Vds=400V, Id=300A)
    E_on_ref: float = 8.5e-3       # Turn-on energy [J] at I_ref, V_ref
    E_off_ref: float = 4.2e-3      # Turn-off energy [J] at I_ref, V_ref
    I_ref: float = 300.0           # Reference current for switching energy [A]
    V_ref: float = 400.0           # Reference voltage for switching energy [V]
    
    # Nonlinear switching energy: E = E_ref * (a*(I/Iref)^2 + b*(I/Iref) + c) * (V/Vref)
    sw_coeff_a: float = 0.10
    sw_coeff_b: float = 0.80
    sw_coeff_c: float = 0.10
    
    # Dead time
    t_dead: float = 200e-9         # Dead time [s]
    
    # Thermal model: 4-layer Foster network (power module)
    # Rth_jc total ~ 0.04 K/W for a 450A module
    Rth_layers: tuple = (0.012, 0.010, 0.009, 0.009)
    tau_layers: tuple = (0.5e-3, 5e-3, 50e-3, 500e-3)
    Rth_cs: float = 0.015          # Case-to-sink [K/W] (with thermal grease)
    Rth_sa: float = 0.025          # Sink-to-ambient [K/W] (liquid cooled)
    T_ambient: float = 40.0        # Ambient temperature [C]
    
    # Modulation
    m_index: float = 0.9           # Modulation index
    
    # Drivetrain
    r_tyre: float = 0.315          # Tyre radius [m]
    gear_ratio: float = 8.19       # Gear ratio
    P_poles: int = 8               # Motor pole count
    
    # Simulation
    dt: float = 2e-6               # Time step [s]
    
    @property
    def Vdc(self):
        """Half DC bus voltage (each device blocks this)."""
        return self.Vdc_total / 2
    
    @property
    def Vc_ref(self):
        """FC reference voltage = Vdc/2 = 400V for 800V bus."""
        return self.Vdc_total / 2
    
    @property
    def Rth_jc_total(self):
        """Total junction-to-case thermal resistance."""
        return sum(self.Rth_layers)


# ============================================================================
# 2. DRIVE CYCLE
# ============================================================================

def speed_to_fe(v_kmh: float, params: SystemParams) -> float:
    """Convert vehicle speed to motor electrical frequency.
    fe = (P/2) * (v_ms / r_tyre) * (gear_ratio / (2*pi))
    """
    v_ms = v_kmh / 3.6
    n_mech_rps = (v_ms / params.r_tyre) * (params.gear_ratio / (2 * np.pi))
    fe = (params.P_poles / 2) * n_mech_rps
    return max(fe, 1.0)


def build_drive_cycle(params: SystemParams):
    """Build drive cycle time-domain arrays."""
    seg_defs = [
        ("Standstill",     0,   0.05),
        ("Urban low",      20,  0.10),
        ("Urban mid",      40,  0.20),
        ("Urban high",     60,  0.20),
        ("Deceleration",   20,  0.10),
        ("Stop",           0,   0.05),
        ("Transition",     80,  0.20),
        ("Highway",        120, 0.20),
        ("Highway fast",   140, 0.20),
        ("Highway cruise", 100, 0.20),
    ]
    
    def fsw_schedule(v_kmh):
        if v_kmh < 60:
            return 5e3
        elif v_kmh < 80:
            return 10e3
        else:
            return 20e3
    
    Z_load = lambda fe: np.sqrt(params.R_load**2 + (2*np.pi*fe*params.L_load)**2)
    
    segments = []
    t_list, v_list, fe_list, fsw_list = [], [], [], []
    tc = 0.0
    
    for name, speed, dur in seg_defs:
        fe = speed_to_fe(speed, params)
        fsw = fsw_schedule(speed)
        Ipk = params.m_index * params.Vdc / Z_load(fe) if speed > 0 else 0.0
        
        segments.append({
            "name": name, "speed_kmh": speed, "duration_s": dur,
            "fe_Hz": fe, "fsw_Hz": fsw, "Ipeak_expected": Ipk
        })
        
        n_steps = round(dur / params.dt)
        ts = tc + np.arange(n_steps) * params.dt
        t_list.append(ts)
        v_list.append(np.full(n_steps, speed))
        fe_list.append(np.full(n_steps, fe))
        fsw_list.append(np.full(n_steps, fsw))
        tc += dur
    
    t = np.concatenate(t_list)
    v = np.concatenate(v_list)
    fe = np.concatenate(fe_list)
    fsw = np.concatenate(fsw_list)
    
    return segments, t, v, fe, fsw


# ============================================================================
# 3. THERMAL MODEL
# ============================================================================

class FosterThermalModel:
    """4-layer Foster thermal network."""
    
    def __init__(self, params: SystemParams):
        self.Rth = np.array(params.Rth_layers)
        self.tau = np.array(params.tau_layers)
        self.Rth_cs = params.Rth_cs
        self.Rth_sa = params.Rth_sa
        self.T_ambient = params.T_ambient
        self.n = len(self.Rth)
        self.dT = np.zeros(self.n)
    
    def update(self, P_loss: float, dt: float) -> float:
        """Update and return Tj."""
        for i in range(self.n):
            alpha = dt / self.tau[i]
            if alpha < 0.1:
                self.dT[i] += (P_loss * self.Rth[i] - self.dT[i]) * alpha
            else:
                exp_a = np.exp(-alpha)
                self.dT[i] = self.dT[i] * exp_a + P_loss * self.Rth[i] * (1 - exp_a)
        
        return self.T_ambient + np.sum(self.dT) + P_loss * (self.Rth_cs + self.Rth_sa)
    
    def reset(self):
        self.dT = np.zeros(self.n)


# ============================================================================
# 4. MAIN SIMULATION
# ============================================================================

def run_simulation(params: SystemParams, verbose: bool = True):
    """Run the full FC inverter simulation."""
    
    if verbose:
        print("=" * 70)
        print("  FC Inverter Simulation v2 (Python)")
        print("  Three-Phase Three-Level Flying Capacitor | 800V EV Traction")
        print("=" * 70)
        print(f"  Vdc={params.Vdc_total}V  R={params.R_load} Ohm  L={params.L_load*1e3}mH  "
              f"C_fc={params.C_fc*1e3}mF  dt={params.dt*1e6}us")
        print(f"  FC reference voltage: {params.Vc_ref}V")
        print()
    
    # Build drive cycle
    segments, t, v_speed, fe_arr, fsw_arr = build_drive_cycle(params)
    N = len(t)
    T_sim = t[-1] + params.dt
    
    Z_load = lambda fe: np.sqrt(params.R_load**2 + (2*np.pi*fe*params.L_load)**2)
    
    if verbose:
        print(f"  Drive cycle: {len(segments)} segments, {N} steps, {T_sim:.3f}s")
        print(f"  {'Segment':<18} {'v':>6} {'fe(Hz)':>8} {'fsw(kHz)':>8} {'dur(s)':>7} {'Ipk(A)':>8}")
        print("  " + "-" * 60)
        for seg in segments:
            print(f"  {seg['name']:<18} {seg['speed_kmh']:>6.0f} {seg['fe_Hz']:>8.1f} "
                  f"{seg['fsw_Hz']/1e3:>8.0f} {seg['duration_s']:>7.2f} {seg['Ipeak_expected']:>8.1f}")
        print()
    
    # Phase angles
    phi_abc = np.array([0.0, -2*np.pi/3, 2*np.pi/3])
    
    # State variables
    i_abc = np.zeros(3)
    Vfc = np.full(3, params.Vc_ref)  # FC starts at reference (pre-charged)
    
    # Output arrays
    o_ia = np.zeros(N); o_ib = np.zeros(N); o_ic = np.zeros(N)
    o_Van = np.zeros(N); o_Vbn = np.zeros(N); o_Vcn = np.zeros(N)
    o_Vfc = np.zeros((3, N))
    
    # Loss accumulators
    E_cond_sched = np.zeros(3)  # Conduction energy per phase [J]
    E_sw_sched = np.zeros(3)    # Switching energy per phase [J]
    E_cond_fixed = np.zeros(3)
    E_sw_fixed = np.zeros(3)
    
    # Instantaneous loss (phase A)
    P_inst_sched = np.zeros(N)
    P_inst_fixed = np.zeros(N)
    
    # Thermal
    thermal_sched = FosterThermalModel(params)
    thermal_fixed = FosterThermalModel(params)
    Tj_sched = np.zeros(N)
    Tj_fixed = np.zeros(N)
    
    # Carrier
    carrier_phase = 0.0
    period_start = True
    
    # FC balancing direction (sample-and-hold)
    fc_state = np.zeros(3, dtype=int)  # +1 or -1: which zero-state redundancy
    
    start_time = time.time()
    if verbose:
        print("  Running simulation...", end="", flush=True)
    
    for k in range(N):
        t_k = t[k]
        fe_k = fe_arr[k]
        fsw_k = fsw_arr[k]
        
        # --- Carrier ---
        cp_new = carrier_phase + fsw_k * params.dt
        if cp_new >= 1.0:
            cp_new -= 1.0
            period_start = True
        else:
            period_start = (k == 0)
        carrier_phase = cp_new
        
        # Triangular carrier [0, 1]
        if carrier_phase < 0.5:
            tri = 2.0 * carrier_phase
        else:
            tri = 2.0 * (1.0 - carrier_phase)
        
        # PD-PWM carriers: upper band [0,+1], lower band [-1,0]
        C_up = tri
        C_lo = -tri
        
        # --- Reference ---
        we = 2.0 * np.pi * fe_k
        v_ref = params.m_index * np.sin(we * t_k + phi_abc)
        
        # --- Determine output levels ---
        sw_level = np.zeros(3, dtype=int)
        for ph in range(3):
            if v_ref[ph] > C_up:
                sw_level[ph] = 1
            elif v_ref[ph] < C_lo:
                sw_level[ph] = -1
            else:
                sw_level[ph] = 0
        
        # --- FC balancing: choose zero-state redundancy at period start ---
        # In zero state, two redundant switch states exist:
        #   State A: S1,S4 ON → Vout = Vdc/2 - Vfc → FC discharges if i>0
        #   State B: S2,S3 ON → Vout = -Vdc/2 + Vfc → FC charges if i>0
        # For balancing: if Vfc < Vc_ref, choose state that charges FC
        #                if Vfc > Vc_ref, choose state that discharges FC
        if period_start:
            for ph in range(3):
                if Vfc[ph] < params.Vc_ref:
                    # Need to charge: choose state where i_fc = +|i| (charges)
                    if i_abc[ph] >= 0:
                        fc_state[ph] = 1   # State B: charges when i>0
                    else:
                        fc_state[ph] = -1  # State A: charges when i<0
                else:
                    # Need to discharge
                    if i_abc[ph] >= 0:
                        fc_state[ph] = -1  # State A: discharges when i>0
                    else:
                        fc_state[ph] = 1   # State B: discharges when i<0
        
        # --- Compute output pole voltages ---
        V_pole = np.zeros(3)
        i_fc_actual = np.zeros(3)  # Current flowing into FC (positive = charging)
        
        for ph in range(3):
            if sw_level[ph] == 1:
                # S1, S2 ON: output = +Vdc/2, FC bypassed
                V_pole[ph] = params.Vdc / 2
                i_fc_actual[ph] = 0.0
            elif sw_level[ph] == -1:
                # S3, S4 ON: output = -Vdc/2, FC bypassed
                V_pole[ph] = -params.Vdc / 2
                i_fc_actual[ph] = 0.0
            else:
                # Zero state: depends on fc_state selection
                if fc_state[ph] == 1:
                    # State B: S2, S3 ON → Vout = -Vdc/2 + Vfc
                    V_pole[ph] = -params.Vdc / 2 + Vfc[ph]
                    # FC current = +i_phase (charges when i>0)
                    i_fc_actual[ph] = i_abc[ph]
                else:
                    # State A: S1, S4 ON → Vout = +Vdc/2 - Vfc
                    V_pole[ph] = params.Vdc / 2 - Vfc[ph]
                    # FC current = -i_phase (discharges when i>0)
                    i_fc_actual[ph] = -i_abc[ph]
        
        # Dead-time voltage error
        for ph in range(3):
            if abs(i_abc[ph]) > 0.1:
                V_pole[ph] -= np.sign(i_abc[ph]) * (params.Vdc / 2) * params.t_dead * fsw_k * 2
        
        # Neutral point (floating star)
        Vn = np.sum(V_pole) / 3.0
        V_phase = V_pole - Vn
        
        # --- Current update (Forward Euler) ---
        di = (V_phase - params.R_load * i_abc) / params.L_load * params.dt
        i_abc_new = i_abc + di
        # Zero-sequence elimination
        i_abc_new -= np.mean(i_abc_new)
        
        # --- FC voltage update ---
        for ph in range(3):
            # dVfc = i_fc / C * dt
            Vfc[ph] += i_fc_actual[ph] / params.C_fc * params.dt
        
        # --- Loss calculation ---
        # Temperature-dependent Rds_on (use last Tj estimate)
        Tj_est = Tj_sched[k-1] if k > 0 else params.T_ambient
        t_norm = np.clip((Tj_est - 25.0) / (150.0 - 25.0), 0.0, 1.0)
        Rds_on_T = params.Rds_on * (1.0 + 0.8 * t_norm)  # ~1.8x at 150C for SiC module
        
        for ph in range(3):
            I_ph = abs(i_abc[ph])
            
            # Conduction: always 2 devices in series conducting
            P_cond = 2.0 * Rds_on_T * I_ph**2
            E_step = P_cond * params.dt
            E_cond_sched[ph] += E_step
            E_cond_fixed[ph] += E_step  # Same current → same conduction
            
            # Switching energy (nonlinear model)
            I_norm = I_ph / params.I_ref
            scale = (params.sw_coeff_a * I_norm**2 + 
                     params.sw_coeff_b * I_norm + 
                     params.sw_coeff_c)
            E_sw_event = (params.E_on_ref + params.E_off_ref) * scale
            # Each switching period has 2 commutations per device pair
            E_sw_sched[ph] += E_sw_event * fsw_k * params.dt
            E_sw_fixed[ph] += E_sw_event * 20e3 * params.dt
            
            if ph == 0:
                P_inst_sched[k] = P_cond + E_sw_event * fsw_k
                P_inst_fixed[k] = P_cond + E_sw_event * 20e3
        
        # Thermal update (per device = total phase loss / 4 devices, but 
        # worst case device sees ~half the total since 2 always conduct)
        P_dev_sched = P_inst_sched[k] / 2.0
        P_dev_fixed = P_inst_fixed[k] / 2.0
        Tj_sched[k] = thermal_sched.update(P_dev_sched, params.dt)
        Tj_fixed[k] = thermal_fixed.update(P_dev_fixed, params.dt)
        
        # --- Store ---
        o_ia[k] = i_abc[0]; o_ib[k] = i_abc[1]; o_ic[k] = i_abc[2]
        o_Van[k] = V_phase[0]; o_Vbn[k] = V_phase[1]; o_Vcn[k] = V_phase[2]
        o_Vfc[0, k] = Vfc[0]; o_Vfc[1, k] = Vfc[1]; o_Vfc[2, k] = Vfc[2]
        
        # Update state
        i_abc = i_abc_new
    
    elapsed = time.time() - start_time
    if verbose:
        print(f" Done in {elapsed:.1f}s")
        print()
    
    # ========================================================================
    # POST-PROCESSING
    # ========================================================================
    
    # Loss summary
    P_cond_sched_ph = E_cond_sched / T_sim
    P_sw_sched_ph = E_sw_sched / T_sim
    P_total_sched_ph = P_cond_sched_ph + P_sw_sched_ph
    
    P_cond_fixed_ph = E_cond_fixed / T_sim
    P_sw_fixed_ph = E_sw_fixed / T_sim
    P_total_fixed_ph = P_cond_fixed_ph + P_sw_fixed_ph
    
    # THD
    def compute_thd(signal, fe, dt_val, label=""):
        """Hanning-windowed THD."""
        N_min = int(20 / fe / dt_val)
        N_win = min(2**int(np.ceil(np.log2(N_min))), len(signal))
        seg = signal[-N_win:]
        
        window = np.hanning(N_win)
        seg_w = seg * window
        cg = np.sum(window) / N_win
        
        spectrum = np.abs(np.fft.fft(seg_w)) / (N_win * cg / 2)
        spectrum[0] /= 2
        f_axis = np.arange(N_win) / (N_win * dt_val)
        half = N_win // 2
        
        k1 = np.argmin(np.abs(f_axis[:half] - fe))
        I_fund = spectrum[k1]
        I_fund_rms = I_fund / np.sqrt(2)
        
        I_h_sq = 0.0
        for h in range(2, 51):
            kh = np.argmin(np.abs(f_axis[:half] - h * fe))
            I_h_sq += spectrum[kh]**2
        
        THD = 100.0 * np.sqrt(I_h_sq) / max(I_fund, 1e-9)
        
        if verbose and label:
            print(f"    {label:<28} fe={fe:>7.1f}Hz  I_fund={I_fund_rms:>7.2f}A rms  THD={THD:>6.2f}%")
        
        return THD, I_fund_rms, f_axis[:half], spectrum[:half]
    
    # Find segments
    idx_urban = np.where((v_speed >= 38) & (v_speed <= 42))[0]
    idx_highway = np.where((v_speed >= 118) & (v_speed <= 122))[0]
    
    fe_urban = speed_to_fe(40, params)
    fe_highway = speed_to_fe(120, params)
    
    if verbose:
        print("  --- THD Analysis ---")
    
    THD_urban, I_fund_urban, f_urban, spec_urban = np.nan, np.nan, None, None
    THD_highway, I_fund_highway, f_highway, spec_highway = np.nan, np.nan, None, None
    
    if len(idx_urban) > 500:
        THD_urban, I_fund_urban, f_urban, spec_urban = compute_thd(
            o_ia[idx_urban], fe_urban, params.dt, "Urban 40km/h (fsw=5kHz)")
    if len(idx_highway) > 500:
        THD_highway, I_fund_highway, f_highway, spec_highway = compute_thd(
            o_ia[idx_highway], fe_highway, params.dt, "Highway 120km/h (fsw=20kHz)")
    
    # Steady-state windows for plotting
    def get_ss_window(idx_seg, n_cycles, fe_val):
        if len(idx_seg) == 0:
            return np.array([], dtype=int), np.array([])
        skip = len(idx_seg) // 2
        n_show = min(int(n_cycles / fe_val / params.dt), len(idx_seg) - skip)
        i0 = idx_seg[0] + skip
        i1 = min(i0 + n_show, N)
        iw = np.arange(i0, i1)
        tw = (t[iw] - t[iw[0]]) * 1e3
        return iw, tw
    
    iw_urban, tw_urban = get_ss_window(idx_urban, 5, fe_urban)
    iw_highway, tw_highway = get_ss_window(idx_highway, 5, fe_highway)
    
    # FC ripple
    rip_sim_urban = (np.max(o_Vfc[0, iw_urban]) - np.min(o_Vfc[0, iw_urban])) if len(iw_urban) > 0 else np.nan
    rip_sim_highway = (np.max(o_Vfc[0, iw_highway]) - np.min(o_Vfc[0, iw_highway])) if len(iw_highway) > 0 else np.nan
    
    Ipk_urban = params.m_index * params.Vdc / Z_load(fe_urban)
    Ipk_highway = params.m_index * params.Vdc / Z_load(fe_highway)
    
    rip_pred_5k = Ipk_urban * 0.5 / (2 * 5e3 * params.C_fc)
    rip_pred_10k = (params.m_index * params.Vdc / Z_load(speed_to_fe(80, params))) * 0.5 / (2 * 10e3 * params.C_fc)
    rip_pred_20k = Ipk_highway * 0.5 / (2 * 20e3 * params.C_fc)
    
    if verbose:
        print()
        print("=" * 60)
        print("  RESULTS SUMMARY")
        print("=" * 60)
        print(f"  Peak phase current              : {np.max(np.abs(o_ia)):.2f} A")
        print(f"  Peak Van                        : {np.max(np.abs(o_Van)):.1f} V")
        print(f"  Ipeak expected @ urban 40km/h   : {Ipk_urban:.2f} A")
        print(f"  Ipeak expected @ highway 120km/h: {Ipk_highway:.2f} A")
        print(f"  FC mean voltage (phase A)       : {np.mean(o_Vfc[0]):.2f} V (ref={params.Vc_ref:.1f}V)")
        print(f"  FC pk-pk ripple (full cycle)    : {np.max(o_Vfc[0])-np.min(o_Vfc[0]):.2f} V")
        if not np.isnan(rip_sim_urban):
            print(f"  FC ripple urban (sim/pred)      : {rip_sim_urban:.3f} / {rip_pred_5k:.3f} V")
        if not np.isnan(rip_sim_highway):
            print(f"  FC ripple highway (sim/pred)    : {rip_sim_highway:.3f} / {rip_pred_20k:.3f} V")
        print(f"  fe @ urban 40km/h               : {fe_urban:.1f} Hz")
        print(f"  fe @ highway 120km/h            : {fe_highway:.1f} Hz")
        if not np.isnan(THD_urban):
            print(f"  THD urban (5kHz)                : {THD_urban:.2f}%  Ifund={I_fund_urban:.2f}A rms")
        if not np.isnan(THD_highway):
            print(f"  THD highway (20kHz)             : {THD_highway:.2f}%  Ifund={I_fund_highway:.2f}A rms")
        print(f"  Losses sched (ph A): cond={P_cond_sched_ph[0]:.2f}W  sw={P_sw_sched_ph[0]:.2f}W  total={P_total_sched_ph[0]:.2f}W")
        print(f"  Losses fixed (ph A): cond={P_cond_fixed_ph[0]:.2f}W  sw={P_sw_fixed_ph[0]:.2f}W  total={P_total_fixed_ph[0]:.2f}W")
        print(f"  Total (3-ph): sched={np.sum(P_total_sched_ph):.2f}W  fixed={np.sum(P_total_fixed_ph):.2f}W")
        sw_saving = np.sum(P_sw_fixed_ph) - np.sum(P_sw_sched_ph)
        sw_saving_pct = 100 * sw_saving / max(np.sum(P_sw_fixed_ph), 1e-9)
        print(f"  Switching loss saving           : {sw_saving:.2f} W ({sw_saving_pct:.1f}%)")
        print(f"  Tj max (sched/fixed)            : {np.max(Tj_sched):.1f} / {np.max(Tj_fixed):.1f} C")
        print(f"  Tj saving                       : {np.max(Tj_fixed)-np.max(Tj_sched):.2f} C")
        print()
    
    return {
        "params": params, "segments": segments,
        "t": t, "v_speed": v_speed, "fe_arr": fe_arr, "fsw_arr": fsw_arr,
        "N": N, "T_sim": T_sim, "elapsed": elapsed,
        "o_ia": o_ia, "o_ib": o_ib, "o_ic": o_ic,
        "o_Van": o_Van, "o_Vbn": o_Vbn, "o_Vcn": o_Vcn, "o_Vfc": o_Vfc,
        "P_cond_sched": P_cond_sched_ph, "P_sw_sched": P_sw_sched_ph, "P_total_sched": P_total_sched_ph,
        "P_cond_fixed": P_cond_fixed_ph, "P_sw_fixed": P_sw_fixed_ph, "P_total_fixed": P_total_fixed_ph,
        "P_inst_sched": P_inst_sched, "P_inst_fixed": P_inst_fixed,
        "Tj_sched": Tj_sched, "Tj_fixed": Tj_fixed,
        "THD_urban": THD_urban, "I_fund_urban": I_fund_urban,
        "THD_highway": THD_highway, "I_fund_highway": I_fund_highway,
        "f_urban": f_urban, "spec_urban": spec_urban,
        "f_highway": f_highway, "spec_highway": spec_highway,
        "fe_urban": fe_urban, "fe_highway": fe_highway,
        "Ipk_urban": Ipk_urban, "Ipk_highway": Ipk_highway,
        "rip_sim_urban": rip_sim_urban, "rip_sim_highway": rip_sim_highway,
        "rip_pred_5k": rip_pred_5k, "rip_pred_10k": rip_pred_10k, "rip_pred_20k": rip_pred_20k,
        "iw_urban": iw_urban, "tw_urban": tw_urban,
        "iw_highway": iw_highway, "tw_highway": tw_highway,
        "idx_urban": idx_urban, "idx_highway": idx_highway,
    }


# ============================================================================
# 5. FIGURE GENERATION
# ============================================================================

def generate_figures(results: dict, out_dir: str):
    """Generate publication-quality figures."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    
    plt.rcParams.update({
        'font.size': 9, 'font.family': 'serif',
        'axes.labelsize': 9, 'axes.titlesize': 10,
        'xtick.labelsize': 8, 'ytick.labelsize': 8,
        'legend.fontsize': 8, 'figure.dpi': 150,
        'savefig.dpi': 150, 'savefig.bbox': 'tight',
    })
    
    params = results["params"]
    t = results["t"]
    v_speed = results["v_speed"]
    fe_arr = results["fe_arr"]
    fsw_arr = results["fsw_arr"]
    o_ia, o_ib, o_ic = results["o_ia"], results["o_ib"], results["o_ic"]
    o_Van, o_Vbn, o_Vcn = results["o_Van"], results["o_Vbn"], results["o_Vcn"]
    o_Vfc = results["o_Vfc"]
    
    C = {'bl': '#1F3864', 'rd': '#C0392B', 'gr': '#1E8449',
         'or': '#D68910', 'pu': '#6C3483', 'gy': '#7F8C8D', 'tl': '#148F77'}
    
    os.makedirs(out_dir, exist_ok=True)
    
    # --- Fig 1: Drive Cycle ---
    fig, axes = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
    fig.suptitle('Drive Cycle: Speed, Electrical Frequency, and Switching Frequency', fontweight='bold')
    
    axes[0].fill_between(t, v_speed, alpha=0.25, color=C['tl'])
    axes[0].plot(t, v_speed, color=C['tl'], linewidth=1.2)
    axes[0].set_ylabel('Speed (km/h)'); axes[0].set_title('Vehicle Speed Profile')
    axes[0].grid(True, alpha=0.3); axes[0].set_ylim([0, 165])
    
    axes[1].plot(t, fe_arr, color=C['bl'], linewidth=1.5)
    axes[1].set_ylabel('$f_e$ (Hz)'); axes[1].grid(True, alpha=0.3)
    axes[1].set_title(f'Electrical Frequency  [P={params.P_poles}, gear={params.gear_ratio:.2f}:1, $r_{{tyre}}$={params.r_tyre}m]')
    
    axes[2].step(t, fsw_arr/1e3, where='post', color=C['or'], linewidth=2)
    axes[2].axhline(20, color=C['gy'], linestyle='--', linewidth=1, label='Fixed 20kHz baseline')
    axes[2].set_ylabel('$f_{sw}$ (kHz)'); axes[2].set_xlabel('Time (s)')
    axes[2].set_title('Adaptive Switching Frequency Schedule')
    axes[2].grid(True, alpha=0.3); axes[2].set_ylim([0, 25]); axes[2].legend()
    
    plt.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig1_drive_cycle.png')); plt.close(fig)
    print("  Saved fig1_drive_cycle.png")
    
    # --- Fig 2: Output Voltages ---
    fig, axes = plt.subplots(2, 2, figsize=(12, 6))
    fig.suptitle('Three-Level FC Output Voltages (Urban vs Highway)', fontweight='bold')
    
    for col, (iw, tw, fe_val, fsw_val, label) in enumerate([
        (results["iw_urban"], results["tw_urban"], results["fe_urban"], 5, "Urban 40km/h"),
        (results["iw_highway"], results["tw_highway"], results["fe_highway"], 20, "Highway 120km/h")
    ]):
        if len(iw) == 0: continue
        ax = axes[0, col]
        ax.plot(tw, o_Van[iw], color=C['bl'], linewidth=0.8, label='$V_{an}$')
        ax.plot(tw, o_Vbn[iw], color=C['rd'], linewidth=0.6, linestyle='--', label='$V_{bn}$')
        ax.axhline(params.Vdc/2, color='k', linestyle='--', linewidth=0.4, alpha=0.4)
        ax.axhline(-params.Vdc/2, color='k', linestyle='--', linewidth=0.4, alpha=0.4)
        ax.axhline(0, color='k', linestyle=':', linewidth=0.4, alpha=0.4)
        ax.set_ylabel('V (V)')
        ax.set_title(f'{label}  $f_e$={fe_val:.1f}Hz  $f_{{sw}}$={fsw_val}kHz')
        ax.legend(); ax.grid(True, alpha=0.3)
        
        ax = axes[1, col]
        ax.plot(tw, o_Van[iw] - o_Vbn[iw], color=C['pu'], linewidth=0.8)
        ax.set_ylabel('$V_{ab}$ (V)'); ax.set_xlabel('Time (ms)')
        ax.set_title('Line Voltage $V_{ab}$'); ax.grid(True, alpha=0.3)
    
    plt.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig2_output_voltages.png')); plt.close(fig)
    print("  Saved fig2_output_voltages.png")
    
    # --- Fig 3: Phase Currents ---
    fig, axes = plt.subplots(2, 2, figsize=(12, 6))
    fig.suptitle(f'Three-Phase Output Currents (R={params.R_load}$\\Omega$, L={params.L_load*1e3:.0f}mH)', fontweight='bold')
    
    axes[0, 0].plot(t, o_ia, color=C['bl'], linewidth=0.3, label='$i_a$')
    axes[0, 0].plot(t, o_ib, color=C['rd'], linewidth=0.3, label='$i_b$')
    axes[0, 0].plot(t, o_ic, color=C['gr'], linewidth=0.3, label='$i_c$')
    axes[0, 0].set_ylabel('I (A)'); axes[0, 0].set_title('Full Drive Cycle')
    axes[0, 0].grid(True, alpha=0.3); axes[0, 0].legend()
    
    for idx, (iw, tw, fe_val, fsw_val, label) in enumerate([
        (results["iw_urban"], results["tw_urban"], results["fe_urban"], 5, "Urban 40km/h"),
        (results["iw_highway"], results["tw_highway"], results["fe_highway"], 20, "Highway 120km/h")
    ]):
        if len(iw) == 0: continue
        ax = axes[0, 1] if idx == 0 else axes[1, 0]
        Z = np.sqrt(params.R_load**2 + (2*np.pi*fe_val*params.L_load)**2)
        Ipk = params.m_index * params.Vdc / Z
        ax.plot(tw, o_ia[iw], color=C['bl'], linewidth=1.0, label='$i_a$')
        ax.plot(tw, o_ib[iw], color=C['rd'], linewidth=1.0, label='$i_b$')
        ax.plot(tw, o_ic[iw], color=C['gr'], linewidth=1.0, label='$i_c$')
        ax.set_ylabel('I (A)'); ax.set_xlabel('Time (ms)')
        ax.set_title(f'{label}  $f_e$={fe_val:.1f}Hz  $I_{{pk,exp}}$={Ipk:.1f}A')
        ax.grid(True, alpha=0.3); ax.legend()
    
    # Spectrum
    ax = axes[1, 1]
    if results["f_urban"] is not None:
        fe_u = results["fe_urban"]
        orders = results["f_urban"] / fe_u
        mask = orders < 50
        ax.stem(orders[mask], results["spec_urban"][mask],
                linefmt=C['tl'], markerfmt='.', basefmt='k-')
        ax.set_xlabel('Harmonic Order'); ax.set_ylabel('I (A)')
        ax.set_title(f'Urban Spectrum (THD={results["THD_urban"]:.2f}%)')
        ax.grid(True, alpha=0.3)
        ax.axvline(5e3/fe_u, color='r', linestyle='--', linewidth=1, label=f'$f_{{sw}}/f_e$')
        ax.legend()
    
    plt.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig3_phase_currents.png')); plt.close(fig)
    print("  Saved fig3_phase_currents.png")
    
    # --- Fig 4: FC Voltages ---
    fig, axes = plt.subplots(2, 2, figsize=(12, 5.5))
    fig.suptitle(f'Flying Capacitor Voltages ($V_{{ref}}$ = {params.Vc_ref:.0f}V)', fontweight='bold')
    
    axes[0, 0].plot(t, o_Vfc[0], color=C['bl'], linewidth=0.4, label='$V_{fc,a}$')
    axes[0, 0].plot(t, o_Vfc[1], color=C['rd'], linewidth=0.4, label='$V_{fc,b}$')
    axes[0, 0].plot(t, o_Vfc[2], color=C['gr'], linewidth=0.4, label='$V_{fc,c}$')
    axes[0, 0].axhline(params.Vc_ref, color='k', linestyle='--', linewidth=1.2)
    axes[0, 0].set_ylabel('$V_{fc}$ (V)'); axes[0, 0].set_title('All Phases - Full Cycle')
    axes[0, 0].grid(True, alpha=0.3); axes[0, 0].legend()
    
    for col, (iw, tw, fsw_val, label) in enumerate([
        (results["iw_urban"], results["tw_urban"], 5, "Urban ($f_{sw}$=5kHz)"),
        (results["iw_highway"], results["tw_highway"], 20, "Highway ($f_{sw}$=20kHz)")
    ]):
        if len(iw) == 0: continue
        ax = axes[0, 1] if col == 0 else axes[1, 0]
        tfc = (t[iw] - t[iw[0]]) * 1e3
        ax.plot(tfc, o_Vfc[0, iw], color=C['bl'], linewidth=1.0)
        ax.axhline(params.Vc_ref, color='k', linestyle='--', linewidth=0.8)
        rip = np.max(o_Vfc[0, iw]) - np.min(o_Vfc[0, iw])
        ax.set_ylabel('$V_{fc,a}$ (V)'); ax.set_xlabel('Time (ms)')
        ax.set_title(f'{label} - Ripple: {rip:.2f}V pk-pk'); ax.grid(True, alpha=0.3)
    
    # Bar chart
    ax = axes[1, 1]
    x = np.arange(3); width = 0.35
    rip_pred = [results["rip_pred_5k"], results["rip_pred_10k"], results["rip_pred_20k"]]
    rip_sim = [results["rip_sim_urban"] if not np.isnan(results["rip_sim_urban"]) else 0,
               0, results["rip_sim_highway"] if not np.isnan(results["rip_sim_highway"]) else 0]
    ax.bar(x - width/2, rip_pred, width, color=C['tl'], alpha=0.6, label='Predicted')
    ax.bar(x + width/2, rip_sim, width, color=C['rd'], alpha=0.6, label='Simulated')
    ax.set_xticks(x); ax.set_xticklabels(['5 kHz', '10 kHz', '20 kHz'])
    ax.set_ylabel('$\\Delta V_{fc}$ (V)'); ax.set_title('FC Ripple: Predicted vs Simulated')
    ax.grid(True, alpha=0.3, axis='y'); ax.legend()
    
    plt.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig4_fc_voltages.png')); plt.close(fig)
    print("  Saved fig4_fc_voltages.png")
    
    # --- Fig 5: Thermal ---
    fig, axes = plt.subplots(3, 1, figsize=(11, 7))
    fig.suptitle('Semiconductor Loss & Junction Temperature: Scheduled vs Fixed $f_{sw}$', fontweight='bold')
    
    axes[0].plot(t, results["P_inst_sched"], color=C['bl'], linewidth=0.4, label='Scheduled')
    axes[0].plot(t, results["P_inst_fixed"], color=C['rd'], linewidth=0.4, linestyle='--', label='Fixed 20kHz')
    axes[0].set_ylabel('$P_{loss}$ (W)'); axes[0].set_title('Instantaneous Loss - Phase A')
    axes[0].grid(True, alpha=0.3); axes[0].legend()
    
    axes[1].plot(t, results["Tj_sched"], color=C['bl'], linewidth=1.5,
                 label=f'Scheduled ($T_{{j,max}}$={np.max(results["Tj_sched"]):.1f}$^\\circ$C)')
    axes[1].plot(t, results["Tj_fixed"], color=C['rd'], linewidth=1.5, linestyle='--',
                 label=f'Fixed 20kHz ($T_{{j,max}}$={np.max(results["Tj_fixed"]):.1f}$^\\circ$C)')
    axes[1].axhline(150, color=C['rd'], linestyle='--', linewidth=1.2, alpha=0.5, label='Limit 150$^\\circ$C')
    axes[1].set_ylabel('$T_j$ ($^\\circ$C)'); axes[1].set_title('Junction Temperature (4-layer Foster)')
    axes[1].grid(True, alpha=0.3); axes[1].legend()
    
    ax = axes[2]
    cats = ['Scheduled', 'Fixed 20kHz']
    cond = [np.sum(results["P_cond_sched"]), np.sum(results["P_cond_fixed"])]
    sw = [np.sum(results["P_sw_sched"]), np.sum(results["P_sw_fixed"])]
    x = np.arange(2)
    ax.bar(x, cond, 0.5, color=C['bl'], label='Conduction')
    ax.bar(x, sw, 0.5, bottom=cond, color=C['or'], label='Switching')
    ax.set_xticks(x); ax.set_xticklabels(cats)
    ax.set_ylabel('Average Power (W)'); ax.set_title('Total Loss - All 3 Phases')
    ax.grid(True, alpha=0.3, axis='y'); ax.legend()
    
    plt.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig5_thermal.png')); plt.close(fig)
    print("  Saved fig5_thermal.png")
    
    # --- Fig 6: THD Spectra ---
    fig, axes = plt.subplots(2, 2, figsize=(12, 5.5))
    fig.suptitle('Output Current Harmonic Spectrum', fontweight='bold')
    
    for col, (idx_seg, fe_seg, fsw_seg, thd_val, clr, lbl) in enumerate([
        (results["idx_urban"], results["fe_urban"], 5e3, results["THD_urban"], C['bl'], "Urban 40km/h"),
        (results["idx_highway"], results["fe_highway"], 20e3, results["THD_highway"], C['tl'], "Highway 120km/h")
    ]):
        if np.isnan(thd_val) or len(idx_seg) == 0: continue
        sg = o_ia[idx_seg]; N_sg = len(sg)
        window = np.hanning(N_sg); cg = np.sum(window) / N_sg
        fa = np.arange(N_sg) / (N_sg * params.dt)
        Im = np.abs(np.fft.fft(sg * window)) / (N_sg * cg / 2)
        half = N_sg // 2
        
        ax = axes[0, col]
        orders = fa[:half] / fe_seg; mask = orders < 50
        ax.stem(orders[mask], Im[:half][mask], linefmt=clr, markerfmt='.', basefmt='k-')
        ax.set_xlabel('Harmonic Order'); ax.set_ylabel('I (A)')
        ax.set_title(f'{lbl}  $f_e$={fe_seg:.1f}Hz  THD={thd_val:.2f}%')
        ax.grid(True, alpha=0.3)
        ax.axvline(fsw_seg/fe_seg, color='r', linestyle='--', linewidth=1)
        
        ax = axes[1, col]
        f_max = min(3 * fsw_seg, 60e3)
        mask2 = fa[:half] < f_max
        ax.plot(fa[:half][mask2]/1e3, Im[:half][mask2], color=clr, linewidth=0.8)
        ax.set_xlabel('Frequency (kHz)'); ax.set_ylabel('I (A)')
        ax.set_title(f'Spectrum to {f_max/1e3:.0f}kHz'); ax.grid(True, alpha=0.3)
        ax.axvline(fsw_seg/1e3, color='r', linestyle='--', linewidth=1, label=f'$f_{{sw}}$={fsw_seg/1e3:.0f}kHz')
        ax.legend()
    
    plt.tight_layout(); fig.savefig(os.path.join(out_dir, 'fig6_thd_spectra.png')); plt.close(fig)
    print("  Saved fig6_thd_spectra.png")


# ============================================================================
# 6. JSON EXPORT
# ============================================================================

def export_json(results: dict, out_dir: str):
    """Export structured results."""
    params = results["params"]
    Z_load = lambda fe: np.sqrt(params.R_load**2 + (2*np.pi*fe*params.L_load)**2)
    
    R = {
        "simulation_info": {
            "version": "FC_Inverter_Sim_v2",
            "topology": "Three-phase three-level Flying Capacitor Inverter",
            "application": "800V EV traction with adaptive fsw scheduling",
            "time_step_us": params.dt * 1e6,
            "total_sim_time_s": round(results["T_sim"], 4),
            "total_steps": results["N"],
            "wall_time_s": round(results["elapsed"], 1),
            "model_features": [
                "Nonlinear switching energy (quadratic I scaling)",
                "4-layer Foster thermal network",
                "Temperature-dependent Rds_on",
                "Dead-time voltage distortion",
                "Hanning-windowed THD",
                "Dynamic FC voltage with natural PD-PWM balancing"
            ]
        },
        "parameters": {
            "Vdc_bus_V": params.Vdc_total,
            "Vdc_half_V": params.Vdc,
            "Vc_fc_ref_V": params.Vc_ref,
            "R_load_ohm": params.R_load,
            "L_load_mH": params.L_load * 1e3,
            "C_fc_mF": params.C_fc * 1e3,
            "m_index": params.m_index,
            "device": "Wolfspeed C3M0065090D (650V SiC MOSFET)",
            "Rds_on_mohm": params.Rds_on * 1e3,
            "E_on_uJ": params.E_on_ref * 1e6,
            "E_off_uJ": params.E_off_ref * 1e6,
            "dead_time_ns": params.t_dead * 1e9
        },
        "drivetrain": {
            "tyre_radius_m": params.r_tyre,
            "gear_ratio": params.gear_ratio,
            "motor_poles": params.P_poles,
            "fe_40kmh_Hz": round(speed_to_fe(40, params), 2),
            "fe_80kmh_Hz": round(speed_to_fe(80, params), 2),
            "fe_120kmh_Hz": round(speed_to_fe(120, params), 2),
            "fe_140kmh_Hz": round(speed_to_fe(140, params), 2)
        },
        "segments": [
            {"name": s["name"], "speed_kmh": s["speed_kmh"], "fe_Hz": round(s["fe_Hz"], 1),
             "fsw_kHz": s["fsw_Hz"]/1e3, "duration_s": s["duration_s"],
             "Ipk_A": round(s["Ipeak_expected"], 1)}
            for s in results["segments"]
        ],
        "electrical": {
            "peak_Ia_A": round(float(np.max(np.abs(results["o_ia"]))), 2),
            "peak_Van_V": round(float(np.max(np.abs(results["o_Van"]))), 1),
            "Ipk_urban_A": round(results["Ipk_urban"], 2),
            "Ipk_highway_A": round(results["Ipk_highway"], 2)
        },
        "fc_balance": {
            "Vc_ref_V": params.Vc_ref,
            "mean_Vfc_a_V": round(float(np.mean(results["o_Vfc"][0])), 2),
            "ripple_full_cycle_V": round(float(np.max(results["o_Vfc"][0]) - np.min(results["o_Vfc"][0])), 3),
            "ripple_urban_sim_V": round(float(results["rip_sim_urban"]), 3) if not np.isnan(results["rip_sim_urban"]) else None,
            "ripple_highway_sim_V": round(float(results["rip_sim_highway"]), 3) if not np.isnan(results["rip_sim_highway"]) else None,
            "ripple_5kHz_pred_V": round(results["rip_pred_5k"], 3),
            "ripple_20kHz_pred_V": round(results["rip_pred_20k"], 3)
        },
        "thd": {
            "urban": {"fe_Hz": round(results["fe_urban"], 1), "fsw_kHz": 5,
                      "THD_pct": round(float(results["THD_urban"]), 2) if not np.isnan(results["THD_urban"]) else None,
                      "I_fund_rms_A": round(float(results["I_fund_urban"]), 2) if not np.isnan(results["I_fund_urban"]) else None},
            "highway": {"fe_Hz": round(results["fe_highway"], 1), "fsw_kHz": 20,
                        "THD_pct": round(float(results["THD_highway"]), 2) if not np.isnan(results["THD_highway"]) else None,
                        "I_fund_rms_A": round(float(results["I_fund_highway"]), 2) if not np.isnan(results["I_fund_highway"]) else None}
        },
        "losses": {
            "scheduled": {"cond_W": round(float(np.sum(results["P_cond_sched"])), 2),
                          "sw_W": round(float(np.sum(results["P_sw_sched"])), 2),
                          "total_W": round(float(np.sum(results["P_total_sched"])), 2)},
            "fixed_20kHz": {"cond_W": round(float(np.sum(results["P_cond_fixed"])), 2),
                            "sw_W": round(float(np.sum(results["P_sw_fixed"])), 2),
                            "total_W": round(float(np.sum(results["P_total_fixed"])), 2)},
            "sw_saving_W": round(float(np.sum(results["P_sw_fixed"]) - np.sum(results["P_sw_sched"])), 2),
            "sw_saving_pct": round(float(100*(np.sum(results["P_sw_fixed"])-np.sum(results["P_sw_sched"]))/max(np.sum(results["P_sw_fixed"]),1e-9)), 1)
        },
        "thermal": {
            "model": "4-layer Foster + Rth_cs + Rth_sa",
            "T_ambient_C": params.T_ambient,
            "Tj_max_sched_C": round(float(np.max(results["Tj_sched"])), 1),
            "Tj_max_fixed_C": round(float(np.max(results["Tj_fixed"])), 1),
            "Tj_limit_C": 150,
            "margin_sched_C": round(float(150 - np.max(results["Tj_sched"])), 1),
            "margin_fixed_C": round(float(150 - np.max(results["Tj_fixed"])), 1),
            "Tj_saving_C": round(float(np.max(results["Tj_fixed"]) - np.max(results["Tj_sched"])), 2)
        }
    }
    
    with open(os.path.join(out_dir, 'results.json'), 'w') as f:
        json.dump(R, f, indent=2)
    print("  Saved results.json")
    return R


# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fc_inverter_results')
    
    params = SystemParams()
    results = run_simulation(params, verbose=True)
    
    print("  Generating figures...")
    generate_figures(results, OUT_DIR)
    
    print("\n  Exporting results...")
    export_json(results, OUT_DIR)
    
    print(f"\n{'='*60}")
    print(f"  All outputs in: {OUT_DIR}")
    print(f"{'='*60}")
