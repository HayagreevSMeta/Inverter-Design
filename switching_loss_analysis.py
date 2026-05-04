#!/usr/bin/env python3
"""
Analytical Switching Loss Analysis for 3-Level Flying Capacitor Inverter
========================================================================
Uses the Graovac/Purschel method (Infineon AN 2009-05) for duty-cycle-weighted
average switching loss calculation — the standard approach in IEEE traction
inverter literature.

Vehicle Architecture: 800V EV (e.g., Hyundai E-GMP / Porsche PPE class)
Motor: 8-pole PMSM, gear ratio 8.19:1, tyre radius 0.315m
Topology: 3-Level Flying Capacitor with PD-PWM
Device: Wolfspeed CAB450M12XM3 (1200V, 450A SiC Half-Bridge)
"""

import numpy as np
import matplotlib.pyplot as plt
import json
import os

# ============================================================================
# VEHICLE ARCHITECTURE PARAMETERS
# ============================================================================
# These define the electrical operating points at each vehicle speed
TYRE_RADIUS = 0.315       # m (245/45R19, typical performance EV)
GEAR_RATIO = 8.19         # Single-speed reduction (e.g., Hyundai E-GMP)
POLE_PAIRS = 4            # 8-pole PMSM (common for EV traction)
VDC = 800.0               # V, DC bus voltage (800V architecture)

# ============================================================================
# DEVICE PARAMETERS (from Wolfspeed CAB450M12XM3 datasheet)
# ============================================================================
RDS_ON_25C = 3.3e-3       # Ω, total effective Rds_on at 25°C (die + package)
RDS_ON_175C = 6.6e-3      # Ω, at 175°C (2.0x from datasheet Fig. 2)
EON_REF = 25.4e-3         # J, turn-on energy at I_ref, V_ref
EOFF_REF = 7.51e-3        # J, turn-off energy at I_ref, V_ref
I_REF = 450.0             # A, reference current for switching energy
V_REF = 600.0             # V, reference voltage for switching energy
RTH_JC = 0.094            # °C/W, junction-to-case thermal resistance
RTH_CS = 0.02             # °C/W, case-to-sink (thermal grease)
RTH_SA = 0.03             # °C/W, sink-to-ambient (liquid cooled)
T_COOLANT = 65.0          # °C, coolant temperature (typical EV)

# ============================================================================
# LOAD PARAMETERS (per project specification)
# ============================================================================
R_LOAD = 1.3              # Ω, load resistance
L_LOAD = 2.0e-3           # H, load inductance
M_INDEX = 0.9             # Modulation index

# ============================================================================
# DERIVED: Vehicle speed → Electrical frequency → Peak current
# ============================================================================
def speed_to_fe(v_kmh):
    """Convert vehicle speed (km/h) to electrical frequency (Hz)."""
    v_ms = v_kmh / 3.6
    omega_wheel = v_ms / TYRE_RADIUS
    omega_motor = omega_wheel * GEAR_RATIO
    fe = omega_motor * POLE_PAIRS / (2 * np.pi)
    return fe

def fe_to_ipk(fe):
    """Peak phase current for RL load at given frequency."""
    omega = 2 * np.pi * fe
    Z = np.sqrt(R_LOAD**2 + (omega * L_LOAD)**2)
    Vpk = M_INDEX * VDC / 2  # Peak phase voltage (3-level)
    return Vpk / Z

def fe_to_pf(fe):
    """Power factor (cos φ) for RL load at given frequency."""
    omega = 2 * np.pi * fe
    return R_LOAD / np.sqrt(R_LOAD**2 + (omega * L_LOAD)**2)

# ============================================================================
# ANALYTICAL LOSS MODEL (Graovac/Purschel Method)
# ============================================================================
def switching_loss_per_phase(fsw, Ipk, Vdc_sw=VDC/2):
    """
    Analytical switching loss per phase leg using the standard
    duty-cycle-weighted average method [Graovac & Purschel, 2009].
    
    For a 3-level FC inverter with PD-PWM:
    - Each switching transition involves 2 devices commutating
    - Voltage across each switch during commutation = Vdc/2
    - Current is sinusoidal: i(θ) = Ipk * sin(θ)
    
    P_sw = N_commutating * (fsw/π) * (Eon+Eoff)_ref * (Ipk/I_ref) * (Vdc_sw/V_ref)
    
    The (1/π) factor comes from averaging |sin(θ)| over a full cycle:
    (1/2π) * ∫₀²π |sin(θ)| dθ = 2/π → times Ipk gives average |i| = 2*Ipk/π
    Then E_sw scales linearly with current → integral gives (Ipk/I_ref)*(1/π)
    """
    N_commutating = 2  # In 3L-FC, 2 switches commutate per PWM transition
    Esw_ref = EON_REF + EOFF_REF  # Total switching energy at reference
    
    P_sw = N_commutating * (fsw / np.pi) * Esw_ref * (Ipk / I_REF) * (Vdc_sw / V_REF)
    return P_sw

def conduction_loss_per_phase(Ipk, cos_phi, Tj=100.0):
    """
    Analytical conduction loss per phase leg.
    
    For 3L-FC with 4 switches per phase, at any instant 2 switches conduct.
    RMS current through each switch depends on modulation index and power factor.
    
    Simplified: I²_rms per switch ≈ Ipk²/8 for 3-level
    Total conduction loss per phase = 2 * Rds_on(Tj) * Ipk²/8
    """
    # Temperature-dependent Rds_on (linear interpolation)
    alpha = (Tj - 25.0) / (175.0 - 25.0)
    Rds_on = RDS_ON_25C * (1.0 + alpha)  # 2x at 175°C
    
    # Two switches always conducting in 3L-FC
    Irms_sq_per_switch = Ipk**2 / 8.0
    P_cond = 2 * Rds_on * Irms_sq_per_switch
    return P_cond

def junction_temperature(P_total_phase, T_coolant=T_COOLANT):
    """Estimate steady-state junction temperature."""
    # Each phase has 4 switches, loss distributed
    Rth_total = RTH_JC + RTH_CS + RTH_SA
    # P_total_phase is split among 4 switches (approximately)
    P_per_switch = P_total_phase / 4.0
    Tj = T_coolant + P_per_switch * Rth_total
    return Tj

# ============================================================================
# ANALYSIS: Sweep across vehicle speeds and switching frequencies
# ============================================================================
print("=" * 70)
print("  ANALYTICAL SWITCHING LOSS ANALYSIS")
print("  3-Level Flying Capacitor Inverter | 800V EV Traction")
print("  Method: Graovac/Purschel (Infineon AN 2009-05)")
print("=" * 70)

# Vehicle speed range
speeds = np.array([20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140])
fe_array = np.array([speed_to_fe(v) for v in speeds])
Ipk_array = np.array([fe_to_ipk(fe) for fe in fe_array])
pf_array = np.array([fe_to_pf(fe) for fe in fe_array])

# Switching frequencies to compare
fsw_values = [5e3, 10e3, 15e3, 20e3]

# Print operating points
print(f"\n{'Speed':>6} {'fe':>8} {'Ipk':>8} {'cos φ':>8} {'Z':>8}")
print(f"{'km/h':>6} {'Hz':>8} {'A':>8} {'':>8} {'Ω':>8}")
print("-" * 45)
for v, fe, Ipk, pf in zip(speeds, fe_array, Ipk_array, pf_array):
    Z = np.sqrt(R_LOAD**2 + (2*np.pi*fe*L_LOAD)**2)
    print(f"{v:>6.0f} {fe:>8.1f} {Ipk:>8.1f} {pf:>8.3f} {Z:>8.2f}")

# Compute losses at each speed for each fsw
print(f"\n{'='*70}")
print(f"  SWITCHING LOSSES PER PHASE (W) vs Speed and fsw")
print(f"{'='*70}")
print(f"{'Speed':>6} {'fe':>7} {'Ipk':>6} | {'5kHz':>8} {'10kHz':>8} {'15kHz':>8} {'20kHz':>8}")
print("-" * 65)

results = {}
for fsw in fsw_values:
    results[fsw] = {'sw': [], 'cond': [], 'total': [], 'Tj': []}

for i, (v, fe, Ipk, pf) in enumerate(zip(speeds, fe_array, Ipk_array, pf_array)):
    line = f"{v:>6.0f} {fe:>7.1f} {Ipk:>6.1f} |"
    for fsw in fsw_values:
        P_sw = switching_loss_per_phase(fsw, Ipk)
        P_cond = conduction_loss_per_phase(Ipk, pf, Tj=100.0)
        P_total = P_sw + P_cond
        Tj = junction_temperature(P_total)
        results[fsw]['sw'].append(P_sw)
        results[fsw]['cond'].append(P_cond)
        results[fsw]['total'].append(P_total)
        results[fsw]['Tj'].append(Tj)
        line += f" {P_sw:>7.1f}W"
    print(line)

# ============================================================================
# ADAPTIVE SCHEDULE: Define the optimal fsw for each speed range
# ============================================================================
# Schedule: Urban (<60 km/h) → 5 kHz, Extra-urban (60-80) → 10 kHz, Highway (>80) → 20 kHz
def get_scheduled_fsw(v_kmh):
    if v_kmh <= 60:
        return 5e3
    elif v_kmh <= 80:
        return 10e3
    else:
        return 20e3

scheduled_sw = []
scheduled_total = []
fixed_20k_sw = []
fixed_20k_total = []

print(f"\n{'='*70}")
print(f"  ADAPTIVE vs FIXED COMPARISON (per phase)")
print(f"{'='*70}")
print(f"{'Speed':>6} {'fe':>7} {'Ipk':>6} | {'Sched fsw':>9} {'P_sw_s':>8} {'P_sw_f':>8} {'Saving':>8}")
print("-" * 65)

for i, (v, fe, Ipk, pf) in enumerate(zip(speeds, fe_array, Ipk_array, pf_array)):
    fsw_s = get_scheduled_fsw(v)
    P_sw_sched = switching_loss_per_phase(fsw_s, Ipk)
    P_sw_fixed = switching_loss_per_phase(20e3, Ipk)
    P_cond = conduction_loss_per_phase(Ipk, pf, Tj=100.0)
    
    scheduled_sw.append(P_sw_sched)
    scheduled_total.append(P_sw_sched + P_cond)
    fixed_20k_sw.append(P_sw_fixed)
    fixed_20k_total.append(P_sw_fixed + P_cond)
    
    saving = (1 - P_sw_sched/P_sw_fixed) * 100 if P_sw_fixed > 0 else 0
    print(f"{v:>6.0f} {fe:>7.1f} {Ipk:>6.1f} | {fsw_s/1e3:>7.0f}kHz {P_sw_sched:>7.1f}W {P_sw_fixed:>7.1f}W {saving:>6.1f}%")

# ============================================================================
# GENERATE FIGURES
# ============================================================================
out_dir = '/home/ubuntu/Inverter-Design/switching_loss_figures'
os.makedirs(out_dir, exist_ok=True)

plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 9,
    'axes.labelsize': 10,
    'axes.titlesize': 11,
    'legend.fontsize': 8,
    'figure.dpi': 200,
})

# --- Figure 1: Switching Loss vs Vehicle Speed for different fsw ---
fig, ax = plt.subplots(figsize=(7, 4.5))
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
for j, fsw in enumerate(fsw_values):
    ax.plot(speeds, results[fsw]['sw'], 'o-', color=colors[j], 
            label=f'$f_{{sw}}$ = {fsw/1e3:.0f} kHz', linewidth=1.5, markersize=4)

# Overlay the scheduled line
ax.plot(speeds, scheduled_sw, 'k-s', linewidth=2.5, markersize=6, 
        label='Adaptive Schedule', zorder=5)

ax.set_xlabel('Vehicle Speed (km/h)')
ax.set_ylabel('Switching Loss per Phase (W)')
ax.set_title('Switching Loss vs. Vehicle Speed — Effect of $f_{sw}$')
ax.legend(loc='upper right')
ax.grid(True, alpha=0.3)
ax.set_xlim([20, 140])
ax.set_ylim(bottom=0)

# Add speed regime annotations
ax.axvspan(20, 60, alpha=0.05, color='blue', label='_nolegend_')
ax.axvspan(60, 80, alpha=0.05, color='green', label='_nolegend_')
ax.axvspan(80, 140, alpha=0.05, color='red', label='_nolegend_')
ax.text(40, ax.get_ylim()[1]*0.92, 'Urban\n5 kHz', ha='center', fontsize=8, color='blue')
ax.text(70, ax.get_ylim()[1]*0.92, 'Extra-\nurban\n10 kHz', ha='center', fontsize=8, color='green')
ax.text(110, ax.get_ylim()[1]*0.92, 'Highway\n20 kHz', ha='center', fontsize=8, color='red')

plt.tight_layout()
plt.savefig(f'{out_dir}/fig1_switching_loss_vs_speed.png', dpi=200, bbox_inches='tight')
plt.close()
print(f"\nSaved fig1_switching_loss_vs_speed.png")

# --- Figure 2: Loss Breakdown (Stacked Bar) at key operating points ---
fig, ax = plt.subplots(figsize=(7, 4))
key_speeds = [30, 40, 60, 80, 100, 120, 130]
key_indices = [list(speeds).index(v) for v in key_speeds]

x = np.arange(len(key_speeds))
width = 0.35

# Scheduled
sw_s = [scheduled_sw[i] for i in key_indices]
cond_s = [conduction_loss_per_phase(Ipk_array[i], pf_array[i], 100.0) for i in key_indices]

# Fixed 20kHz
sw_f = [fixed_20k_sw[i] for i in key_indices]
cond_f = [conduction_loss_per_phase(Ipk_array[i], pf_array[i], 100.0) for i in key_indices]

bars1 = ax.bar(x - width/2, cond_s, width, label='Conduction (Sched.)', color='#2c3e50')
bars2 = ax.bar(x - width/2, sw_s, width, bottom=cond_s, label='Switching (Sched.)', color='#e67e22')
bars3 = ax.bar(x + width/2, cond_f, width, label='Conduction (Fixed)', color='#2c3e50', alpha=0.4)
bars4 = ax.bar(x + width/2, sw_f, width, bottom=cond_f, label='Switching (Fixed)', color='#e67e22', alpha=0.4)

ax.set_xlabel('Vehicle Speed (km/h)')
ax.set_ylabel('Loss per Phase (W)')
ax.set_title('Loss Breakdown: Adaptive Schedule vs. Fixed 20 kHz')
ax.set_xticks(x)
ax.set_xticklabels([f'{v}' for v in key_speeds])
ax.legend(loc='upper right', ncol=2)
ax.grid(True, alpha=0.2, axis='y')

plt.tight_layout()
plt.savefig(f'{out_dir}/fig2_loss_breakdown.png', dpi=200, bbox_inches='tight')
plt.close()
print(f"Saved fig2_loss_breakdown.png")

# --- Figure 3: Switching Loss Saving (%) vs Speed ---
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 4))

# Left: Absolute saving
saving_W = np.array(fixed_20k_sw) - np.array(scheduled_sw)
ax1.bar(speeds, saving_W, width=7, color='#27ae60', edgecolor='#1e8449')
ax1.set_xlabel('Vehicle Speed (km/h)')
ax1.set_ylabel('Switching Loss Saving (W/phase)')
ax1.set_title('Absolute Switching Loss Reduction')
ax1.grid(True, alpha=0.3, axis='y')
ax1.set_xlim([15, 145])

# Right: Percentage saving
saving_pct = np.where(np.array(fixed_20k_sw) > 0, 
                      saving_W / np.array(fixed_20k_sw) * 100, 0)
ax2.bar(speeds, saving_pct, width=7, color='#2980b9', edgecolor='#1a5276')
ax2.set_xlabel('Vehicle Speed (km/h)')
ax2.set_ylabel('Switching Loss Saving (%)')
ax2.set_title('Relative Switching Loss Reduction')
ax2.grid(True, alpha=0.3, axis='y')
ax2.set_xlim([15, 145])
ax2.set_ylim([0, 100])
ax2.axhline(y=75, color='r', linestyle='--', alpha=0.5, label='75% (urban)')
ax2.legend()

plt.tight_layout()
plt.savefig(f'{out_dir}/fig3_switching_saving.png', dpi=200, bbox_inches='tight')
plt.close()
print(f"Saved fig3_switching_saving.png")

# --- Figure 4: Vehicle Architecture Mapping ---
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(7, 7), sharex=True)

ax1.plot(speeds, fe_array, 'b-o', markersize=4, linewidth=1.5)
ax1.set_ylabel('Electrical Frequency $f_e$ (Hz)')
ax1.set_title('Vehicle Architecture → Inverter Operating Points')
ax1.grid(True, alpha=0.3)
ax1.axhline(y=500, color='r', linestyle='--', alpha=0.5)
ax1.text(25, 520, '$f_e$ = 500 Hz (typical max for 5 kHz carrier)', fontsize=7, color='r')

ax2.plot(speeds, Ipk_array, 'r-o', markersize=4, linewidth=1.5)
ax2.set_ylabel('Peak Phase Current $I_{pk}$ (A)')
ax2.grid(True, alpha=0.3)
ax2.axhline(y=136, color='gray', linestyle=':', alpha=0.7)
ax2.text(100, 142, 'Urban design point (136A)', fontsize=7, color='gray')

# fsw schedule
fsw_sched = [get_scheduled_fsw(v)/1e3 for v in speeds]
ax3.step(speeds, fsw_sched, 'g-', linewidth=2, where='mid')
ax3.fill_between(speeds, fsw_sched, alpha=0.2, color='green', step='mid')
ax3.set_ylabel('Switching Frequency $f_{sw}$ (kHz)')
ax3.set_xlabel('Vehicle Speed (km/h)')
ax3.grid(True, alpha=0.3)
ax3.set_ylim([0, 25])
ax3.set_yticks([5, 10, 15, 20])

plt.tight_layout()
plt.savefig(f'{out_dir}/fig4_vehicle_architecture.png', dpi=200, bbox_inches='tight')
plt.close()
print(f"Saved fig4_vehicle_architecture.png")

# --- Figure 5: fsw/fe ratio (carrier ratio) ---
fig, ax = plt.subplots(figsize=(7, 4))
for fsw in fsw_values:
    ratio = fsw / fe_array
    ax.plot(speeds, ratio, 'o-', label=f'$f_{{sw}}$ = {fsw/1e3:.0f} kHz', markersize=4)

# Scheduled ratio
sched_ratio = [get_scheduled_fsw(v) / fe for v, fe in zip(speeds, fe_array)]
ax.plot(speeds, sched_ratio, 'k-s', linewidth=2.5, markersize=6, label='Adaptive Schedule')

ax.axhline(y=9, color='r', linestyle='--', alpha=0.7)
ax.text(25, 10, 'Minimum $m_f$ = 9 (IEEE recommended)', fontsize=8, color='r')
ax.set_xlabel('Vehicle Speed (km/h)')
ax.set_ylabel('Frequency Modulation Ratio $m_f = f_{sw}/f_e$')
ax.set_title('Carrier Ratio Across Drive Cycle')
ax.legend(loc='upper right')
ax.grid(True, alpha=0.3)
ax.set_xlim([20, 140])
ax.set_ylim([0, 80])

plt.tight_layout()
plt.savefig(f'{out_dir}/fig5_carrier_ratio.png', dpi=200, bbox_inches='tight')
plt.close()
print(f"Saved fig5_carrier_ratio.png")

# ============================================================================
# SUMMARY TABLE
# ============================================================================
print(f"\n{'='*70}")
print(f"  SUMMARY: KEY RESULTS")
print(f"{'='*70}")

# Urban design point
v_urban, fe_urban, Ipk_urban = 40, speed_to_fe(40), fe_to_ipk(speed_to_fe(40))
P_sw_urban_5k = switching_loss_per_phase(5e3, Ipk_urban)
P_sw_urban_20k = switching_loss_per_phase(20e3, Ipk_urban)
P_cond_urban = conduction_loss_per_phase(Ipk_urban, fe_to_pf(fe_urban), 100.0)

# Highway design point
v_hw, fe_hw, Ipk_hw = 120, speed_to_fe(120), fe_to_ipk(speed_to_fe(120))
P_sw_hw_20k = switching_loss_per_phase(20e3, Ipk_hw)
P_cond_hw = conduction_loss_per_phase(Ipk_hw, fe_to_pf(fe_hw), 100.0)

print(f"\n  Urban (40 km/h): fe={fe_urban:.1f} Hz, Ipk={Ipk_urban:.1f} A")
print(f"    Switching loss @ 5kHz:  {P_sw_urban_5k:.1f} W/phase")
print(f"    Switching loss @ 20kHz: {P_sw_urban_20k:.1f} W/phase")
print(f"    Conduction loss:        {P_cond_urban:.1f} W/phase")
print(f"    Saving (5kHz vs 20kHz): {(1-P_sw_urban_5k/P_sw_urban_20k)*100:.1f}%")

print(f"\n  Highway (120 km/h): fe={fe_hw:.1f} Hz, Ipk={Ipk_hw:.1f} A")
print(f"    Switching loss @ 20kHz: {P_sw_hw_20k:.1f} W/phase")
print(f"    Conduction loss:        {P_cond_hw:.1f} W/phase")

print(f"\n  Total 3-phase losses (adaptive schedule):")
total_urban = 3 * (P_sw_urban_5k + P_cond_urban)
total_hw = 3 * (P_sw_hw_20k + P_cond_hw)
print(f"    Urban:   {total_urban:.1f} W")
print(f"    Highway: {total_hw:.1f} W")

print(f"\n  Total 3-phase losses (fixed 20kHz):")
total_urban_fixed = 3 * (P_sw_urban_20k + P_cond_urban)
print(f"    Urban:   {total_urban_fixed:.1f} W")
print(f"    Highway: {total_hw:.1f} W (same)")

print(f"\n  Urban switching loss saving: {3*(P_sw_urban_20k-P_sw_urban_5k):.1f} W (3-phase)")

# Save results as JSON
output = {
    "method": "Graovac/Purschel analytical model (Infineon AN 2009-05)",
    "vehicle_architecture": {
        "Vdc_V": VDC,
        "tyre_radius_m": TYRE_RADIUS,
        "gear_ratio": GEAR_RATIO,
        "pole_pairs": POLE_PAIRS,
        "motor_type": "8-pole PMSM"
    },
    "device": "Wolfspeed CAB450M12XM3",
    "device_params": {
        "Rds_on_25C_mohm": RDS_ON_25C*1e3,
        "Eon_ref_mJ": EON_REF*1e3,
        "Eoff_ref_mJ": EOFF_REF*1e3,
        "I_ref_A": I_REF,
        "V_ref_V": V_REF
    },
    "operating_points": [
        {"speed_kmh": int(v), "fe_Hz": round(fe, 1), "Ipk_A": round(Ipk, 1), 
         "cos_phi": round(pf, 3)}
        for v, fe, Ipk, pf in zip(speeds, fe_array, Ipk_array, pf_array)
    ],
    "key_results": {
        "urban_40kmh": {
            "fe_Hz": round(fe_urban, 1),
            "Ipk_A": round(Ipk_urban, 1),
            "P_sw_5kHz_W_per_phase": round(P_sw_urban_5k, 1),
            "P_sw_20kHz_W_per_phase": round(P_sw_urban_20k, 1),
            "P_cond_W_per_phase": round(P_cond_urban, 1),
            "saving_pct": round((1-P_sw_urban_5k/P_sw_urban_20k)*100, 1)
        },
        "highway_120kmh": {
            "fe_Hz": round(fe_hw, 1),
            "Ipk_A": round(Ipk_hw, 1),
            "P_sw_20kHz_W_per_phase": round(P_sw_hw_20k, 1),
            "P_cond_W_per_phase": round(P_cond_hw, 1)
        },
        "total_3phase_urban_scheduled_W": round(total_urban, 1),
        "total_3phase_urban_fixed_W": round(total_urban_fixed, 1),
        "total_3phase_highway_W": round(total_hw, 1),
        "urban_sw_saving_3phase_W": round(3*(P_sw_urban_20k-P_sw_urban_5k), 1)
    }
}

with open(f'{out_dir}/analytical_results.json', 'w') as f:
    json.dump(output, f, indent=2)
print(f"\nSaved analytical_results.json")
print(f"\nAll outputs in: {out_dir}")
