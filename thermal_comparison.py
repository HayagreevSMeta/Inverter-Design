#!/usr/bin/env python3
"""
Thermal Comparison: Adaptive Switching Frequency vs Constant 20 kHz Baseline
=============================================================================
Compares semiconductor junction temperature under:
  - Baseline: Fixed 20 kHz switching frequency at all speeds
  - Adaptive: Speed-dependent fsw schedule (5 kHz urban, 10 kHz extra-urban, 20 kHz highway)

Uses the Wolfspeed CAB450M12XM3 datasheet parameters:
  - 4-layer Foster thermal network (Zth_jc)
  - Temperature-dependent Rds_on (3.3 mΩ @ 25°C → 6.6 mΩ @ 175°C)
  - Linear switching energy scaling (SiC)
"""

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import json
import os

# ===========================================================================
# DEVICE PARAMETERS (Wolfspeed CAB450M12XM3 Datasheet)
# ===========================================================================
Rds_on_25C = 3.3e-3       # Ohms at 25°C
Rds_on_175C = 6.6e-3      # Ohms at 175°C
Eon_ref = 25.4e-3          # J at Iref=450A, Vref=600V
Eoff_ref = 7.51e-3         # J at Iref=450A, Vref=600V
Err_ref = 0.2e-3           # J (diode reverse recovery)
I_ref = 450.0              # A (reference current for Esw)
V_ref = 600.0              # V (reference voltage for Esw)
Vdc = 800.0                # V (DC bus)
Vdc_sw = Vdc / 2           # V per switch in 3L-FC
Nc = 2                     # Commutating switches per transition (3L-FC)

# 4-layer Foster thermal network (junction to case)
# Rth_i (°C/W), tau_i (s)
foster_R = np.array([0.0235, 0.0235, 0.0235, 0.0235])  # Total Rth_jc = 0.094 °C/W
foster_tau = np.array([0.001, 0.005, 0.02, 0.1])

# Case-to-coolant thermal resistance (heatsink + TIM)
Rth_cs = 0.05  # °C/W (typical liquid-cooled heatsink for automotive)

# Ambient/coolant temperature
T_coolant = 40.0  # °C (typical EV coolant temp)

# ===========================================================================
# VEHICLE ARCHITECTURE
# ===========================================================================
G = 8.19           # Gear ratio
r_tyre = 0.315     # m
p = 4              # Pole pairs
R_load = 1.3       # Ohms
L_load = 2e-3      # H
m_a = 0.9          # Modulation index

# ===========================================================================
# HELPER FUNCTIONS
# ===========================================================================
def speed_to_fe(v_kmh):
    """Vehicle speed (km/h) to electrical frequency (Hz)."""
    v_ms = v_kmh / 3.6
    return (v_ms / r_tyre) * (G / (2 * np.pi)) * p

def speed_to_Ipk(v_kmh):
    """Peak phase current at given vehicle speed (RL load model)."""
    fe = speed_to_fe(v_kmh)
    omega_e = 2 * np.pi * fe
    Z = np.sqrt(R_load**2 + (omega_e * L_load)**2)
    Vpk = m_a * Vdc / 2
    return Vpk / Z

def rdson_at_Tj(Tj):
    """Linear interpolation of Rds_on vs temperature."""
    return Rds_on_25C + (Rds_on_175C - Rds_on_25C) * (Tj - 25.0) / (175.0 - 25.0)

def conduction_loss(Ipk, Tj):
    """Conduction loss per phase (4 switches, 2 always in path for 3L-FC)."""
    Rds = rdson_at_Tj(Tj)
    I_rms = Ipk / np.sqrt(2)
    # In 3L-FC, 2 switches always conduct → P_cond = 2 * Rds * I_rms^2
    return 2 * Rds * I_rms**2

def switching_loss(Ipk, fsw):
    """Switching loss per phase using Graovac-Purschel method."""
    Esw_total = Eon_ref + Eoff_ref + Err_ref
    # Average switching loss over sinusoidal half-wave
    Psw = Nc * (fsw / np.pi) * Esw_total * (Ipk / I_ref) * (Vdc_sw / V_ref)
    return Psw

def total_loss(Ipk, fsw, Tj):
    """Total loss per phase."""
    return conduction_loss(Ipk, Tj) + switching_loss(Ipk, fsw)

def steady_state_Tj(Ipk, fsw):
    """Iteratively solve for steady-state Tj (thermal-electrical coupling)."""
    Tj = T_coolant + 10  # Initial guess
    Rth_total = np.sum(foster_R) + Rth_cs  # 0.094 + 0.05 = 0.144 °C/W
    for _ in range(50):
        P = total_loss(Ipk, fsw, Tj)
        Tj_new = T_coolant + P * Rth_total
        if abs(Tj_new - Tj) < 0.01:
            break
        Tj = 0.5 * Tj + 0.5 * Tj_new
    return Tj

def transient_thermal(power_profile, dt):
    """
    Simulate transient thermal response using 4-layer Foster network.
    Uses implicit (exact) exponential solution for numerical stability.
    power_profile: array of power values at each timestep
    dt: timestep (s)
    Returns: array of Tj values
    """
    n_layers = len(foster_R)
    n_steps = len(power_profile)
    T_layers = np.zeros(n_layers)
    Tj_out = np.zeros(n_steps)
    
    # Pre-compute exponential decay factors for each layer
    exp_factors = np.exp(-dt / foster_tau)
    
    for k in range(n_steps):
        P = power_profile[k]
        for i in range(n_layers):
            # Exact solution: T_i(t+dt) = T_ss + (T_i(t) - T_ss) * exp(-dt/tau_i)
            T_ss = P * foster_R[i]  # Steady-state for this layer
            T_layers[i] = T_ss + (T_layers[i] - T_ss) * exp_factors[i]
        Tj_out[k] = T_coolant + np.sum(T_layers) + P * Rth_cs
    
    return Tj_out

# ===========================================================================
# ADAPTIVE SCHEDULE DEFINITION
# ===========================================================================
def adaptive_fsw(v_kmh):
    """Speed-dependent switching frequency schedule."""
    if v_kmh < 60:
        return 5000      # 5 kHz for urban
    elif v_kmh < 90:
        return 10000     # 10 kHz for extra-urban
    else:
        return 20000     # 20 kHz for highway

FIXED_FSW = 20000  # 20 kHz baseline

# ===========================================================================
# ANALYSIS 1: STEADY-STATE Tj vs VEHICLE SPEED
# ===========================================================================
speeds = np.linspace(20, 140, 100)
Tj_fixed = np.zeros_like(speeds)
Tj_adaptive = np.zeros_like(speeds)
Ploss_fixed = np.zeros_like(speeds)
Ploss_adaptive = np.zeros_like(speeds)
Pcond_arr = np.zeros_like(speeds)
Psw_fixed_arr = np.zeros_like(speeds)
Psw_adaptive_arr = np.zeros_like(speeds)

for i, v in enumerate(speeds):
    Ipk = speed_to_Ipk(v)
    fsw_adap = adaptive_fsw(v)
    
    Tj_fixed[i] = steady_state_Tj(Ipk, FIXED_FSW)
    Tj_adaptive[i] = steady_state_Tj(Ipk, fsw_adap)
    
    Ploss_fixed[i] = total_loss(Ipk, FIXED_FSW, Tj_fixed[i])
    Ploss_adaptive[i] = total_loss(Ipk, fsw_adap, Tj_adaptive[i])
    
    Pcond_arr[i] = conduction_loss(Ipk, Tj_fixed[i])
    Psw_fixed_arr[i] = switching_loss(Ipk, FIXED_FSW)
    Psw_adaptive_arr[i] = switching_loss(Ipk, fsw_adap)

# ===========================================================================
# ANALYSIS 2: TRANSIENT THERMAL - DRIVE CYCLE SIMULATION
# ===========================================================================
# Simulate a realistic drive cycle: urban → extra-urban → highway → decel
dt = 0.01  # 10ms timestep
t_urban = 30.0      # 30s at 40 km/h
t_accel1 = 10.0     # 10s acceleration 40→80
t_extra = 30.0      # 30s at 80 km/h
t_accel2 = 10.0     # 10s acceleration 80→120
t_highway = 30.0    # 30s at 120 km/h
t_decel = 15.0      # 15s deceleration 120→40
t_urban2 = 25.0     # 25s at 40 km/h again

total_time = t_urban + t_accel1 + t_extra + t_accel2 + t_highway + t_decel + t_urban2
n_steps = int(total_time / dt)
time_arr = np.linspace(0, total_time, n_steps)

# Build speed profile
speed_profile = np.zeros(n_steps)
t_cum = 0
for k in range(n_steps):
    t = time_arr[k]
    if t < t_urban:
        speed_profile[k] = 40
    elif t < t_urban + t_accel1:
        frac = (t - t_urban) / t_accel1
        speed_profile[k] = 40 + frac * 40
    elif t < t_urban + t_accel1 + t_extra:
        speed_profile[k] = 80
    elif t < t_urban + t_accel1 + t_extra + t_accel2:
        frac = (t - t_urban - t_accel1 - t_extra) / t_accel2
        speed_profile[k] = 80 + frac * 40
    elif t < t_urban + t_accel1 + t_extra + t_accel2 + t_highway:
        speed_profile[k] = 120
    elif t < t_urban + t_accel1 + t_extra + t_accel2 + t_highway + t_decel:
        frac = (t - t_urban - t_accel1 - t_extra - t_accel2 - t_highway) / t_decel
        speed_profile[k] = 120 - frac * 80
    else:
        speed_profile[k] = 40

# Compute power profiles for both strategies
P_fixed_transient = np.zeros(n_steps)
P_adaptive_transient = np.zeros(n_steps)
fsw_profile = np.zeros(n_steps)

for k in range(n_steps):
    v = speed_profile[k]
    Ipk = speed_to_Ipk(v)
    fsw_adap = adaptive_fsw(v)
    fsw_profile[k] = fsw_adap / 1000  # kHz for plotting
    
    # Use approximate Tj for loss calc (iterate once)
    Tj_approx = 50.0  # Reasonable operating point
    P_fixed_transient[k] = total_loss(Ipk, FIXED_FSW, Tj_approx)
    P_adaptive_transient[k] = total_loss(Ipk, fsw_adap, Tj_approx)

# Run transient thermal simulation
Tj_fixed_transient = transient_thermal(P_fixed_transient, dt)
Tj_adaptive_transient = transient_thermal(P_adaptive_transient, dt)

# ===========================================================================
# ANALYSIS 3: KEY OPERATING POINTS COMPARISON TABLE
# ===========================================================================
op_points = [
    ("Urban (40 km/h)", 40),
    ("Extra-Urban (80 km/h)", 80),
    ("Highway (120 km/h)", 120),
    ("Peak Torque (20 km/h)", 20),
]

comparison_data = []
for name, v in op_points:
    Ipk = speed_to_Ipk(v)
    fe = speed_to_fe(v)
    fsw_adap = adaptive_fsw(v)
    
    Tj_f = steady_state_Tj(Ipk, FIXED_FSW)
    Tj_a = steady_state_Tj(Ipk, fsw_adap)
    P_f = total_loss(Ipk, FIXED_FSW, Tj_f)
    P_a = total_loss(Ipk, fsw_adap, Tj_a)
    
    comparison_data.append({
        "operating_point": name,
        "speed_kmh": v,
        "fe_Hz": round(fe, 1),
        "Ipk_A": round(Ipk, 1),
        "fsw_fixed_kHz": FIXED_FSW / 1000,
        "fsw_adaptive_kHz": fsw_adap / 1000,
        "Ploss_fixed_W": round(P_f, 1),
        "Ploss_adaptive_W": round(P_a, 1),
        "Tj_fixed_C": round(Tj_f, 1),
        "Tj_adaptive_C": round(Tj_a, 1),
        "Tj_reduction_C": round(Tj_f - Tj_a, 1),
        "Tj_reduction_pct": round((Tj_f - Tj_a) / (Tj_f - T_coolant) * 100, 1),
    })

# ===========================================================================
# FIGURE GENERATION
# ===========================================================================
out_dir = "/home/ubuntu/Inverter-Design/thermal_comparison_figures"
os.makedirs(out_dir, exist_ok=True)

# --- Figure 1: Steady-State Tj vs Speed (main comparison) ---
fig1 = go.Figure()
fig1.add_trace(go.Scatter(
    x=speeds, y=Tj_fixed,
    mode='lines', name='Constant 20 kHz',
    line=dict(color='#E74C3C', width=3)
))
fig1.add_trace(go.Scatter(
    x=speeds, y=Tj_adaptive,
    mode='lines', name='Adaptive Schedule',
    line=dict(color='#2E86AB', width=3)
))
# Add shaded region showing temperature reduction
fig1.add_trace(go.Scatter(
    x=np.concatenate([speeds, speeds[::-1]]),
    y=np.concatenate([Tj_fixed, Tj_adaptive[::-1]]),
    fill='toself', fillcolor='rgba(46, 134, 171, 0.15)',
    line=dict(width=0), showlegend=True,
    name='Temperature Reduction'
))
# Add Tj limit line
fig1.add_hline(y=175, line_dash="dash", line_color="black", line_width=1,
               annotation_text="Tj_max = 175°C", annotation_position="top left")
fig1.add_hline(y=T_coolant, line_dash="dot", line_color="gray", line_width=1,
               annotation_text="T_coolant = 40°C", annotation_position="bottom left")

fig1.update_layout(
    title=dict(text="Steady-State Junction Temperature: Adaptive vs Constant Switching Frequency",
               font=dict(size=16)),
    xaxis_title="Vehicle Speed (km/h)",
    yaxis_title="Junction Temperature (°C)",
    yaxis=dict(range=[35, 80]),
    legend=dict(x=0.55, y=0.95, font=dict(size=12)),
    template="plotly_white",
    width=1200, height=600,
    font=dict(size=12)
)
fig1.write_image(f"{out_dir}/fig1_Tj_steady_state_comparison.png", scale=2)

# --- Figure 2: Transient Thermal Response (Drive Cycle) ---
fig2 = make_subplots(
    rows=3, cols=1, shared_xaxes=True,
    subplot_titles=("Vehicle Speed Profile", "Switching Frequency", "Junction Temperature"),
    vertical_spacing=0.08,
    row_heights=[0.25, 0.25, 0.5]
)

# Speed profile
fig2.add_trace(go.Scatter(x=time_arr, y=speed_profile, mode='lines',
                          line=dict(color='#333333', width=2), name='Speed', showlegend=False),
               row=1, col=1)

# Switching frequency
fig2.add_trace(go.Scatter(x=time_arr, y=np.full_like(time_arr, 20), mode='lines',
                          line=dict(color='#E74C3C', width=2, dash='dash'), name='Fixed 20 kHz'),
               row=2, col=1)
fig2.add_trace(go.Scatter(x=time_arr, y=fsw_profile, mode='lines',
                          line=dict(color='#2E86AB', width=2), name='Adaptive'),
               row=2, col=1)

# Junction temperature
fig2.add_trace(go.Scatter(x=time_arr, y=Tj_fixed_transient, mode='lines',
                          line=dict(color='#E74C3C', width=2.5), name='Tj (Constant 20 kHz)'),
               row=3, col=1)
fig2.add_trace(go.Scatter(x=time_arr, y=Tj_adaptive_transient, mode='lines',
                          line=dict(color='#2E86AB', width=2.5), name='Tj (Adaptive)'),
               row=3, col=1)

fig2.update_yaxes(title_text="Speed (km/h)", row=1, col=1)
fig2.update_yaxes(title_text="fsw (kHz)", row=2, col=1)
fig2.update_yaxes(title_text="Tj (°C)", row=3, col=1)
fig2.update_xaxes(title_text="Time (s)", row=3, col=1)

fig2.update_layout(
    title=dict(text="Transient Thermal Response Over a Mixed Drive Cycle", font=dict(size=16)),
    template="plotly_white",
    width=1200, height=800,
    font=dict(size=11),
    legend=dict(x=0.7, y=0.45, font=dict(size=11))
)
fig2.write_image(f"{out_dir}/fig2_transient_thermal_drive_cycle.png", scale=2)

# --- Figure 3: Temperature Reduction Bar Chart ---
fig3 = go.Figure()
names = [d["operating_point"] for d in comparison_data]
Tj_f_vals = [d["Tj_fixed_C"] for d in comparison_data]
Tj_a_vals = [d["Tj_adaptive_C"] for d in comparison_data]
delta_T = [d["Tj_reduction_C"] for d in comparison_data]

fig3.add_trace(go.Bar(
    x=names, y=Tj_f_vals,
    name='Constant 20 kHz',
    marker_color='#E74C3C',
    text=[f"{v:.1f}°C" for v in Tj_f_vals],
    textposition='outside'
))
fig3.add_trace(go.Bar(
    x=names, y=Tj_a_vals,
    name='Adaptive Schedule',
    marker_color='#2E86AB',
    text=[f"{v:.1f}°C" for v in Tj_a_vals],
    textposition='outside'
))

fig3.update_layout(
    title=dict(text="Junction Temperature at Key Operating Points", font=dict(size=16)),
    yaxis_title="Junction Temperature (°C)",
    yaxis=dict(range=[0, 90]),
    barmode='group',
    template="plotly_white",
    width=1000, height=500,
    font=dict(size=12),
    legend=dict(x=0.65, y=0.95, font=dict(size=12))
)
# Add delta annotations
for i, dt_val in enumerate(delta_T):
    if dt_val > 0:
        fig3.add_annotation(x=names[i], y=max(Tj_f_vals[i], Tj_a_vals[i]) + 6,
                           text=f"ΔTj = {dt_val:.1f}°C", showarrow=False,
                           font=dict(size=11, color='#2E86AB'))
fig3.write_image(f"{out_dir}/fig3_Tj_bar_comparison.png", scale=2)

# --- Figure 4: Loss Breakdown at Each Operating Point ---
fig4 = make_subplots(rows=1, cols=2, subplot_titles=("Constant 20 kHz", "Adaptive Schedule"))

for i, d in enumerate(comparison_data):
    v = d["speed_kmh"]
    Ipk = speed_to_Ipk(v)
    
    P_cond_f = conduction_loss(Ipk, d["Tj_fixed_C"])
    P_sw_f = switching_loss(Ipk, FIXED_FSW)
    P_cond_a = conduction_loss(Ipk, d["Tj_adaptive_C"])
    P_sw_a = switching_loss(Ipk, d["fsw_adaptive_kHz"] * 1000)
    
    fig4.add_trace(go.Bar(x=[d["operating_point"]], y=[P_cond_f],
                          name='Conduction' if i == 0 else None,
                          marker_color='#F39C12', showlegend=(i == 0),
                          legendgroup='cond'), row=1, col=1)
    fig4.add_trace(go.Bar(x=[d["operating_point"]], y=[P_sw_f],
                          name='Switching' if i == 0 else None,
                          marker_color='#E74C3C', showlegend=(i == 0),
                          legendgroup='sw'), row=1, col=1)
    
    fig4.add_trace(go.Bar(x=[d["operating_point"]], y=[P_cond_a],
                          name=None, marker_color='#F39C12', showlegend=False,
                          legendgroup='cond'), row=1, col=2)
    fig4.add_trace(go.Bar(x=[d["operating_point"]], y=[P_sw_a],
                          name=None, marker_color='#E74C3C', showlegend=False,
                          legendgroup='sw'), row=1, col=2)

fig4.update_layout(
    title=dict(text="Loss Breakdown: Conduction vs Switching at Each Operating Point", font=dict(size=14)),
    barmode='stack',
    template="plotly_white",
    width=1200, height=500,
    font=dict(size=11),
    yaxis_title="Loss per Phase (W)",
    yaxis2_title="Loss per Phase (W)"
)
fig4.write_image(f"{out_dir}/fig4_loss_breakdown_comparison.png", scale=2)

# --- Figure 5: Thermal Margin Analysis ---
fig5 = go.Figure()
margin_fixed = [175 - d["Tj_fixed_C"] for d in comparison_data]
margin_adaptive = [175 - d["Tj_adaptive_C"] for d in comparison_data]

fig5.add_trace(go.Bar(
    x=names, y=margin_fixed,
    name='Constant 20 kHz',
    marker_color='#E74C3C',
    text=[f"{v:.0f}°C" for v in margin_fixed],
    textposition='inside', textfont=dict(color='white', size=14)
))
fig5.add_trace(go.Bar(
    x=names, y=margin_adaptive,
    name='Adaptive Schedule',
    marker_color='#2E86AB',
    text=[f"{v:.0f}°C" for v in margin_adaptive],
    textposition='inside', textfont=dict(color='white', size=14)
))

fig5.update_layout(
    title=dict(text="Thermal Safety Margin to Tj_max (175°C)", font=dict(size=16)),
    yaxis_title="Margin (°C)",
    barmode='group',
    template="plotly_white",
    width=1000, height=500,
    font=dict(size=12),
    legend=dict(x=0.65, y=0.95, font=dict(size=12))
)
fig5.write_image(f"{out_dir}/fig5_thermal_margin.png", scale=2)

# --- Figure 6: Tj vs Time with Power Overlay ---
fig6 = make_subplots(rows=2, cols=1, shared_xaxes=True,
                     subplot_titles=("Power Dissipation per Phase", "Junction Temperature"),
                     vertical_spacing=0.12, row_heights=[0.4, 0.6])

fig6.add_trace(go.Scatter(x=time_arr, y=P_fixed_transient, mode='lines',
                          line=dict(color='#E74C3C', width=2), name='P_loss (20 kHz)'),
               row=1, col=1)
fig6.add_trace(go.Scatter(x=time_arr, y=P_adaptive_transient, mode='lines',
                          line=dict(color='#2E86AB', width=2), name='P_loss (Adaptive)'),
               row=1, col=1)

fig6.add_trace(go.Scatter(x=time_arr, y=Tj_fixed_transient, mode='lines',
                          line=dict(color='#E74C3C', width=2.5), name='Tj (20 kHz)'),
               row=2, col=1)
fig6.add_trace(go.Scatter(x=time_arr, y=Tj_adaptive_transient, mode='lines',
                          line=dict(color='#2E86AB', width=2.5), name='Tj (Adaptive)'),
               row=2, col=1)

fig6.update_yaxes(title_text="Power (W)", row=1, col=1)
fig6.update_yaxes(title_text="Tj (°C)", row=2, col=1)
fig6.update_xaxes(title_text="Time (s)", row=2, col=1)

fig6.update_layout(
    title=dict(text="Power Dissipation and Thermal Response: Adaptive vs Constant", font=dict(size=15)),
    template="plotly_white",
    width=1200, height=700,
    font=dict(size=11),
    legend=dict(x=0.65, y=0.95, font=dict(size=11))
)
fig6.write_image(f"{out_dir}/fig6_power_and_thermal.png", scale=2)

# ===========================================================================
# PRINT RESULTS TABLE
# ===========================================================================
print("=" * 100)
print("THERMAL COMPARISON: ADAPTIVE vs CONSTANT 20 kHz SWITCHING FREQUENCY")
print("=" * 100)
print(f"{'Operating Point':<25} {'fe':>7} {'Ipk':>6} {'fsw_adap':>9} | {'Tj(20kHz)':>10} {'Tj(Adap)':>9} {'ΔTj':>6} {'Reduction':>10}")
print("-" * 100)
for d in comparison_data:
    print(f"{d['operating_point']:<25} {d['fe_Hz']:>6.1f}Hz {d['Ipk_A']:>5.1f}A {d['fsw_adaptive_kHz']:>7.0f}kHz | "
          f"{d['Tj_fixed_C']:>8.1f}°C {d['Tj_adaptive_C']:>7.1f}°C {d['Tj_reduction_C']:>5.1f}°C {d['Tj_reduction_pct']:>8.1f}%")

print("\n" + "=" * 100)
print("TRANSIENT ANALYSIS SUMMARY")
print("=" * 100)
print(f"  Peak Tj (Constant 20 kHz): {np.max(Tj_fixed_transient):.1f}°C")
print(f"  Peak Tj (Adaptive):        {np.max(Tj_adaptive_transient):.1f}°C")
print(f"  Peak ΔTj:                   {np.max(Tj_fixed_transient) - np.max(Tj_adaptive_transient):.1f}°C")
print(f"  Mean Tj (Constant 20 kHz): {np.mean(Tj_fixed_transient):.1f}°C")
print(f"  Mean Tj (Adaptive):        {np.mean(Tj_adaptive_transient):.1f}°C")
print(f"  Mean ΔTj:                   {np.mean(Tj_fixed_transient) - np.mean(Tj_adaptive_transient):.1f}°C")

# Lifetime impact estimate (Coffin-Manson approximation)
# Doubling thermal cycling amplitude roughly halves lifetime
delta_T_cycle_fixed = np.max(Tj_fixed_transient) - np.min(Tj_fixed_transient)
delta_T_cycle_adaptive = np.max(Tj_adaptive_transient) - np.min(Tj_adaptive_transient)
print(f"\n  Thermal Cycling Amplitude (Fixed):    {delta_T_cycle_fixed:.1f}°C")
print(f"  Thermal Cycling Amplitude (Adaptive): {delta_T_cycle_adaptive:.1f}°C")
print(f"  Cycling Reduction:                    {delta_T_cycle_fixed - delta_T_cycle_adaptive:.1f}°C ({(1 - delta_T_cycle_adaptive/delta_T_cycle_fixed)*100:.0f}%)")

# Save results
results = {
    "comparison_table": comparison_data,
    "transient": {
        "peak_Tj_fixed_C": round(float(np.max(Tj_fixed_transient)), 1),
        "peak_Tj_adaptive_C": round(float(np.max(Tj_adaptive_transient)), 1),
        "mean_Tj_fixed_C": round(float(np.mean(Tj_fixed_transient)), 1),
        "mean_Tj_adaptive_C": round(float(np.mean(Tj_adaptive_transient)), 1),
        "thermal_cycling_fixed_C": round(float(delta_T_cycle_fixed), 1),
        "thermal_cycling_adaptive_C": round(float(delta_T_cycle_adaptive), 1),
    }
}
with open(f"{out_dir}/thermal_comparison_results.json", 'w') as f:
    json.dump(results, f, indent=2)

print(f"\nAll figures saved to {out_dir}/")
print("Done.")
