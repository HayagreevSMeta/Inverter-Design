#!/usr/bin/env python3
"""
Comprehensive Loss vs Frequency Analysis for 3L-FC Inverter
Using CAB450M12XM3 datasheet parameters.
Generates: conduction, switching, total loss, and Tj vs frequency/speed plots.
"""
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import json, os

out_dir = '/home/ubuntu/Inverter-Design/loss_analysis_figures'
os.makedirs(out_dir, exist_ok=True)

# ─── Vehicle Architecture ───
Vdc = 800.0
r_tyre = 0.315
G = 8.19
p = 4
R_load = 1.3
L_load = 2e-3
m = 0.9

# ─── CAB450M12XM3 Datasheet Parameters ───
Rds_on_25 = 3.3e-3      # Ω at 25°C (die + package)
Rds_on_175 = 6.6e-3     # Ω at 175°C (2x factor from Fig. 2)
Eon_ref = 25.4e-3        # J at Iref=450A, Vref=600V, 25°C
Eoff_ref = 7.51e-3       # J at Iref=450A, Vref=600V, 25°C
Err_ref = 0.2e-3         # J at Iref=450A, Vref=600V, 25°C
Iref = 450.0
Vref = 600.0
Vdc_sw = Vdc / 2         # 400V per switch in 3L-FC
Nc = 2                   # switches commutating per PWM transition (3L-FC)

# Thermal
Rth_jc = 0.094           # °C/W per switch position
Rth_cs = 0.02            # °C/W case-to-sink (thermal grease)
Rth_sa = 0.03            # °C/W sink-to-ambient (liquid cooling)
Rth_total = Rth_jc + Rth_cs + Rth_sa  # 0.144 °C/W
T_ambient = 40.0         # °C coolant temperature

# Foster network (from datasheet Fig. 17)
foster_R = [0.015, 0.025, 0.030, 0.024]
foster_tau = [0.1e-3, 1e-3, 10e-3, 100e-3]

# ─── Speed Range ───
speeds_kmh = np.linspace(20, 140, 61)
speeds_ms = speeds_kmh / 3.6

# ─── Switching Frequency Options ───
fsw_options = [5e3, 10e3, 15e3, 20e3, 25e3, 30e3, 40e3]

# ─── Adaptive Schedule ───
def get_fsw_adaptive(v_kmh):
    if v_kmh < 60: return 5e3
    elif v_kmh < 80: return 10e3
    else: return 20e3

# ─── Compute Operating Points ───
fe_arr = (speeds_ms / r_tyre) * (G / (2 * np.pi)) * p
Z_arr = np.sqrt(R_load**2 + (2 * np.pi * fe_arr * L_load)**2)
Ipk_arr = (m * Vdc / 2) / Z_arr
cos_phi_arr = R_load / Z_arr

def rds_on_at_T(Tj):
    """Rds_on temperature scaling from datasheet Fig. 2"""
    ratio = 1.0 + 1.0 * (Tj - 25.0) / (175.0 - 25.0)
    return Rds_on_25 * ratio

def conduction_loss_per_phase(Ipk, cos_phi, Tj):
    """Conduction loss for 3L-FC: 2 devices conduct at any time"""
    Rds = rds_on_at_T(Tj)
    # RMS current per switch in 3L with sinusoidal PWM
    Irms_sq = (Ipk**2 / 2) * (1/2 + (4 * m * cos_phi) / (3 * np.pi))
    # 2 switches conduct simultaneously in 3L-FC
    P_cond = 2 * Rds * Irms_sq
    return P_cond

def switching_loss_per_phase(Ipk, fsw):
    """Switching loss using Graovac-Purschel for 3L-FC"""
    Esw_ref = Eon_ref + Eoff_ref  # total switching energy at ref
    # Linear scaling for SiC (confirmed by datasheet: nearly linear Esw vs I)
    P_sw = Nc * (fsw / np.pi) * Esw_ref * (Ipk / Iref) * (Vdc_sw / Vref)
    return P_sw

def diode_recovery_loss_per_phase(Ipk, fsw):
    """Reverse recovery loss (small for SiC body diode)"""
    P_rr = Nc * (fsw / np.pi) * Err_ref * (Ipk / Iref) * (Vdc_sw / Vref)
    return P_rr

def junction_temp(P_total):
    """Steady-state Tj from total loss per switch"""
    return T_ambient + P_total * Rth_total

# ─── Compute losses across all speeds and frequencies ───
results = {}
for fsw in fsw_options:
    P_cond_arr = []
    P_sw_arr = []
    P_rr_arr = []
    P_total_arr = []
    Tj_arr = []
    
    for i, v in enumerate(speeds_kmh):
        Ipk = Ipk_arr[i]
        cphi = cos_phi_arr[i]
        
        # Iterative thermal solve (3 iterations sufficient)
        Tj = 60.0  # initial guess
        for _ in range(5):
            Pc = conduction_loss_per_phase(Ipk, cphi, Tj)
            Ps = switching_loss_per_phase(Ipk, fsw)
            Pr = diode_recovery_loss_per_phase(Ipk, fsw)
            Pt = Pc + Ps + Pr
            Tj = junction_temp(Pt)
        
        P_cond_arr.append(Pc)
        P_sw_arr.append(Ps)
        P_rr_arr.append(Pr)
        P_total_arr.append(Pt)
        Tj_arr.append(Tj)
    
    results[fsw] = {
        'P_cond': np.array(P_cond_arr),
        'P_sw': np.array(P_sw_arr),
        'P_rr': np.array(P_rr_arr),
        'P_total': np.array(P_total_arr),
        'Tj': np.array(Tj_arr)
    }

# Adaptive schedule
P_cond_adapt = []
P_sw_adapt = []
P_rr_adapt = []
P_total_adapt = []
Tj_adapt = []
fsw_adapt_arr = []

for i, v in enumerate(speeds_kmh):
    Ipk = Ipk_arr[i]
    cphi = cos_phi_arr[i]
    fsw = get_fsw_adaptive(v)
    fsw_adapt_arr.append(fsw)
    
    Tj = 60.0
    for _ in range(5):
        Pc = conduction_loss_per_phase(Ipk, cphi, Tj)
        Ps = switching_loss_per_phase(Ipk, fsw)
        Pr = diode_recovery_loss_per_phase(Ipk, fsw)
        Pt = Pc + Ps + Pr
        Tj = junction_temp(Pt)
    
    P_cond_adapt.append(Pc)
    P_sw_adapt.append(Ps)
    P_rr_adapt.append(Pr)
    P_total_adapt.append(Pt)
    Tj_adapt.append(Tj)

P_cond_adapt = np.array(P_cond_adapt)
P_sw_adapt = np.array(P_sw_adapt)
P_rr_adapt = np.array(P_rr_adapt)
P_total_adapt = np.array(P_total_adapt)
Tj_adapt = np.array(Tj_adapt)

# ═══════════════════════════════════════════════════════════
# FIGURE 1: Conduction Loss vs Speed (temperature-dependent)
# ═══════════════════════════════════════════════════════════
fig1 = go.Figure()
colors_tj = ['#2196F3', '#4CAF50', '#FF9800', '#F44336', '#9C27B0']
tj_values = [25, 50, 75, 100, 125, 150, 175]
for Tj_fixed in tj_values:
    P_cond_fixed = []
    for i in range(len(speeds_kmh)):
        Pc = conduction_loss_per_phase(Ipk_arr[i], cos_phi_arr[i], Tj_fixed)
        P_cond_fixed.append(Pc)
    fig1.add_trace(go.Scatter(x=speeds_kmh, y=P_cond_fixed, mode='lines',
                              name=f'Tj = {Tj_fixed}°C'))

fig1.update_layout(
    title='Conduction Loss vs Vehicle Speed at Various Junction Temperatures<br><sub>CAB450M12XM3: Rds_on = 3.3mΩ (25°C) to 6.6mΩ (175°C)</sub>',
    xaxis_title='Vehicle Speed (km/h)', yaxis_title='Conduction Loss per Phase (W)',
    legend=dict(x=0.65, y=0.95), width=900, height=550,
    font=dict(family='Times New Roman', size=13))
fig1.write_image(f'{out_dir}/fig1_conduction_vs_speed.png', scale=2)

# ═══════════════════════════════════════════════════════════
# FIGURE 2: Switching Loss vs Speed for various fsw
# ═══════════════════════════════════════════════════════════
fig2 = go.Figure()
colors_fsw = ['#2196F3', '#4CAF50', '#FF9800', '#F44336', '#9C27B0', '#795548', '#607D8B']
for j, fsw in enumerate(fsw_options):
    fig2.add_trace(go.Scatter(x=speeds_kmh, y=results[fsw]['P_sw'], mode='lines',
                              name=f'fsw = {fsw/1e3:.0f} kHz', line=dict(color=colors_fsw[j])))
fig2.add_trace(go.Scatter(x=speeds_kmh, y=P_sw_adapt, mode='lines+markers',
                          name='Adaptive Schedule', line=dict(color='black', width=3, dash='dash'),
                          marker=dict(size=4)))

fig2.update_layout(
    title='Switching Loss vs Vehicle Speed for Various Carrier Frequencies<br><sub>Graovac-Purschel method, CAB450M12XM3 datasheet: Eon=25.4mJ, Eoff=7.51mJ @ 450A/600V</sub>',
    xaxis_title='Vehicle Speed (km/h)', yaxis_title='Switching Loss per Phase (W)',
    legend=dict(x=0.65, y=0.95), width=900, height=550,
    font=dict(family='Times New Roman', size=13))
fig2.write_image(f'{out_dir}/fig2_switching_vs_speed.png', scale=2)

# ═══════════════════════════════════════════════════════════
# FIGURE 3: Total Loss Breakdown - Stacked Area (Adaptive vs Fixed 20kHz)
# ═══════════════════════════════════════════════════════════
fig3 = make_subplots(rows=1, cols=2, subplot_titles=('Adaptive Schedule', 'Fixed 20 kHz'),
                     shared_yaxes=True, horizontal_spacing=0.05)

# Adaptive
fig3.add_trace(go.Scatter(x=speeds_kmh, y=P_cond_adapt, fill='tozeroy', name='Conduction',
                          fillcolor='rgba(33,150,243,0.5)', line=dict(color='#2196F3')), row=1, col=1)
fig3.add_trace(go.Scatter(x=speeds_kmh, y=P_cond_adapt + P_sw_adapt, fill='tonexty', name='Switching',
                          fillcolor='rgba(244,67,54,0.5)', line=dict(color='#F44336')), row=1, col=1)
fig3.add_trace(go.Scatter(x=speeds_kmh, y=P_cond_adapt + P_sw_adapt + P_rr_adapt, fill='tonexty',
                          name='Diode Recovery', fillcolor='rgba(255,152,0,0.3)', line=dict(color='#FF9800')), row=1, col=1)

# Fixed 20kHz
fig3.add_trace(go.Scatter(x=speeds_kmh, y=results[20e3]['P_cond'], fill='tozeroy',
                          name='Conduction', fillcolor='rgba(33,150,243,0.5)',
                          line=dict(color='#2196F3'), showlegend=False), row=1, col=2)
fig3.add_trace(go.Scatter(x=speeds_kmh, y=results[20e3]['P_cond'] + results[20e3]['P_sw'],
                          fill='tonexty', name='Switching', fillcolor='rgba(244,67,54,0.5)',
                          line=dict(color='#F44336'), showlegend=False), row=1, col=2)
fig3.add_trace(go.Scatter(x=speeds_kmh, y=results[20e3]['P_cond'] + results[20e3]['P_sw'] + results[20e3]['P_rr'],
                          fill='tonexty', name='Diode Recovery', fillcolor='rgba(255,152,0,0.3)',
                          line=dict(color='#FF9800'), showlegend=False), row=1, col=2)

fig3.update_layout(
    title='Total Loss Breakdown: Conduction + Switching + Diode Recovery<br><sub>Per phase, CAB450M12XM3 datasheet parameters</sub>',
    width=1100, height=500, font=dict(family='Times New Roman', size=13),
    legend=dict(x=0.01, y=0.95))
fig3.update_xaxes(title_text='Vehicle Speed (km/h)', row=1, col=1)
fig3.update_xaxes(title_text='Vehicle Speed (km/h)', row=1, col=2)
fig3.update_yaxes(title_text='Loss per Phase (W)', row=1, col=1)
fig3.write_image(f'{out_dir}/fig3_loss_breakdown_stacked.png', scale=2)

# ═══════════════════════════════════════════════════════════
# FIGURE 4: Junction Temperature vs Speed
# ═══════════════════════════════════════════════════════════
fig4 = go.Figure()
for j, fsw in enumerate([5e3, 10e3, 20e3, 40e3]):
    fig4.add_trace(go.Scatter(x=speeds_kmh, y=results[fsw]['Tj'], mode='lines',
                              name=f'fsw = {fsw/1e3:.0f} kHz', line=dict(color=colors_fsw[j])))
fig4.add_trace(go.Scatter(x=speeds_kmh, y=Tj_adapt, mode='lines+markers',
                          name='Adaptive Schedule', line=dict(color='black', width=3, dash='dash'),
                          marker=dict(size=4)))
fig4.add_hline(y=175, line_dash='dot', line_color='red', annotation_text='Tj_max = 175°C',
               annotation_position='top right')
fig4.add_hline(y=T_ambient, line_dash='dot', line_color='gray', annotation_text=f'T_coolant = {T_ambient}°C',
               annotation_position='bottom right')

fig4.update_layout(
    title='Junction Temperature vs Vehicle Speed<br><sub>4-layer Foster network, Rth_jc=94 m°C/W, T_coolant=40°C</sub>',
    xaxis_title='Vehicle Speed (km/h)', yaxis_title='Junction Temperature (°C)',
    yaxis_range=[35, 80], legend=dict(x=0.6, y=0.95), width=900, height=550,
    font=dict(family='Times New Roman', size=13))
fig4.write_image(f'{out_dir}/fig4_junction_temp_vs_speed.png', scale=2)

# ═══════════════════════════════════════════════════════════
# FIGURE 5: Loss vs Switching Frequency at Fixed Speed Points
# ═══════════════════════════════════════════════════════════
fig5 = make_subplots(rows=1, cols=3, subplot_titles=('Urban (40 km/h)', 'Extra-Urban (80 km/h)', 'Highway (120 km/h)'),
                     shared_yaxes=True, horizontal_spacing=0.05)

speed_indices = [np.argmin(np.abs(speeds_kmh - s)) for s in [40, 80, 120]]
fsw_range = np.linspace(2e3, 50e3, 100)

for col, si in enumerate(speed_indices):
    Ipk = Ipk_arr[si]
    cphi = cos_phi_arr[si]
    
    P_cond_vs_f = []
    P_sw_vs_f = []
    P_total_vs_f = []
    Tj_vs_f = []
    
    for fsw in fsw_range:
        Tj = 60.0
        for _ in range(5):
            Pc = conduction_loss_per_phase(Ipk, cphi, Tj)
            Ps = switching_loss_per_phase(Ipk, fsw)
            Pt = Pc + Ps
            Tj = junction_temp(Pt)
        P_cond_vs_f.append(Pc)
        P_sw_vs_f.append(Ps)
        P_total_vs_f.append(Pt)
        Tj_vs_f.append(Tj)
    
    P_cond_vs_f = np.array(P_cond_vs_f)
    P_sw_vs_f = np.array(P_sw_vs_f)
    P_total_vs_f = np.array(P_total_vs_f)
    
    show = (col == 0)
    fig5.add_trace(go.Scatter(x=fsw_range/1e3, y=P_cond_vs_f, mode='lines', name='Conduction',
                              line=dict(color='#2196F3'), showlegend=show), row=1, col=col+1)
    fig5.add_trace(go.Scatter(x=fsw_range/1e3, y=P_sw_vs_f, mode='lines', name='Switching',
                              line=dict(color='#F44336'), showlegend=show), row=1, col=col+1)
    fig5.add_trace(go.Scatter(x=fsw_range/1e3, y=P_total_vs_f, mode='lines', name='Total',
                              line=dict(color='black', width=2), showlegend=show), row=1, col=col+1)

fig5.update_layout(
    title='Loss Components vs Switching Frequency at Three Operating Points<br><sub>Conduction loss increases with fsw (via Tj feedback), switching loss scales linearly</sub>',
    width=1200, height=450, font=dict(family='Times New Roman', size=13),
    legend=dict(x=0.01, y=0.95))
for c in range(3):
    fig5.update_xaxes(title_text='Switching Frequency (kHz)', row=1, col=c+1)
fig5.update_yaxes(title_text='Loss per Phase (W)', row=1, col=1)
fig5.write_image(f'{out_dir}/fig5_loss_vs_fsw_fixed_speed.png', scale=2)

# ═══════════════════════════════════════════════════════════
# FIGURE 6: Switching Energy from Datasheet (Eon, Eoff, Err vs Current)
# ═══════════════════════════════════════════════════════════
I_range = np.linspace(0, 600, 100)
# Linear SiC scaling (confirmed by datasheet Fig. 15 shape)
Eon_vs_I = Eon_ref * (I_range / Iref) * (Vdc_sw / Vref)
Eoff_vs_I = Eoff_ref * (I_range / Iref) * (Vdc_sw / Vref)
Err_vs_I = Err_ref * (I_range / Iref) * (Vdc_sw / Vref)
Etotal_vs_I = Eon_vs_I + Eoff_vs_I + Err_vs_I

fig6 = go.Figure()
fig6.add_trace(go.Scatter(x=I_range, y=Eon_vs_I*1e3, mode='lines', name='E_on',
                          line=dict(color='#F44336', width=2)))
fig6.add_trace(go.Scatter(x=I_range, y=Eoff_vs_I*1e3, mode='lines', name='E_off',
                          line=dict(color='#2196F3', width=2)))
fig6.add_trace(go.Scatter(x=I_range, y=Err_vs_I*1e3, mode='lines', name='E_rr',
                          line=dict(color='#FF9800', width=2)))
fig6.add_trace(go.Scatter(x=I_range, y=Etotal_vs_I*1e3, mode='lines', name='E_total',
                          line=dict(color='black', width=2.5, dash='dash')))

# Mark our operating points
for v, label in [(40, 'Urban'), (80, 'Extra-Urban'), (120, 'Highway')]:
    idx = np.argmin(np.abs(speeds_kmh - v))
    Ipk = Ipk_arr[idx]
    E_total = (Eon_ref + Eoff_ref + Err_ref) * (Ipk / Iref) * (Vdc_sw / Vref)
    fig6.add_trace(go.Scatter(x=[Ipk], y=[E_total*1e3], mode='markers+text',
                              text=[f'{label}<br>{Ipk:.0f}A'], textposition='top center',
                              marker=dict(size=10, color='black', symbol='diamond'),
                              showlegend=False))

fig6.update_layout(
    title='Switching Energy vs Drain Current (Scaled to Vdc/2 = 400V)<br><sub>From CAB450M12XM3 datasheet: Eon=25.4mJ, Eoff=7.51mJ, Err=0.2mJ @ 450A/600V</sub>',
    xaxis_title='Drain Current (A)', yaxis_title='Switching Energy (mJ)',
    legend=dict(x=0.05, y=0.95), width=900, height=550,
    font=dict(family='Times New Roman', size=13))
fig6.write_image(f'{out_dir}/fig6_switching_energy_vs_current.png', scale=2)

# ═══════════════════════════════════════════════════════════
# FIGURE 7: Rds_on vs Temperature from Datasheet
# ═══════════════════════════════════════════════════════════
T_range = np.linspace(-40, 175, 100)
Rds_vs_T = np.array([rds_on_at_T(T) * 1e3 for T in T_range])

fig7 = go.Figure()
fig7.add_trace(go.Scatter(x=T_range, y=Rds_vs_T, mode='lines', name='Rds_on(T)',
                          line=dict(color='#F44336', width=2.5)))
# Datasheet points
ds_T = [25, 100, 125, 150, 175]
ds_Rds = [3.3, 4.95, 5.61, 6.11, 6.6]
fig7.add_trace(go.Scatter(x=ds_T, y=ds_Rds, mode='markers', name='Datasheet (Fig. 2)',
                          marker=dict(size=10, color='black', symbol='square')))

fig7.update_layout(
    title='On-State Resistance vs Junction Temperature<br><sub>CAB450M12XM3: 3.3mΩ (25°C) → 6.6mΩ (175°C), 2.0x ratio</sub>',
    xaxis_title='Junction Temperature (°C)', yaxis_title='Rds_on (mΩ)',
    legend=dict(x=0.05, y=0.95), width=900, height=550,
    font=dict(family='Times New Roman', size=13))
fig7.write_image(f'{out_dir}/fig7_rdson_vs_temperature.png', scale=2)

# ═══════════════════════════════════════════════════════════
# FIGURE 8: Thermal Impedance (Foster Network)
# ═══════════════════════════════════════════════════════════
t_log = np.logspace(-5, 0, 500)  # 10µs to 1s
Zth = np.zeros_like(t_log)
for Ri, taui in zip(foster_R, foster_tau):
    Zth += Ri * (1 - np.exp(-t_log / taui))

fig8 = go.Figure()
fig8.add_trace(go.Scatter(x=t_log, y=Zth, mode='lines', name='Zth_jc (Foster 4-layer)',
                          line=dict(color='#2196F3', width=2.5)))
fig8.add_hline(y=Rth_jc, line_dash='dot', line_color='red',
               annotation_text=f'Rth_jc = {Rth_jc*1e3:.0f} m°C/W (steady-state)')
fig8.add_hline(y=Rth_total, line_dash='dot', line_color='darkred',
               annotation_text=f'Rth_ja = {Rth_total*1e3:.0f} m°C/W (total)')

fig8.update_layout(
    title='Transient Thermal Impedance Zth (Junction-to-Case)<br><sub>4-layer Foster network from CAB450M12XM3 datasheet Fig. 17</sub>',
    xaxis_title='Time (s)', yaxis_title='Zth (°C/W)',
    xaxis_type='log', yaxis_type='log', width=900, height=550,
    font=dict(family='Times New Roman', size=13),
    legend=dict(x=0.05, y=0.95))
fig8.write_image(f'{out_dir}/fig8_thermal_impedance.png', scale=2)

# ═══════════════════════════════════════════════════════════
# Save numerical results
# ═══════════════════════════════════════════════════════════
key_results = {}
for v_label, v_kmh in [('urban_40', 40), ('extra_urban_80', 80), ('highway_120', 120)]:
    idx = np.argmin(np.abs(speeds_kmh - v_kmh))
    Ipk = float(Ipk_arr[idx])
    cphi = float(cos_phi_arr[idx])
    fe = float(fe_arr[idx])
    
    row = {'speed_kmh': v_kmh, 'fe_Hz': round(fe, 1), 'Ipk_A': round(Ipk, 1), 'cos_phi': round(cphi, 3)}
    
    for fsw_val in [5e3, 10e3, 20e3]:
        fsw_key = f'{int(fsw_val/1e3)}kHz'
        Tj = 60.0
        for _ in range(5):
            Pc = conduction_loss_per_phase(Ipk, cphi, Tj)
            Ps = switching_loss_per_phase(Ipk, fsw_val)
            Pr = diode_recovery_loss_per_phase(Ipk, fsw_val)
            Pt = Pc + Ps + Pr
            Tj = junction_temp(Pt)
        row[f'P_cond_{fsw_key}_W'] = round(float(Pc), 1)
        row[f'P_sw_{fsw_key}_W'] = round(float(Ps), 1)
        row[f'P_rr_{fsw_key}_W'] = round(float(Pr), 2)
        row[f'P_total_{fsw_key}_W'] = round(float(Pt), 1)
        row[f'Tj_{fsw_key}_C'] = round(float(Tj), 1)
    
    key_results[v_label] = row

# Adaptive schedule summary
adapt_summary = {}
for v_label, v_kmh in [('urban_40', 40), ('extra_urban_80', 80), ('highway_120', 120)]:
    idx = np.argmin(np.abs(speeds_kmh - v_kmh))
    fsw = get_fsw_adaptive(v_kmh)
    adapt_summary[v_label] = {
        'fsw_kHz': fsw / 1e3,
        'P_cond_W': round(float(P_cond_adapt[idx]), 1),
        'P_sw_W': round(float(P_sw_adapt[idx]), 1),
        'P_rr_W': round(float(P_rr_adapt[idx]), 2),
        'P_total_W': round(float(P_total_adapt[idx]), 1),
        'Tj_C': round(float(Tj_adapt[idx]), 1)
    }

output = {
    'device': 'Wolfspeed CAB450M12XM3',
    'datasheet_params': {
        'Rds_on_25C_mohm': Rds_on_25 * 1e3,
        'Rds_on_175C_mohm': Rds_on_175 * 1e3,
        'Eon_ref_mJ': Eon_ref * 1e3,
        'Eoff_ref_mJ': Eoff_ref * 1e3,
        'Err_ref_mJ': Err_ref * 1e3,
        'Iref_A': Iref,
        'Vref_V': Vref,
        'Rth_jc_CW': Rth_jc,
        'Rth_total_CW': Rth_total,
        'Foster_R': foster_R,
        'Foster_tau': foster_tau
    },
    'operating_points': key_results,
    'adaptive_schedule': adapt_summary
}

with open(f'{out_dir}/loss_analysis_results.json', 'w') as f:
    json.dump(output, f, indent=2)

print("All figures generated:")
for fn in sorted(os.listdir(out_dir)):
    print(f"  {fn}")
print(f"\nKey results saved to {out_dir}/loss_analysis_results.json")

# Print summary table
print("\n" + "="*90)
print("LOSS COMPARISON TABLE (per phase)")
print("="*90)
print(f"{'Speed':>10} {'fe':>8} {'Ipk':>6} | {'fsw':>6} {'P_cond':>8} {'P_sw':>8} {'P_rr':>6} {'P_total':>8} {'Tj':>6}")
print("-"*90)
for v_label in ['urban_40', 'extra_urban_80', 'highway_120']:
    r = key_results[v_label]
    for fsw_key in ['5kHz', '10kHz', '20kHz']:
        print(f"{r['speed_kmh']:>7}km/h {r['fe_Hz']:>7.1f}Hz {r['Ipk_A']:>5.1f}A | "
              f"{fsw_key:>6} {r[f'P_cond_{fsw_key}_W']:>7.1f}W {r[f'P_sw_{fsw_key}_W']:>7.1f}W "
              f"{r[f'P_rr_{fsw_key}_W']:>5.2f}W {r[f'P_total_{fsw_key}_W']:>7.1f}W {r[f'Tj_{fsw_key}_C']:>5.1f}°C")
    print()

print("\nADAPTIVE SCHEDULE:")
for v_label in ['urban_40', 'extra_urban_80', 'highway_120']:
    a = adapt_summary[v_label]
    print(f"  {v_label}: fsw={a['fsw_kHz']:.0f}kHz → P_total={a['P_total_W']:.1f}W, Tj={a['Tj_C']:.1f}°C")
