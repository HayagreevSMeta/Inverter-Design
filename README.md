# Three-Level Flying Capacitor Inverter for 800V EV Traction

## Overview

This repository contains the specification, simulation, and analysis of a **three-phase three-level Flying Capacitor (FC) inverter** designed for 800V electric vehicle traction applications with integrated DC charging capability.

## Project Structure

```
Inverter-Design/
├── fc_inverter_sim.py              # Python simulation (primary, runnable)
├── FC_Inverter_Run.m               # MATLAB simulation (equivalent implementation)
├── IEEE_FC_Inverter_Final.md       # IEEE conference paper (Markdown)
├── IEEE_FC_Inverter_Final.pdf      # IEEE conference paper (PDF)
├── FC_Topology_Selection_Updated.docx  # Topology selection analysis
├── Project1.docx                   # Original project specification
├── fc_inverter_results/            # Simulation output directory
│   ├── fig1_drive_cycle.png        # Drive cycle & frequency schedule
│   ├── fig2_output_voltages.png    # Three-level output voltages
│   ├── fig3_phase_currents.png     # Phase currents & spectrum
│   ├── fig4_fc_voltages.png        # FC voltage balancing
│   ├── fig5_thermal.png            # Losses & junction temperature
│   ├── fig6_thd_spectra.png        # THD harmonic spectra
│   └── results.json                # Structured numerical results
└── README.md                       # This file
```

## Key Specifications

| Parameter | Value |
|-----------|-------|
| DC Bus Voltage | 800 V (±400 V rails) |
| Topology | Three-level Flying Capacitor |
| Modulation | Phase Disposition PWM (PD-PWM) |
| Device Technology | SiC MOSFET Power Module (1200V/450A class) |
| Flying Capacitor | 1.5 mF per phase (MKP film) |
| FC Reference Voltage | 400 V (= Vdc/2) |
| Switching Frequency | 5 kHz (urban) → 20 kHz (highway) |
| Load | R = 1.3 Ω, L = 2 mH |
| Motor | 8-pole PMSM, gear ratio 8.19:1 |

## Simulation Features

The simulation implements the following models:

- **Dynamic FC voltage model** with correct charge/discharge physics and ESR
- **PD-PWM modulation** with natural zero-state redundancy selection for FC balancing
- **Nonlinear switching energy** model (quadratic current scaling from datasheet)
- **4-layer Foster thermal network** for transient junction temperature estimation
- **Temperature-dependent Rds_on** (SiC characteristic: ~1.8x at 150C)
- **Dead-time voltage distortion** modelling
- **Hanning-windowed THD** computation (prevents spectral leakage)
- **Adaptive switching frequency scheduling** (speed-dependent)

## Key Results

| Metric | Value |
|--------|-------|
| THD (Urban, 5 kHz) | 1.90% |
| THD (Highway, 20 kHz) | 1.62% |
| FC Voltage Balance | 400.05 V mean (ref = 400 V) |
| FC Ripple (Urban) | 8.3 V pk-pk |
| FC Ripple (Highway) | 0.93 V pk-pk |
| Switching Loss Saving | 40.7% (scheduled vs fixed 20 kHz) |
| Max Junction Temp | 48.3 C (scheduled) / 49.8 C (fixed) |

## Running the Simulation

### Python (recommended)
```bash
python3 fc_inverter_sim.py
```
Requires: `numpy`, `matplotlib` (standard scientific Python stack).

### MATLAB
```matlab
FC_Inverter_Run
```
Requires: MATLAB R2020b or later (no additional toolboxes needed).

## Author

Hayagreev S.

## License

This project is for academic/educational purposes.
