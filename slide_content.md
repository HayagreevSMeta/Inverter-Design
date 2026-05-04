# 800V Three-Level Flying Capacitor Traction Inverter
## Presentation Slide Content

---

## Slide 1: Title Slide
**Heading:** Vehicle Architecture-Driven Design and Comprehensive Loss Analysis of an 800V Three-Level Flying Capacitor Traction Inverter
**Author:** Hayagreev S.
**Affiliation:** Department of Electrical Engineering
**Visual:** Clean title layout with the FC inverter schematic as a subtle background element

---

## Slide 2: Why 800V? The Case for High-Voltage EV Architecture
**Heading:** Doubling bus voltage halves current, enabling faster charging and lighter cables

**Key Points:**
- 800V enables ultra-fast DC charging at 350+ kW (vs. 150 kW at 400V), reducing charge time from 45 min to under 20 min
- Halving current (I = P/V) reduces I²R losses by 75% in the inverter, motor, and cabling
- Copper cross-section reduced by 50%, saving 10-15 kg of vehicle weight
- Industry adoption: Porsche Taycan, Hyundai E-GMP, Lucid Air all use 800V platforms
- Challenge: Higher dv/dt stress on motor insulation demands multilevel topology solutions

---

## Slide 3: The dv/dt Problem — Why Multilevel?
**Heading:** Two-level inverters at 800V produce 800V voltage steps that destroy motor insulation

**Key Points:**
- 2-level VSI at 800V: full bus voltage switched in <50ns → dv/dt > 16 kV/µs
- Motor insulation rated for 1200V/µs maximum — 13x exceeded
- Partial discharge and insulation breakdown within 1000 hours of operation
- 3-level topology solution: voltage step = Vdc/2 = 400V → dv/dt halved to 8 kV/µs
- Additional benefit: output THD improved, EMI reduced, smaller output filter

---

## Slide 4: Topology Selection — FC vs NPC
**Heading:** Flying Capacitor chosen over NPC for thermal symmetry and natural balancing

**Key Points:**
- NPC: Unequal loss distribution between inner/outer switches → asymmetric heating, complex cooling
- NPC: Neutral point balance is difficult under transient EV loads, requires complex control
- FC: All 4 switches block identical voltage (Vdc/2 = 400V) → symmetric thermal stress
- FC: Redundant switching states enable natural capacitor balancing regardless of power factor
- FC trade-off: Requires bulky capacitor and pre-charge circuit (acceptable in modern EVs)

**Visual:** Side-by-side comparison diagram or the FC schematic from Fig. 1

---

## Slide 5: FC Inverter Topology and Switching States
**Heading:** Four switches and one flying capacitor produce three output voltage levels

**Key Points:**
- Single phase leg: S1-S2-S3-S4 in series, Cfc connected between S1/S2 and S3/S4 junctions
- Output voltage levels: +Vdc/2, 0, -Vdc/2
- Two redundant zero states: S1=1,S2=0,S3=1,S4=0 (charges Cfc) and S1=0,S2=1,S3=0,S4=1 (discharges Cfc)
- PD-PWM modulation with active state selection based on current direction
- Pre-charge required: 20ms ramp to Vfc = 400V before switching

**Visual:** The FC inverter schematic diagram (fc_inverter_schematic.png)

---

## Slide 6: Component Selection — Driven by Vehicle Architecture
**Heading:** 1200V/450A SiC module and 680µF film capacitor selected from first principles

**Key Points:**
- Switch: Wolfspeed CAB450M12XM3 — 1200V (50% margin over 400V/switch), 450A (3.3x over 136A peak)
- SiC chosen over Si IGBT: 10x lower switching energy, no tail current, enables 20 kHz operation
- Capacitor sizing: C = Ipk / (2 × fsw_min × ΔVfc) = 136A / (2 × 5kHz × 20V) = 680µF
- Film capacitor (polypropylene): ESR < 5mΩ, >100A RMS ripple, self-healing, automotive-grade
- Thermal: Rth_jc = 0.094 °C/W, 4-layer Foster network for transient modeling

---

## Slide 7: Vehicle Speed Maps to Inverter Operating Points
**Heading:** Urban driving demands 136A at 184 Hz; highway requires only 51A at 552 Hz

**Key Points:**
- Drivetrain: G = 8.19 gear ratio, r = 0.315m tyre, p = 4 pole pairs
- Electrical frequency: fe = (v/r) × (G/2π) × p
- RL load model: R = 1.3Ω, L = 2mH → current decreases with speed (impedance increases)
- Urban (40 km/h): fe = 184 Hz, Ipk = 136A — highest thermal stress
- Highway (120 km/h): fe = 552 Hz, Ipk = 51A — lowest loss but highest frequency demand

---

## Slide 8: Switching Loss Dominates at Low Speed with Fixed Frequency
**Heading:** At fixed 20 kHz, switching losses reach 84W/phase during urban driving — 63% of total loss

**Key Points:**
- Graovac-Purschel method: Psw = Nc × (fsw/π) × Esw_ref × (Ipk/Iref) × (Vdc_sw/Vref)
- Urban at 20 kHz: P_sw = 84.3 W/phase vs P_cond = 51.5 W/phase → switching dominates
- Highway at 20 kHz: P_sw = 31.7 W/phase vs P_cond = 5.6 W/phase → both low
- Key insight: 20 kHz is unnecessary at 40 km/h where fe is only 184 Hz
- Minimum carrier ratio mf = fsw/fe = 27 at 5 kHz/184 Hz is sufficient for good THD

**Visual:** Switching loss vs speed figure (fig2_switching_vs_speed.png)

---

## Slide 9: Adaptive Switching Frequency Reduces Urban Losses by 75%
**Heading:** Speed-dependent fsw schedule saves 190W (3-phase) at 40 km/h with no THD penalty

**Key Points:**
- Schedule: 5 kHz (v < 60 km/h), 10 kHz (60-90 km/h), 20 kHz (v > 90 km/h)
- Urban saving: 84.3W → 21.1W per phase (75% reduction)
- 3-phase total saving at 40 km/h: 190W
- THD remains excellent: 1.92% at 5 kHz (urban), 1.13% at 20 kHz (highway)
- Highway: no change (both use 20 kHz), losses already low

**Visual:** Loss breakdown stacked chart (fig3_loss_breakdown_stacked.png)

---

## Slide 10: Thermal Impact — 9.7°C Junction Temperature Reduction
**Heading:** Adaptive schedule reduces Tj from 63°C to 54°C in urban driving, extending module lifetime

**Key Points:**
- Urban Tj: 63.2°C (fixed) → 53.5°C (adaptive) — ΔTj = 9.7°C
- Peak torque (20 km/h): 87.4°C → 71.3°C — ΔTj = 16.2°C
- Thermal cycling amplitude reduced by 53% (16.4°C → 7.7°C)
- Reduced cycling directly extends solder fatigue lifetime (Coffin-Manson model)
- All operating points remain well below Tj_max = 175°C (>100°C margin)

**Visual:** Steady-state Tj comparison figure (fig1_Tj_steady_state_comparison.png)

---

## Slide 11: Simulation Validates Design — Waveforms and FC Balancing
**Heading:** Time-domain simulation confirms proper 3-level operation and natural FC voltage balancing

**Key Points:**
- Three-level output voltage successfully synthesized at both urban and highway speeds
- Phase currents are clean sinusoids with THD < 2%
- FC voltage maintained at 400V ± 0.04V mean error
- FC ripple: 18V simulated vs 20V analytical budget — excellent agreement
- Pre-charge transient: 20ms linear ramp from 0V to 400V before active switching

**Visual:** FC voltage balancing figure (fig4_fc_voltages.png) or phase currents (fig3_phase_currents.png)

---

## Slide 12: Conclusion — Architecture-Driven Design Yields 75% Loss Reduction
**Heading:** Mapping vehicle speed to inverter parameters enables intelligent optimization impossible with fixed-frequency designs

**Key Points:**
- 800V architecture justified by charging speed, cable weight, and efficiency
- FC topology selected for thermal symmetry and natural balancing under dynamic EV loads
- Adaptive fsw schedule: 75% switching loss reduction in urban driving (190W saved, 3-phase)
- Thermal benefit: 9.7°C Tj reduction, 53% less thermal cycling → longer module lifetime
- All validated: THD < 2%, FC balance < 0.04V error, Tj < 54°C with >120°C margin
- Future work: experimental validation on hardware prototype

---
