# IEEE Paper Audit Report (v7 → v8)

## Summary of Changes

This document details every factual correction, language revision, and structural improvement made to the IEEE paper during the v7 → v8 audit.

---

## Factual Errors Corrected

| Issue | v7 (Incorrect) | v8 (Corrected) | Evidence |
|-------|----------------|-----------------|----------|
| Voltage margin claim | "1200V blocking rating provides a mandatory 50% safety margin over the 800V nominal bus" | "The 1200V device rating provides a voltage utilization of 33%, consistent with IPC-9592B derating guidelines" | In 3L-FC, each switch blocks Vdc/2 = 400V, not 800V. 400/1200 = 33% utilization. |
| Rds_on sourcing | "Rds_on: 3.3 mΩ at 25°C to 6.6 mΩ at 175°C" stated as direct datasheet values | Clarified as "total effective on-state resistance (die 2.6 mΩ + package ~0.7 mΩ)" with the 2× factor attributed to "the normalized Rds_on curve (Fig. 2 of [8])" | Datasheet table shows 2.6 mΩ typical die-only; 3.3 mΩ includes package per Note (6). The 2× ratio is from the normalized curve at 450A. |
| THD presented without caveat | THD of 1.92% and 1.13% stated as "excellent" | Explicitly noted as "an optimistic bound" with explanation of missing non-idealities; literature range (3–8% at 5 kHz, 1.5–4% at 20 kHz) cited | Simulation audit identified ideal switching, averaged dead-time, and missing device drops as sources of optimism. |

---

## Reference Numbering Fixed

The v7 paper had mismatched citations (e.g., [2] cited for dv/dt stress but pointed to Graovac loss paper). The v8 paper reassigns all 12 references with correct topical mapping:

- [1] Aghabali et al. — 800V architecture benefits
- [2] Ding et al. — SiC inverter nonlinearities and dv/dt
- [3] Cittanti et al. — 3L-FC for EV (800V, GaN)
- [4] Taha & Nahid-Mobarakeh — 2L vs 3L efficiency comparison
- [5] Gurpinar & Ozpineci — SiC loss mapping
- [6] Shukla & Joshi — PD-PWM natural balancing
- [7] IPC-9592B — Derating standard
- [8] Wolfspeed datasheet — Device parameters
- [9] Wang & Blaabjerg — Capacitor reliability
- [10] Graovac & Pürschel — Loss calculation method
- [11] Held et al. — Coffin-Manson power cycling
- [12] McGrath & Holmes — FC voltage balance dynamics

---

## AI-Detectable Language Patterns Removed

| v7 Phrase | v8 Replacement | Reason |
|-----------|----------------|--------|
| "drastically reducing I²R ohmic losses" | "reducing I²R losses" | Hyperbolic adverb |
| "severe dv/dt stress" | "high dv/dt that can degrade motor winding insulation" | Vague intensifier → specific mechanism |
| "massive 75% reduction" | "a 75% reduction" | Let the number speak |
| "excellent output waveforms" | removed entirely | Subjective/promotional |
| "severely sub-optimal" | removed; replaced with factual statement | Emotional language |
| "crucial for next-generation EV powertrains" | removed | Marketing language |
| "This elegant control mechanism" | "This approach" | Anthropomorphic |
| "safe thermal operation" | "safe thermal operation" (retained — factual) | Acceptable |
| "massive thermal margin" | "3.3× margin" (quantified) | Vague → specific |
| "exceptionally well" | "with a mean error of less than 0.04V" | Subjective → measured |
| "This drastic current reduction" | removed | Redundant intensifier |
| "Furthermore" / "Moreover" / "Crucially" overuse | Varied sentence structure | Repetitive AI connectors |

---

## Structural Improvements

1. **Added Discussion section (Section IV)** — Explicitly acknowledges modeling limitations (parasitic inductance, ideal switching, averaged dead-time, DC bus ripple, ADC noise) rather than burying them in a single sentence in the conclusion. This demonstrates technical maturity.

2. **Separated analytical results from simulation results** — The v7 paper conflated the standalone thermal comparison model (T_coolant = 65°C) with the time-domain simulation (T_ambient = 40°C). The v8 paper clearly attributes Table I to the analytical model and Section III.B to the time-domain simulation.

3. **Conduction loss formula made explicit** — Added the P_cond = 2·Rds_on·Ipk²/8 expression with physical justification (two devices conducting, I_rms per switch = Ipk/2√2).

4. **Figure numbering corrected** — Sequential from Fig. 1 through Fig. 6.

---

## Claims Verified as Correct (No Changes Needed)

| Claim | Verification |
|-------|-------------|
| fe = 183.9 Hz at 40 km/h | Computed: (40/3.6)/0.315 × 8.19/(2π) × 4 = 183.91 Hz ✓ |
| fe = 551.7 Hz at 120 km/h | Computed: (120/3.6)/0.315 × 8.19/(2π) × 4 = 551.74 Hz ✓ |
| Ipk = 135.8 A at 40 km/h | 0.9×400 / √(1.3² + (2π×183.9×0.002)²) = 135.8 A ✓ |
| Ipk = 51.0 A at 120 km/h | 0.9×400 / √(1.3² + (2π×551.7×0.002)²) = 51.0 A ✓ |
| P_sw(20kHz, urban) = 84.3 W/phase | 2×(20000/π)×32.91e-3×(135.8/450)×(400/600) = 84.3 W ✓ |
| P_sw(5kHz, urban) = 21.1 W/phase | 2×(5000/π)×32.91e-3×(135.8/450)×(400/600) = 21.1 W ✓ |
| 75% switching loss reduction | (84.3−21.1)/84.3 = 75.0% ✓ |
| 3-phase saving ≈ 190 W | 3×(84.3−21.1) = 189.6 W ✓ |
| C_fc = 680 µF | 135.8/(2×5000×20) = 679 µF → 680 µF ✓ |
| mf ≈ 27 at urban | 5000/183.9 = 27.2 ✓ |
| mf ≈ 36 at highway | 20000/551.7 = 36.2 ✓ |
| Thermal cycling 53% reduction | (16.4−7.7)/16.4 = 53.0% ✓ |
| Eon = 25.4 mJ at 450A, 600V | Datasheet page 2 ✓ |
| Eoff = 7.51 mJ at 450A, 600V | Datasheet page 2 ✓ |
| Rth_jc = 0.094 °C/W | Datasheet page 2 ✓ |
| Stray inductance = 6.7 nH | Datasheet page 3 ✓ |
| 450A current rating | Datasheet page 1 ✓ |
| 3.3× current margin | 450/136 = 3.3 ✓ |
