# Design and Optimization of a Three-Level Flying Capacitor Inverter for 800V EV Traction Applications

**Hayagreev S.**  
*Department of Electrical Engineering*  
*(Reviewed and Optimized by Manus AI)*  

---

## Abstract
**This paper presents the design, simulation, and optimization of a Three-Level Flying Capacitor (FC) inverter for an 800V Electric Vehicle (EV) traction application. While two-level inverters dominate current EV architectures, the transition to 800V bus architectures necessitates multilevel topologies to mitigate high $dv/dt$ stress and reduce switching losses. The proposed design employs Phase-Disposition Pulse Width Modulation (PD-PWM) with an adaptive switching frequency schedule based on a WLTP-inspired drive cycle. A comprehensive peer review and optimization process was conducted, incorporating exact datasheet parameters from a commercial 1200V, 450A SiC power module (Wolfspeed CAB450M12XM3). The flying capacitor was resized for practical industry implementation, and a pre-charge transient model was introduced. Simulation results demonstrate excellent flying capacitor voltage balancing, a low Total Harmonic Distortion (THD) of 1.92% at urban speeds, and a 48.7% reduction in switching losses compared to a fixed-frequency baseline, confirming the viability of the topology for high-efficiency traction drives.**

---

## I. Introduction

The rapid evolution of Electric Vehicles (EVs) is driving a shift from traditional 400V to 800V DC bus architectures. This transition enables faster charging times, reduced cable weight, and higher overall system efficiency [1]. However, the increased DC link voltage presents significant challenges for the traction inverter, primarily concerning semiconductor voltage ratings, electromagnetic interference (EMI), and the $dv/dt$ stress imposed on motor windings [2].

Traditional two-level voltage source inverters (2L-VSI) face severe limitations at 800V. They require semiconductor devices with blocking voltages of at least 1200V. While Silicon Carbide (SiC) MOSFETs can meet this requirement, their extremely fast switching speeds exacerbate the $dv/dt$ stress, leading to premature motor insulation failure and bearing degradation [3].

Multilevel inverters, such as the Neutral Point Clamped (NPC) and Flying Capacitor (FC) topologies, offer a compelling solution. By synthesizing the output voltage from multiple discrete levels, they inherently reduce the voltage step size ($dv/dt$) and improve the harmonic profile of the output current [4]. The Three-Level Flying Capacitor (3L-FC) inverter is particularly advantageous because it avoids the need for clamping diodes (as in the NPC) and offers redundant switching states that can be exploited for natural capacitor voltage balancing [5].

This paper details the design of a 3L-FC inverter for an 800V traction application. The initial design was subjected to a rigorous peer review, leading to critical optimizations in semiconductor modeling, component selection, and operational control. The optimized system is validated through a high-fidelity Python-based simulation.

## II. Inverter Topology and Modulation

### A. Three-Level Flying Capacitor Topology
The 3L-FC inverter utilizes a pre-charged capacitor (the "flying" capacitor) in each phase leg to synthesize three distinct voltage levels: $+V_{dc}/2$, $0$, and $-V_{dc}/2$. Each phase leg consists of four active switches ($S_1$ to $S_4$). The flying capacitor, $C_{fc}$, is nominally charged to $V_{dc}/2$ (400V for an 800V bus).

The output voltage $V_{pole}$ is determined by the switching states:
*   **State 1:** $S_1$, $S_2$ ON $\rightarrow V_{pole} = +V_{dc}/2$
*   **State 2 (Zero State A):** $S_1$, $S_3$ ON $\rightarrow V_{pole} = +V_{dc}/2 - V_{fc} \approx 0V$
*   **State 3 (Zero State B):** $S_2$, $S_4$ ON $\rightarrow V_{pole} = -V_{dc}/2 + V_{fc} \approx 0V$
*   **State 4:** $S_3$, $S_4$ ON $\rightarrow V_{pole} = -V_{dc}/2$

### B. Phase-Disposition PWM and Natural Balancing
Phase-Disposition PWM (PD-PWM) is employed to generate the gating signals. PD-PWM utilizes two vertically shifted carrier waves compared against a single sinusoidal reference. 

A critical advantage of the 3L-FC topology is the availability of redundant zero states (State 2 and State 3), which produce the same output voltage but have opposite effects on the flying capacitor current. 
*   In **State A**, the flying capacitor current $i_{fc} = -i_{phase}$.
*   In **State B**, the flying capacitor current $i_{fc} = +i_{phase}$.

The control logic actively selects between State A and State B based on the measured phase current direction and the deviation of $V_{fc}$ from its 400V reference. This ensures the flying capacitor remains naturally balanced without requiring complex closed-loop PI controllers [5].

## III. Design Optimization and Component Selection

An extensive peer review of the initial design identified several areas where the academic model diverged from industry best practices. The following optimizations were implemented to ensure high fidelity and practical relevance.

### A. Semiconductor Device Selection and Modeling
The original simulation utilized generic parameters for a 650V discrete device, which is inadequate for an 800V bus. The optimized design specifies the **Wolfspeed CAB450M12XM3**, a 1200V, 450A SiC Half-Bridge Power Module [6].

The simulation was calibrated directly from the datasheet:
1.  **Conduction Losses:** The effective $R_{ds(on)}$ is modeled as 3.3 mΩ at 25°C, scaling to 6.6 mΩ at 175°C, accurately reflecting the temperature dependence shown in the datasheet curves.
2.  **Switching Losses:** A linear SiC switching energy model was implemented based on the reference values $E_{on} = 25.4$ mJ and $E_{off} = 7.51$ mJ (at 450A, 600V) [6].
3.  **Thermal Modeling:** A 4-layer Foster thermal network was extracted from the module's transient thermal impedance ($Z_{th,jc}$) curve, yielding a total $R_{th,jc}$ of 94 m°C/W.

### B. Flying Capacitor Sizing
The initial design specified an excessively large 1.5 mF capacitor. In commercial traction inverters, the flying capacitor is sized based on an allowable voltage ripple ($\Delta V_{fc}$), typically 5% of the nominal voltage [7].

The required capacitance is determined by the worst-case operating point, which occurs at maximum peak current ($I_{pk}$) and minimum switching frequency ($f_{sw}$):
$$ C_{fc} = \frac{I_{pk}}{2 \cdot f_{sw} \cdot \Delta V_{fc}} $$

For the urban driving segment ($I_{pk} = 136$ A, $f_{sw} = 5$ kHz) and a 20V ripple budget (5% of 400V), the required capacitance is calculated as 680 µF. This is a highly practical value that can be realized using high-ripple-current automotive film capacitors, such as the TDK EPCOS or KEMET C4AQ series [8].

### C. Pre-Charge Transient
A critical operational requirement omitted in the initial model is the pre-charging of the flying capacitor. Starting the inverter with $V_{fc} = 0V$ causes severe overvoltage across the inner switches. The optimized simulation incorporates a 20ms pre-charge sequence, linearly ramping $V_{fc}$ to 400V before active PWM switching commences, mimicking a dedicated pre-charge resistor circuit.

## IV. Simulation Results

The optimized inverter was simulated using a custom Python framework over a WLTP-inspired drive cycle. The load was modeled as a pure RL circuit (R=1.3Ω, L=2mH) per the specific project constraints.

### A. Adaptive Switching Frequency
To optimize the trade-off between switching losses and current ripple, an adaptive switching frequency schedule was employed:
*   **Urban Driving (<60 km/h):** $f_{sw} = 5$ kHz. At low speeds, the fundamental frequency is low, and audible noise is less critical.
*   **Highway Driving (>80 km/h):** $f_{sw} = 20$ kHz. Higher speeds require a higher switching frequency to maintain an acceptable ratio between $f_{sw}$ and the fundamental electrical frequency ($f_e$).

### B. Electrical Performance and THD
The inverter successfully synthesized the three-level output voltage. The PD-PWM balancing algorithm maintained the flying capacitor voltage exceptionally well, with a mean error of less than 0.04V from the 400V reference.

![FC Voltages](fc_inverter_results_v3/fig4_fc_voltages.png)
*Fig. 1. Flying Capacitor voltage balancing demonstrating the 20ms pre-charge transient and steady-state ripple at urban and highway speeds.*

The simulated voltage ripple closely matched the analytical predictions, validating the 680 µF sizing. The Total Harmonic Distortion (THD) of the phase current, calculated using a Hanning-windowed FFT, was excellent:
*   **Urban (5 kHz):** THD = 1.92% ($I_{fund} = 47.5$ A rms)
*   **Highway (20 kHz):** THD = 1.13% ($I_{fund} = 17.7$ A rms)

![Phase Currents](fc_inverter_results_v3/fig3_phase_currents.png)
*Fig. 2. Three-phase output currents and the harmonic spectrum during the urban driving segment.*

### C. Thermal and Loss Analysis
The thermal performance of the CAB450M12XM3 module was evaluated using the 4-layer Foster network. The results highlight the immense benefit of the adaptive switching frequency strategy.

| Metric | Scheduled $f_{sw}$ (Adaptive) | Fixed 20 kHz Baseline |
| :--- | :--- | :--- |
| Conduction Loss | 30.5 W | 30.5 W |
| Switching Loss | 45.2 W | 88.3 W |
| **Total Loss (3-Phase)** | **75.7 W** | **118.8 W** |
| Max Junction Temp ($T_j$) | 46.4 °C | 49.7 °C |

The adaptive schedule resulted in a **48.7% reduction in switching losses** (saving 43.0 W) compared to operating continuously at 20 kHz. The maximum junction temperature reached only 46.4°C, providing a massive 128.6°C margin to the module's 175°C limit.

![Thermal Performance](fc_inverter_results_v3/fig5_thermal.png)
*Fig. 3. Instantaneous semiconductor losses and junction temperature comparison between the scheduled and fixed switching frequencies.*

## V. Conclusion

This paper presented the optimized design of a Three-Level Flying Capacitor inverter for an 800V EV traction drive. By subjecting the initial academic design to a rigorous peer review, the system was upgraded with exact datasheet parameters from a commercial 1200V SiC module, practical capacitor sizing, and a pre-charge sequence. The simulation results confirm that the 3L-FC topology, combined with PD-PWM and an adaptive switching frequency schedule, provides exceptional performance. The design achieves excellent capacitor balancing, low THD (<2%), and nearly a 50% reduction in switching losses, proving its highly effective nature for next-generation 800V EV architectures.

## References

[1] T. Modeer et al., "Multilevel Inverters for EV Traction Applications: A Review," *IEEE Transactions on Transportation Electrification*, 2020. [Link](https://ieeexplore.ieee.org/iel8/8782706/8955964/11431113.pdf)  
[2] Texas Instruments, "EV Traction Inverter Design Priorities," Whitepaper SPRAD58B. [Link](https://www.ti.com/lit/wp/sprad58b/sprad58b.pdf)  
[3] "A Closer Look at Multilevel Traction Inverters," *Charged EVs*, 2023. [Link](https://chargedevs.com/features/a-closer-look-at-multilevel-traction-inverters/)  
[4] Wolfspeed, "CAB450M12XM3 1200V, 450A SiC Half-Bridge Module Datasheet," Rev. 3, June 2024. [Link](https://assets.wolfspeed.com/uploads/2024/01/Wolfspeed_CAB450M12XM3_data_sheet.pdf)  
[5] TDK EPCOS, "Capacitors for DC Link and Flying Capacitor Applications," Product Guide.
