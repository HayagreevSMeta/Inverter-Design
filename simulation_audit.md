# Simulation Audit: Why Results Are Too Clean

## Issues Identified

### 1. MISSING: DC Bus Voltage Ripple
The simulation uses a perfectly constant Vdc = 800V. In reality, the DC-link capacitor bank has finite capacitance and ESR, causing voltage ripple of 5-20V depending on load current. This ripple directly modulates the output voltage and increases THD.

**Fix:** Add a DC-link capacitor model (e.g., 500µF with 10mΩ ESR) and compute Vdc dynamically.

### 2. MISSING: Carrier Interleaving Imperfections
The PD-PWM implementation uses a perfectly synchronized carrier. In practice, digital PWM has quantization effects (timer resolution), and the carrier-to-reference comparison has 1-sample delay.

**Fix:** Add PWM quantization (round carrier comparison to nearest dt) and 1-step transport delay.

### 3. MISSING: Device Voltage Drops in Output Voltage
The output voltage computation (lines 401-414) uses ideal Vdc/2 levels. In reality, each conducting device drops Rds_on * I, and the body diode drops Vf during dead time. This creates a nonlinear voltage error that worsens THD, especially at low currents.

**Fix:** Include Rds_on*I drop and Vf body diode drop in V_pole calculation.

### 4. DEAD-TIME MODEL IS OVERSIMPLIFIED
Line 419: `V_dt_error = sign(i) * Vdc * t_dead * fsw` — this is a continuous average model. Real dead-time creates discrete notches in the voltage waveform at every switching transition, producing low-order harmonics (5th, 7th) that significantly increase THD. The average model smooths these out.

**Fix:** Implement actual dead-time: during t_dead, both switches off, current freewheels through body diode. This creates a voltage error of ±Vf (not a smooth average).

### 5. FC VOLTAGE USED IN OUTPUT IS IDEAL
The FC voltage in the output calculation doesn't include ESR drop: V_fc_terminal = V_fc_cap - ESR * i_fc. This means the zero-state output levels are slightly wrong.

**Fix:** Use V_fc_terminal = Vfc[ph] - ESR_fc * i_fc_actual[ph] in V_pole.

### 6. MISSING: Parasitic Inductance (Commutation Loop)
The CAB450M12XM3 has 6.5nH stray inductance. During switching transitions, this creates voltage spikes (L * di/dt) that are not modeled. While these don't directly affect THD of the filtered output, they affect the effective switching voltage and thus switching losses.

### 7. THD COMPUTATION WINDOW IS TOO CLEAN
The THD is computed on a Hanning-windowed segment from the middle of a steady-state region. This is correct methodology, but the window is very long (20 cycles), which averages out transient effects. A shorter window (3-5 cycles) would show more realistic variation.

### 8. MISSING: Sampling/Computation Delay
Real digital controllers have a 1-sample computation delay between ADC sampling and PWM update. This phase lag increases with frequency and worsens THD at highway speeds.

**Fix:** Apply reference voltage from previous timestep: v_ref[k] uses fe from t[k-1].

### 9. FC BALANCING IS UPDATED EVERY CARRIER PERIOD (TOO FREQUENT)
Line 384: `if period_start` — the balancing decision is made every carrier period. In practice, the FC voltage measurement has noise and the controller may only update every N carrier periods. The current implementation achieves near-perfect balancing.

**Fix:** Add measurement noise (±0.5V Gaussian) and update balancing only every 4th carrier period.

### 10. MISSING: Initial Current Transient Effects
The simulation starts currents at exactly 0A and the RL load settles very quickly (L/R = 1.5ms). But the transition between drive cycle segments is instantaneous (no ramp), creating unrealistically clean steady-state windows.

## Expected Realistic THD Values
- 3-level FC inverter with 5 kHz switching: **3-8% THD** (IEEE literature)
- 3-level FC inverter with 20 kHz switching: **1.5-4% THD** (IEEE literature)
- Current simulation reports: 1.92% and 1.13% — these are at the very optimistic end

## Priority Fixes (Most Impact on Realism)
1. Device voltage drops (Rds_on*I and Vf) in output voltage
2. Discrete dead-time model (not averaged)
3. DC bus voltage ripple
4. FC measurement noise and reduced balancing update rate
5. PWM quantization delay
