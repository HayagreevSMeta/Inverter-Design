%% FC_Inverter_Run.m
%  Three-Phase Three-Level Flying Capacitor Inverter Simulation
%  800V EV Traction with Adaptive Switching Frequency Scheduling
%  ---------------------------------------------------------------
%  Version 2.0 - Updated May 2026
%  Author: Hayagreev S.
%
%  Topology: 3-level FC inverter, 800V DC bus
%   - 4 switches per phase (S1-S4), one flying capacitor per phase
%   - FC reference voltage = Vdc/2 = 400V
%   - Output levels: +Vdc/2, 0, -Vdc/2
%   - PD-PWM with natural FC voltage balancing
%
%  Improvements over v1:
%   - Corrected FC reference voltage (400V = Vdc/2 for 3-level topology)
%   - Proper drivetrain model (gear ratio + tyre radius -> fe)
%   - Nonlinear switching energy model (quadratic I scaling)
%   - 4-layer Foster thermal network
%   - Temperature-dependent Rds_on (SiC module)
%   - Dead-time voltage distortion
%   - Hanning-windowed THD computation
%   - Corrected loss accounting (conduction identical for both strategies)
%
%  Companion Python version: fc_inverter_sim.py (produces identical results)
%% ========================================================================

clear; clc; close all;
fprintf('==================================================================\n');
fprintf('  FC Inverter Simulation v2 (MATLAB)\n');
fprintf('  Three-Phase Three-Level Flying Capacitor | 800V EV Traction\n');
fprintf('==================================================================\n\n');

OUT_DIR = fullfile(fileparts(mfilename('fullpath')), 'fc_inverter_results');
if ~exist(OUT_DIR,'dir'), mkdir(OUT_DIR); end

%% 1. SYSTEM PARAMETERS
% ========================================================================

% DC Bus
Vdc_total = 800;           % Total DC bus voltage [V]
Vdc = Vdc_total / 2;       % Half-bus = device blocking voltage [V]
Vc_ref = Vdc_total / 2;    % FC reference voltage = Vdc/2 = 400V

% Load (per project spec)
R_load = 1.3;              % Load resistance [Ohm]
L_load = 2e-3;             % Load inductance [H]

% Flying Capacitor
C_fc = 1.5e-3;             % Flying capacitor [F]
ESR_fc = 3.3e-3;           % Equivalent series resistance [Ohm]

% SiC MOSFET Power Module (Wolfspeed CAB450M12XM3 class, 1200V/450A)
Rds_on_25C = 4.0e-3;       % On-state resistance [Ohm] at 25C
Vf_body = 1.5;             % Body diode forward voltage [V]

% Switching energy (module datasheet at Vds=400V, Id=300A)
E_on_ref = 8.5e-3;         % Turn-on energy [J] at I_ref, V_ref
E_off_ref = 4.2e-3;        % Turn-off energy [J] at I_ref, V_ref
I_ref = 300;               % Reference current [A]
V_ref = 400;               % Reference voltage [V]

% Nonlinear switching energy coefficients
% E = E_ref * (a*(I/Iref)^2 + b*(I/Iref) + c) * (V/Vref)
sw_a = 0.10; sw_b = 0.80; sw_c = 0.10;

% Dead time
t_dead = 200e-9;           % Dead time [s]

% Thermal model: 4-layer Foster (power module)
Rth_layers = [0.012, 0.010, 0.009, 0.009];  % [K/W]
tau_layers = [0.5e-3, 5e-3, 50e-3, 500e-3]; % [s]
Rth_cs = 0.015;            % Case-to-sink [K/W]
Rth_sa = 0.025;            % Sink-to-ambient [K/W]
T_ambient = 40;            % Ambient temperature [C]

% Modulation
m_index = 0.9;             % Modulation index

% Drivetrain
r_tyre = 0.315;            % Tyre radius [m]
gear_ratio = 8.19;         % Gear ratio
P_poles = 8;               % Motor pole count

% Simulation
dt = 2e-6;                 % Time step [s]

Z_load_fn = @(fe) sqrt(R_load^2 + (2*pi*fe*L_load)^2);
phi_abc = [0, -2*pi/3, 2*pi/3];

fprintf('  Vdc=%gV  R=%g Ohm  L=%gmH  C_fc=%gmF  dt=%gus\n', ...
    Vdc_total, R_load, L_load*1e3, C_fc*1e3, dt*1e6);
fprintf('  FC reference voltage: %gV\n\n', Vc_ref);

%% 2. DRIVE CYCLE
% ========================================================================

% Speed to electrical frequency conversion
speed_to_fe = @(v_kmh) max((P_poles/2) * ((v_kmh/3.6)/r_tyre) * (gear_ratio/(2*pi)), 1.0);

% Switching frequency schedule
fsw_schedule = @(v) (v < 60)*5e3 + (v >= 60 & v < 80)*10e3 + (v >= 80)*20e3;

% Segment definitions
seg_names = {'Standstill','Urban low','Urban mid','Urban high',...
             'Deceleration','Stop','Transition','Highway',...
             'Highway fast','Highway cruise'};
seg_speeds = [0, 20, 40, 60, 20, 0, 80, 120, 140, 100];
seg_durs   = [0.05, 0.10, 0.20, 0.20, 0.10, 0.05, 0.20, 0.20, 0.20, 0.20];

% Build time-domain arrays
N_total = round(sum(seg_durs) / dt);
t_arr = zeros(N_total, 1);
v_speed = zeros(N_total, 1);
fe_arr = zeros(N_total, 1);
fsw_arr = zeros(N_total, 1);

idx = 1;
tc = 0;
fprintf('  %-18s %6s %8s %8s %7s %8s\n', 'Segment', 'v', 'fe(Hz)', 'fsw(kHz)', 'dur(s)', 'Ipk(A)');
fprintf('  %s\n', repmat('-', 1, 60));

for s = 1:length(seg_speeds)
    spd = seg_speeds(s);
    dur = seg_durs(s);
    fe = speed_to_fe(spd);
    fsw = fsw_schedule(spd);
    Ipk = 0;
    if spd > 0
        Ipk = m_index * Vdc / Z_load_fn(fe);
    end
    
    n_steps = round(dur / dt);
    t_arr(idx:idx+n_steps-1) = tc + (0:n_steps-1)' * dt;
    v_speed(idx:idx+n_steps-1) = spd;
    fe_arr(idx:idx+n_steps-1) = fe;
    fsw_arr(idx:idx+n_steps-1) = fsw;
    
    fprintf('  %-18s %6.0f %8.1f %8.0f %7.2f %8.1f\n', ...
        seg_names{s}, spd, fe, fsw/1e3, dur, Ipk);
    
    idx = idx + n_steps;
    tc = tc + dur;
end
N = idx - 1;
t_arr = t_arr(1:N); v_speed = v_speed(1:N);
fe_arr = fe_arr(1:N); fsw_arr = fsw_arr(1:N);
T_sim = t_arr(end) + dt;
fprintf('\n  Total: %d steps, %.3fs\n\n', N, T_sim);

%% 3. MAIN SIMULATION LOOP
% ========================================================================

% State variables
i_abc = zeros(1, 3);
Vfc = Vc_ref * ones(1, 3);  % Pre-charged to reference

% Output storage
o_ia = zeros(N, 1); o_ib = zeros(N, 1); o_ic = zeros(N, 1);
o_Van = zeros(N, 1); o_Vbn = zeros(N, 1); o_Vcn = zeros(N, 1);
o_Vfc = zeros(N, 3);

% Loss accumulators
E_cond_sched = zeros(1, 3);
E_sw_sched = zeros(1, 3);
E_cond_fixed = zeros(1, 3);
E_sw_fixed = zeros(1, 3);

% Instantaneous loss (phase A)
P_inst_sched = zeros(N, 1);
P_inst_fixed = zeros(N, 1);

% Thermal state (4-layer Foster)
dT_sched = zeros(4, 1);
dT_fixed = zeros(4, 1);
Tj_sched = zeros(N, 1);
Tj_fixed = zeros(N, 1);

% Carrier state
carrier_phase = 0;
fc_state = zeros(1, 3);

fprintf('  Running simulation...');
tic;

for k = 1:N
    t_k = t_arr(k);
    fe_k = fe_arr(k);
    fsw_k = fsw_arr(k);
    
    % --- Carrier ---
    cp_new = carrier_phase + fsw_k * dt;
    period_start = (cp_new >= 1.0) || (k == 1);
    if cp_new >= 1.0
        cp_new = cp_new - 1.0;
    end
    carrier_phase = cp_new;
    
    % Triangular carrier [0, 1]
    if carrier_phase < 0.5
        tri = 2.0 * carrier_phase;
    else
        tri = 2.0 * (1.0 - carrier_phase);
    end
    
    % PD-PWM carriers
    C_up = tri;
    C_lo = -tri;
    
    % --- Reference ---
    we = 2 * pi * fe_k;
    v_ref = m_index * sin(we * t_k + phi_abc);
    
    % --- Determine output levels ---
    sw_level = zeros(1, 3);
    for ph = 1:3
        if v_ref(ph) > C_up
            sw_level(ph) = 1;
        elseif v_ref(ph) < C_lo
            sw_level(ph) = -1;
        else
            sw_level(ph) = 0;
        end
    end
    
    % --- FC balancing at period start ---
    if period_start
        for ph = 1:3
            if Vfc(ph) < Vc_ref
                if i_abc(ph) >= 0
                    fc_state(ph) = 1;   % State B: charges when i>0
                else
                    fc_state(ph) = -1;  % State A: charges when i<0
                end
            else
                if i_abc(ph) >= 0
                    fc_state(ph) = -1;  % State A: discharges when i>0
                else
                    fc_state(ph) = 1;   % State B: discharges when i<0
                end
            end
        end
    end
    
    % --- Compute output pole voltages ---
    V_pole = zeros(1, 3);
    i_fc_actual = zeros(1, 3);
    
    for ph = 1:3
        if sw_level(ph) == 1
            % S1, S2 ON: output = +Vdc/2, FC bypassed
            V_pole(ph) = Vdc / 2;
            i_fc_actual(ph) = 0;
        elseif sw_level(ph) == -1
            % S3, S4 ON: output = -Vdc/2, FC bypassed
            V_pole(ph) = -Vdc / 2;
            i_fc_actual(ph) = 0;
        else
            % Zero state: depends on fc_state selection
            if fc_state(ph) == 1
                % State B: S2, S3 ON -> Vout = -Vdc/2 + Vfc
                V_pole(ph) = -Vdc/2 + Vfc(ph);
                i_fc_actual(ph) = i_abc(ph);  % Charges when i>0
            else
                % State A: S1, S4 ON -> Vout = +Vdc/2 - Vfc
                V_pole(ph) = Vdc/2 - Vfc(ph);
                i_fc_actual(ph) = -i_abc(ph); % Discharges when i>0
            end
        end
    end
    
    % Dead-time voltage error
    for ph = 1:3
        if abs(i_abc(ph)) > 0.1
            V_pole(ph) = V_pole(ph) - sign(i_abc(ph)) * (Vdc/2) * t_dead * fsw_k * 2;
        end
    end
    
    % Neutral point (floating star)
    Vn = sum(V_pole) / 3;
    V_phase = V_pole - Vn;
    
    % --- Current update (Forward Euler) ---
    di = (V_phase - R_load * i_abc) / L_load * dt;
    i_abc_new = i_abc + di;
    i_abc_new = i_abc_new - mean(i_abc_new);  % Zero-sequence elimination
    
    % --- FC voltage update ---
    for ph = 1:3
        Vfc(ph) = Vfc(ph) + i_fc_actual(ph) / C_fc * dt;
    end
    
    % --- Loss calculation ---
    Tj_est = T_ambient;
    if k > 1
        Tj_est = Tj_sched(k-1);
    end
    t_norm = min(max((Tj_est - 25) / (150 - 25), 0), 1);
    Rds_on_T = Rds_on_25C * (1 + 0.8 * t_norm);
    
    for ph = 1:3
        I_ph = abs(i_abc(ph));
        
        % Conduction: 2 devices in series
        P_cond = 2 * Rds_on_T * I_ph^2;
        E_step = P_cond * dt;
        E_cond_sched(ph) = E_cond_sched(ph) + E_step;
        E_cond_fixed(ph) = E_cond_fixed(ph) + E_step;
        
        % Switching energy (nonlinear model)
        I_norm = I_ph / I_ref;
        scale = sw_a * I_norm^2 + sw_b * I_norm + sw_c;
        E_sw_event = (E_on_ref + E_off_ref) * scale;
        E_sw_sched(ph) = E_sw_sched(ph) + E_sw_event * fsw_k * dt;
        E_sw_fixed(ph) = E_sw_fixed(ph) + E_sw_event * 20e3 * dt;
        
        if ph == 1
            P_inst_sched(k) = P_cond + E_sw_event * fsw_k;
            P_inst_fixed(k) = P_cond + E_sw_event * 20e3;
        end
    end
    
    % Thermal update (worst-case device = half phase loss)
    P_dev_sched = P_inst_sched(k) / 2;
    P_dev_fixed = P_inst_fixed(k) / 2;
    
    for i = 1:4
        alpha = dt / tau_layers(i);
        if alpha < 0.1
            dT_sched(i) = dT_sched(i) + (P_dev_sched * Rth_layers(i) - dT_sched(i)) * alpha;
            dT_fixed(i) = dT_fixed(i) + (P_dev_fixed * Rth_layers(i) - dT_fixed(i)) * alpha;
        else
            exp_a = exp(-alpha);
            dT_sched(i) = dT_sched(i) * exp_a + P_dev_sched * Rth_layers(i) * (1 - exp_a);
            dT_fixed(i) = dT_fixed(i) * exp_a + P_dev_fixed * Rth_layers(i) * (1 - exp_a);
        end
    end
    Tj_sched(k) = T_ambient + sum(dT_sched) + P_dev_sched * (Rth_cs + Rth_sa);
    Tj_fixed(k) = T_ambient + sum(dT_fixed) + P_dev_fixed * (Rth_cs + Rth_sa);
    
    % --- Store ---
    o_ia(k) = i_abc(1); o_ib(k) = i_abc(2); o_ic(k) = i_abc(3);
    o_Van(k) = V_phase(1); o_Vbn(k) = V_phase(2); o_Vcn(k) = V_phase(3);
    o_Vfc(k, :) = Vfc;
    
    % Update state
    i_abc = i_abc_new;
end

elapsed = toc;
fprintf(' Done in %.1fs (%d steps)\n\n', elapsed, N);

%% 4. POST-PROCESSING
% ========================================================================

% Loss summary
P_cond_sched = E_cond_sched / T_sim;
P_sw_sched_ph = E_sw_sched / T_sim;
P_total_sched = P_cond_sched + P_sw_sched_ph;

P_cond_fixed = E_cond_fixed / T_sim;
P_sw_fixed_ph = E_sw_fixed / T_sim;
P_total_fixed = P_cond_fixed + P_sw_fixed_ph;

% THD Analysis
fe_urban = speed_to_fe(40);
fe_highway = speed_to_fe(120);

idx_urban = find(v_speed >= 38 & v_speed <= 42);
idx_highway = find(v_speed >= 118 & v_speed <= 122);

fprintf('  --- THD Analysis (Hanning-windowed FFT) ---\n');

THD_urban = NaN; I_fund_urban = NaN;
THD_highway = NaN; I_fund_highway = NaN;

if length(idx_urban) > 500
    [THD_urban, I_fund_urban] = compute_thd_fn(o_ia(idx_urban), fe_urban, dt);
    fprintf('    Urban 40km/h (5kHz):     fe=%7.1fHz  I_fund=%7.2fA rms  THD=%6.2f%%\n', ...
        fe_urban, I_fund_urban, THD_urban);
end

if length(idx_highway) > 500
    [THD_highway, I_fund_highway] = compute_thd_fn(o_ia(idx_highway), fe_highway, dt);
    fprintf('    Highway 120km/h (20kHz): fe=%7.1fHz  I_fund=%7.2fA rms  THD=%6.2f%%\n', ...
        fe_highway, I_fund_highway, THD_highway);
end

% FC ripple
rip_urban = max(o_Vfc(idx_urban, 1)) - min(o_Vfc(idx_urban, 1));
rip_highway = max(o_Vfc(idx_highway, 1)) - min(o_Vfc(idx_highway, 1));

Ipk_urban = m_index * Vdc / Z_load_fn(fe_urban);
Ipk_highway = m_index * Vdc / Z_load_fn(fe_highway);
rip_pred_5k = Ipk_urban * 0.5 / (2 * 5e3 * C_fc);
rip_pred_20k = Ipk_highway * 0.5 / (2 * 20e3 * C_fc);

% Print summary
fprintf('\n');
fprintf('  ============================================================\n');
fprintf('  RESULTS SUMMARY\n');
fprintf('  ============================================================\n');
fprintf('  Peak phase current              : %.2f A\n', max(abs(o_ia)));
fprintf('  Peak Van                        : %.1f V\n', max(abs(o_Van)));
fprintf('  Ipeak expected @ urban 40km/h   : %.2f A\n', Ipk_urban);
fprintf('  Ipeak expected @ highway 120km/h: %.2f A\n', Ipk_highway);
fprintf('  FC mean voltage (phase A)       : %.2f V (ref=%.1fV)\n', mean(o_Vfc(:,1)), Vc_ref);
fprintf('  FC pk-pk ripple (full cycle)    : %.2f V\n', max(o_Vfc(:,1))-min(o_Vfc(:,1)));
fprintf('  FC ripple urban (sim/pred)      : %.3f / %.3f V\n', rip_urban, rip_pred_5k);
fprintf('  FC ripple highway (sim/pred)    : %.3f / %.3f V\n', rip_highway, rip_pred_20k);
fprintf('  fe @ urban 40km/h               : %.1f Hz\n', fe_urban);
fprintf('  fe @ highway 120km/h            : %.1f Hz\n', fe_highway);
if ~isnan(THD_urban)
    fprintf('  THD urban (5kHz)                : %.2f%%  Ifund=%.2fA rms\n', THD_urban, I_fund_urban);
end
if ~isnan(THD_highway)
    fprintf('  THD highway (20kHz)             : %.2f%%  Ifund=%.2fA rms\n', THD_highway, I_fund_highway);
end
fprintf('  Losses sched (ph A): cond=%.2fW  sw=%.2fW  total=%.2fW\n', ...
    P_cond_sched(1), P_sw_sched_ph(1), P_total_sched(1));
fprintf('  Losses fixed (ph A): cond=%.2fW  sw=%.2fW  total=%.2fW\n', ...
    P_cond_fixed(1), P_sw_fixed_ph(1), P_total_fixed(1));
fprintf('  Total (3-ph): sched=%.2fW  fixed=%.2fW\n', sum(P_total_sched), sum(P_total_fixed));
sw_saving = sum(P_sw_fixed_ph) - sum(P_sw_sched_ph);
fprintf('  Switching loss saving           : %.2f W (%.1f%%)\n', sw_saving, ...
    100*sw_saving/max(sum(P_sw_fixed_ph), 1e-9));
fprintf('  Tj max (sched/fixed)            : %.1f / %.1f C\n', max(Tj_sched), max(Tj_fixed));
fprintf('  Tj saving                       : %.2f C\n', max(Tj_fixed)-max(Tj_sched));
fprintf('\n');

%% 5. FIGURE GENERATION
% ========================================================================

fprintf('  Generating figures...\n');

% --- Fig 1: Drive Cycle ---
fig1 = figure('Position', [50 50 1000 600], 'Visible', 'off');
subplot(3,1,1);
area(t_arr, v_speed, 'FaceColor', [0.08 0.56 0.47], 'FaceAlpha', 0.25);
ylabel('Speed (km/h)'); title('Vehicle Speed Profile'); grid on; ylim([0 165]);

subplot(3,1,2);
plot(t_arr, fe_arr, 'Color', [0.12 0.22 0.39], 'LineWidth', 1.5);
ylabel('f_e (Hz)'); grid on;
title(sprintf('Electrical Frequency [P=%d, gear=%.2f:1, r_{tyre}=%.3fm]', P_poles, gear_ratio, r_tyre));

subplot(3,1,3);
stairs(t_arr, fsw_arr/1e3, 'Color', [0.84 0.54 0.06], 'LineWidth', 2);
hold on; yline(20, '--', 'Color', [0.5 0.55 0.55], 'LineWidth', 1);
ylabel('f_{sw} (kHz)'); xlabel('Time (s)');
title('Adaptive Switching Frequency Schedule'); grid on; ylim([0 25]);
legend('Scheduled', 'Fixed 20kHz baseline');

saveas(fig1, fullfile(OUT_DIR, 'fig1_drive_cycle.png'));
fprintf('    Saved fig1_drive_cycle.png\n');
close(fig1);

% --- Fig 2: Phase Currents and FC ---
fig2 = figure('Position', [50 50 1200 600], 'Visible', 'off');

subplot(2,2,1);
plot(t_arr, o_ia, 'Color', [0.12 0.22 0.39], 'LineWidth', 0.3); hold on;
plot(t_arr, o_ib, 'Color', [0.75 0.22 0.17], 'LineWidth', 0.3);
plot(t_arr, o_ic, 'Color', [0.12 0.52 0.29], 'LineWidth', 0.3);
ylabel('I (A)'); title('Full Drive Cycle'); grid on;
legend('i_a', 'i_b', 'i_c');

% Urban zoom
skip = round(length(idx_urban)/2);
n_show = min(round(5/fe_urban/dt), length(idx_urban)-skip);
iw = idx_urban(skip:skip+n_show-1);
tw = (t_arr(iw) - t_arr(iw(1))) * 1e3;
subplot(2,2,2);
plot(tw, o_ia(iw), 'Color', [0.12 0.22 0.39], 'LineWidth', 1); hold on;
plot(tw, o_ib(iw), 'Color', [0.75 0.22 0.17], 'LineWidth', 1);
plot(tw, o_ic(iw), 'Color', [0.12 0.52 0.29], 'LineWidth', 1);
ylabel('I (A)'); xlabel('Time (ms)');
title(sprintf('Urban 40km/h  f_e=%.1fHz  I_{pk,exp}=%.1fA', fe_urban, Ipk_urban));
grid on; legend('i_a', 'i_b', 'i_c');

% Highway zoom
skip = round(length(idx_highway)/2);
n_show = min(round(5/fe_highway/dt), length(idx_highway)-skip);
iw = idx_highway(skip:skip+n_show-1);
tw = (t_arr(iw) - t_arr(iw(1))) * 1e3;
subplot(2,2,3);
plot(tw, o_ia(iw), 'Color', [0.12 0.22 0.39], 'LineWidth', 1); hold on;
plot(tw, o_ib(iw), 'Color', [0.75 0.22 0.17], 'LineWidth', 1);
plot(tw, o_ic(iw), 'Color', [0.12 0.52 0.29], 'LineWidth', 1);
ylabel('I (A)'); xlabel('Time (ms)');
title(sprintf('Highway 120km/h  f_e=%.1fHz  I_{pk,exp}=%.1fA', fe_highway, Ipk_highway));
grid on; legend('i_a', 'i_b', 'i_c');

% FC voltages
subplot(2,2,4);
plot(t_arr, o_Vfc(:,1), 'Color', [0.12 0.22 0.39], 'LineWidth', 0.4); hold on;
plot(t_arr, o_Vfc(:,2), 'Color', [0.75 0.22 0.17], 'LineWidth', 0.4);
plot(t_arr, o_Vfc(:,3), 'Color', [0.12 0.52 0.29], 'LineWidth', 0.4);
yline(Vc_ref, 'k--', 'LineWidth', 1.2);
ylabel('V_{fc} (V)'); xlabel('Time (s)');
title(sprintf('FC Voltages (V_{ref}=%.0fV)', Vc_ref)); grid on;
legend('V_{fc,a}', 'V_{fc,b}', 'V_{fc,c}');

saveas(fig2, fullfile(OUT_DIR, 'fig2_currents_fc.png'));
fprintf('    Saved fig2_currents_fc.png\n');
close(fig2);

% --- Fig 3: Thermal ---
fig3 = figure('Position', [50 50 1100 700], 'Visible', 'off');

subplot(3,1,1);
plot(t_arr, P_inst_sched, 'Color', [0.12 0.22 0.39], 'LineWidth', 0.4); hold on;
plot(t_arr, P_inst_fixed, 'Color', [0.75 0.22 0.17], 'LineWidth', 0.4);
ylabel('P_{loss} (W)'); title('Instantaneous Loss - Phase A'); grid on;
legend('Scheduled', 'Fixed 20kHz');

subplot(3,1,2);
plot(t_arr, Tj_sched, 'Color', [0.12 0.22 0.39], 'LineWidth', 1.5); hold on;
plot(t_arr, Tj_fixed, 'Color', [0.75 0.22 0.17], 'LineWidth', 1.5);
yline(150, '--', 'Color', [0.75 0.22 0.17], 'LineWidth', 1.2);
ylabel('T_j (C)'); title('Junction Temperature (4-layer Foster)'); grid on;
legend(sprintf('Scheduled (T_{j,max}=%.1fC)', max(Tj_sched)), ...
       sprintf('Fixed 20kHz (T_{j,max}=%.1fC)', max(Tj_fixed)), ...
       'Limit 150C');

subplot(3,1,3);
bar_data = [sum(P_cond_sched), sum(P_sw_sched_ph); sum(P_cond_fixed), sum(P_sw_fixed_ph)];
bar(bar_data, 'stacked');
set(gca, 'XTickLabel', {'Scheduled', 'Fixed 20kHz'});
ylabel('Average Power (W)'); title('Total Loss - All 3 Phases'); grid on;
legend('Conduction', 'Switching');

saveas(fig3, fullfile(OUT_DIR, 'fig3_thermal.png'));
fprintf('    Saved fig3_thermal.png\n');
close(fig3);

fprintf('\n  All figures saved to %s\n', OUT_DIR);
fprintf('==================================================================\n');

%% LOCAL FUNCTIONS
% ========================================================================

function [THD, I_fund_rms] = compute_thd_fn(signal, fe, dt_val)
    % Hanning-windowed THD computation
    N_min = round(20 / fe / dt_val);
    N_win = min(2^nextpow2(N_min), length(signal));
    seg = signal(end-N_win+1:end);
    
    window = hanning(N_win);
    seg_w = seg .* window;
    cg = sum(window) / N_win;
    
    spectrum = abs(fft(seg_w)) / (N_win * cg / 2);
    spectrum(1) = spectrum(1) / 2;
    f_axis = (0:N_win-1)' / (N_win * dt_val);
    half = floor(N_win / 2);
    
    [~, k1] = min(abs(f_axis(1:half) - fe));
    I_fund = spectrum(k1);
    I_fund_rms = I_fund / sqrt(2);
    
    I_h_sq = 0;
    for h = 2:50
        [~, kh] = min(abs(f_axis(1:half) - h*fe));
        I_h_sq = I_h_sq + spectrum(kh)^2;
    end
    
    THD = 100 * sqrt(I_h_sq) / max(I_fund, 1e-9);
end
