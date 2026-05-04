%% FC_Inverter_Run.m
%  Runs FC_Inverter_Simulation_v2 logic and exports:
%    fc_inverter_results/results.json
%    fc_inverter_results/summary.md
%    fc_inverter_results/fig1_drive_cycle.png  … fig6_thd_spectra.png
%  All outputs are structured for agent-assisted report generation.
% =========================================================================
clear; close all; clc;

OUT_DIR = fullfile(fileparts(mfilename('fullpath')), 'fc_inverter_results');
if ~exist(OUT_DIR,'dir'), mkdir(OUT_DIR); end

%% =========================================================================
%  1. SYSTEM PARAMETERS
% =========================================================================
Vdc_total = 800;   Vdc = Vdc_total/2;
R_load    = 1.3;   L_load = 2e-3;
C_fc      = 1.5e-3;
Vc_ref    = Vdc/2;   % 200 V

Rds_on    = 65e-3;   Vf_body = 1.8;
E_on_ref  = 85e-6;   E_off_ref = 45e-6;
I_ref = 20;   V_ref = 600;

Rth_jc = 0.38;   Rth_ch = 0.10;   T_ambient = 40;   tau_th = 0.05;

COMPARE_FIXED_FSW = true;
dt = 2e-6;

Z_load = @(fe) sqrt(R_load^2 + (2*pi*fe*L_load)^2);
m_index = 0.9;
phi_abc = [0, -2*pi/3, 2*pi/3];

fprintf('=== FC Inverter Simulation (MATLAB + export) ===\n');
fprintf('Vdc=%dV  R=%.1f  L=%.0fmH  C_fc=%.1fmF  dt=%.0fus\n\n',...
    Vdc_total, R_load, L_load*1e3, C_fc*1e3, dt*1e6);

%% =========================================================================
%  2. DRIVE CYCLE
% =========================================================================
r_tyre     = 0.315;
gear_ratio = 8.19;
P_poles    = 8;
s2fe = @(v) max((P_poles/2)*((v/3.6)/r_tyre)*(gear_ratio/(2*pi)), 1.0);

dc_spd = [0;  20;  40;  60;  20;   0;   80;  120; 140; 100];
dc_dur = [0.05;0.10;0.20;0.20;0.10;0.05;0.20;0.20;0.20;0.20];
dc_nm  = {"Standstill","Urban low","Urban mid","Urban high","Decel","Stop",...
           "Transition","Highway","Highway fast","Highway cruise"};
Ns = numel(dc_spd);

dc_t=[]; dc_v=[]; dc_fe=[]; dc_fsw=[];
tc=0;
for i=1:Ns
    v=dc_spd(i); d=dc_dur(i); fe_i=s2fe(v);
    if    v<60,  fsw_i=5e3;
    elseif v<80, fsw_i=10e3;
    else,        fsw_i=20e3; end
    n = round(d/dt);
    ts = tc + (0:n-1)*dt;
    dc_t  =[dc_t,   ts];
    dc_v  =[dc_v,   v    *ones(1,n)];
    dc_fe =[dc_fe,  fe_i *ones(1,n)];
    dc_fsw=[dc_fsw, fsw_i*ones(1,n)];
    tc=tc+d;
end
N = numel(dc_t);

fprintf('%-18s %8s %9s %9s %9s %12s\n','Segment','v(km/h)','fe(Hz)','fsw(kHz)','dur(s)','Ipk_est(A)');
fprintf('%s\n',repmat('-',68,1));
for i=1:Ns
    v=dc_spd(i); fe_i=s2fe(v);
    if v<60,fsw_p=5;elseif v<80,fsw_p=10;else,fsw_p=20;end
    fprintf('%-18s %8.0f %9.1f %9.0f %9.2f %12.1f\n',...
        dc_nm{i},v,fe_i,fsw_p,dc_dur(i),m_index*Vdc/Z_load(fe_i));
end
fprintf('\n');

%% =========================================================================
%  3. MAIN SIMULATION LOOP
% =========================================================================
ia=0; ib=0; ic=0;
Vfc_a=Vc_ref; Vfc_b=Vc_ref; Vfc_c=Vc_ref;

o_ia=zeros(1,N); o_ib=zeros(1,N); o_ic=zeros(1,N);
o_Van=zeros(1,N); o_Vbn=zeros(1,N); o_Vcn=zeros(1,N);
o_Vfc_a=zeros(1,N); o_Vfc_b=zeros(1,N); o_Vfc_c=zeros(1,N);

Ec_s=zeros(3,4); Es_s=zeros(3,4);
Ec_f=zeros(3,4); Es_f=zeros(3,4);
Pi_s=zeros(1,N); Pi_f=zeros(1,N);

cp=0;
% Sample-and-hold: FC zero-state direction is chosen once per switching period
% and held for the full period, giving physically realistic FC voltage ripple.
ifc_sa_hold=0; ifc_sb_hold=0; ifc_sc_hold=0;

prog=round(linspace(1,N,51));
fprintf('Running %d steps: [',N);
tic;
for k=1:N
    if any(k==prog), fprintf('='); end
    t=dc_t(k); fe=dc_fe(k); fsw=dc_fsw(k);

    % Carrier phase: detect switching period boundary (cp wrapping through 1)
    cp_new = cp + fsw*dt;
    period_start = (cp_new >= 1.0);
    if period_start, cp_new = cp_new - 1.0; end
    cp = cp_new;
    if cp<0.5, tri=2*cp; else, tri=2*(1-cp); end
    C_up=tri; C_lo=-tri;

    we=2*pi*fe;
    va_r = m_index*sin(we*t+phi_abc(1));
    vb_r = m_index*sin(we*t+phi_abc(2));
    vc_r = m_index*sin(we*t+phi_abc(3));

    sw_a = pdpwm(va_r, C_up, C_lo);
    sw_b = pdpwm(vb_r, C_up, C_lo);
    sw_c = pdpwm(vc_r, C_up, C_lo);

    % FC zero-state selection — sample-and-hold at switching period boundaries.
    % Direction is decided once per switching cycle and held for the full period,
    % so the FC integrates charge over the full zero-state interval rather than
    % being corrected every 2µs. This produces the physically correct ~4-12V ripple.
    % Rule: choose the zero state that drives Vfc toward Vc_ref:
    %   ifc_sign = sign(i_phase) * sign(Vc_ref - Vfc)
    if period_start || k == 1
        % sign(error) returns 0 when Vfc==Vc_ref exactly, which prevents any
        % initial charging. Use >= comparison so "at reference" defaults to charge.
        if abs(ia)>1e-9, ifc_sa_hold=sign(ia)*(2*(Vfc_a<=Vc_ref)-1); else, ifc_sa_hold=0; end
        if abs(ib)>1e-9, ifc_sb_hold=sign(ib)*(2*(Vfc_b<=Vc_ref)-1); else, ifc_sb_hold=0; end
        if abs(ic)>1e-9, ifc_sc_hold=sign(ic)*(2*(Vfc_c<=Vc_ref)-1); else, ifc_sc_hold=0; end
    end
    % Zero in non-zero states (FC bypassed at output levels ±1)
    if sw_a~=0, ifc_sa=0; else, ifc_sa=ifc_sa_hold; end
    if sw_b~=0, ifc_sb=0; else, ifc_sb=ifc_sb_hold; end
    if sw_c~=0, ifc_sc=0; else, ifc_sc=ifc_sc_hold; end

    Vap=sw_a*Vdc; Vbp=sw_b*Vdc; Vcp=sw_c*Vdc;
    Vn=(Vap+Vbp+Vcp)/3;
    Van=Vap-Vn; Vbn=Vbp-Vn; Vcn=Vcp-Vn;

    ia2=ia+(Van-R_load*ia)/L_load*dt;
    ib2=ib+(Vbn-R_load*ib)/L_load*dt;
    ic2=ic+(Vcn-R_load*ic)/L_load*dt;
    im=(ia2+ib2+ic2)/3;  ia2=ia2-im; ib2=ib2-im; ic2=ic2-im;

    Vfc_a = Vfc_a + ifc_sa*ia/C_fc*dt;
    Vfc_b = Vfc_b + ifc_sb*ib/C_fc*dt;
    Vfc_c = Vfc_c + ifc_sc*ic/C_fc*dt;

    Iv = [abs(ia), abs(ib), abs(ic)];
    swv = [sw_a, sw_b, sw_c];
    for ph=1:3
        Ip=Iv(ph); sw=swv(ph);
        if sw==1,     qon=[1,1,0,0];
        elseif sw==-1,qon=[0,0,1,1];
        else,         qon=[0,1,0,1]; end
        for s=1:4
            if qon(s)
                dEc = Rds_on*Ip^2*dt;
                Ec_s(ph,s)=Ec_s(ph,s)+dEc;
                Ec_f(ph,s)=Ec_f(ph,s)+dEc;
            end
        end
        Esw = (E_on_ref+E_off_ref)*(Ip/I_ref)*(Vdc/V_ref);
        Es_s(ph,1)=Es_s(ph,1)+Esw*fsw*dt;
        Es_f(ph,1)=Es_f(ph,1)+Esw*20e3*dt;
    end

    Ia=abs(ia); Esw_a=(E_on_ref+E_off_ref)*(Ia/I_ref)*(Vdc/V_ref);
    Pi_s(k)=Rds_on*Ia^2*2 + Esw_a*fsw;
    Pi_f(k)=Rds_on*Ia^2*2 + Esw_a*20e3;

    o_ia(k)=ia; o_ib(k)=ib; o_ic(k)=ic;
    o_Van(k)=Van; o_Vbn(k)=Vbn; o_Vcn(k)=Vcn;
    o_Vfc_a(k)=Vfc_a; o_Vfc_b(k)=Vfc_b; o_Vfc_c(k)=Vfc_c;

    ia=ia2; ib=ib2; ic=ic2;
end
elapsed = toc;
fprintf(']\n  Done in %.1fs\n\n', elapsed);

%% =========================================================================
%  4. THERMAL MODEL
% =========================================================================
Tsim=dc_t(end);

Pc_s=sum(Ec_s,2)/Tsim; Ps_s=sum(Es_s,2)/Tsim; Pt_s=Pc_s+Ps_s;
Pc_f=sum(Ec_f,2)/Tsim; Ps_f=sum(Es_f,2)/Tsim; Pt_f=Pc_f+Ps_f;

% Pi_s = 2*Rds_on*Ia^2 + Esw_a*fsw  (2 switches conduct simultaneously).
% Thermal resistance Rth_jc is per device, so divide by 2 to get per-switch power.
Tj_s=T_ambient+(max(Pt_s)/2)*(Rth_jc+Rth_ch);
Tj_f=T_ambient+(max(Pt_f)/2)*(Rth_jc+Rth_ch);

win=min(round(tau_th/dt),round(N/4)); blp=ones(1,win)/win;
Tj_tr_s=T_ambient+filter(blp,1,Pi_s/2)*(Rth_jc+Rth_ch);
Tj_tr_f=T_ambient+filter(blp,1,Pi_f/2)*(Rth_jc+Rth_ch);

%% =========================================================================
%  5. THD ANALYSIS
% =========================================================================
fe_u = s2fe(40);   fe_h = s2fe(120);

idx_u40  = find(dc_v>=38 & dc_v<=42);
idx_h120 = find(dc_v>=118 & dc_v<=122);

fprintf('--- THD Analysis ---\n');
THD_u=NaN; Ifu=NaN; f_u=[]; Ia_mag_u=[];
THD_h=NaN; Ifh=NaN; f_h=[]; Ia_mag_h=[];

if numel(idx_u40)>300
    [THD_u, Ifu, f_u, Ia_mag_u] = compute_thd(o_ia(idx_u40), fe_u, dt, 'Urban  40km/h');
else
    fprintf('  Urban segment too short\n');
end
if numel(idx_h120)>300
    [THD_h, Ifh, f_h, Ia_mag_h] = compute_thd(o_ia(idx_h120), fe_h, dt, 'Highway 120km/h');
else
    fprintf('  Highway segment too short\n');
end

%% =========================================================================
%  6. STEADY-STATE WINDOWS
% =========================================================================
[iw_u,tw_u,few_u] = sswin(idx_u40,  5, dc_fe, dc_t, dt, N);
[iw_h,tw_h,few_h] = sswin(idx_h120, 5, dc_fe, dc_t, dt, N);

%% =========================================================================
%  7. CONSOLE SUMMARY
% =========================================================================
fprintf('\n=========================================================\n');
fprintf('  SIMULATION RESULTS SUMMARY\n');
fprintf('=========================================================\n');
fprintf('  Peak phase current              : %.2f A\n', max(abs(o_ia)));
fprintf('  Peak Van                        : %.1f V\n', max(abs(o_Van)));
fprintf('  Ipeak expected @ urban  40km/h  : %.2f A\n', m_index*Vdc/Z_load(fe_u));
fprintf('  Ipeak expected @ highway120km/h : %.2f A\n', m_index*Vdc/Z_load(fe_h));
fprintf('  FC mean voltage                 : %.2f V  (ref=%.1f V)\n', mean(o_Vfc_a), Vc_ref);
fprintf('  FC pk-pk ripple (full cycle)    : %.2f V\n', max(o_Vfc_a)-min(o_Vfc_a));
fprintf('  Expected ripple @ 5kHz          : %.2f V\n', max(abs(o_ia))*0.5/(2*5e3*C_fc));
fprintf('  fe @ urban  40km/h              : %.1f Hz\n', fe_u);
fprintf('  fe @ highway120km/h             : %.1f Hz\n', fe_h);
if ~isnan(THD_u)
    fprintf('  THD urban  (5kHz)               : %.2f%%  Ifund=%.2fA rms\n', THD_u, Ifu);
end
if ~isnan(THD_h)
    fprintf('  THD highway (20kHz)             : %.2f%%  Ifund=%.2fA rms\n', THD_h, Ifh);
end
fprintf('  Losses sched : cond=%.3fW  sw=%.3fW  total=%.3fW\n',Pc_s(1),Ps_s(1),Pt_s(1));
fprintf('  Losses fixed : cond=%.3fW  sw=%.3fW  total=%.3fW\n',Pc_f(1),Ps_f(1),Pt_f(1));
fprintf('  Tj steady (sched/fixed)         : %.1f / %.1f C\n', Tj_s, Tj_f);
fprintf('  Tj saving from scheduling       : %.2f C\n', Tj_f-Tj_s);

%% =========================================================================
%  8. JSON EXPORT
% =========================================================================
% Ripple predictions
rip_pred_5  = m_index*Vdc/Z_load(s2fe(40))  * 0.5/(2*5e3 *C_fc);
rip_pred_10 = m_index*Vdc/Z_load(s2fe(80))  * 0.5/(2*10e3*C_fc);
rip_pred_20 = m_index*Vdc/Z_load(s2fe(120)) * 0.5/(2*20e3*C_fc);

if ~isempty(iw_u), rip_sim_u=max(o_Vfc_a(iw_u))-min(o_Vfc_a(iw_u)); else, rip_sim_u=NaN; end
if ~isempty(iw_h), rip_sim_h=max(o_Vfc_a(iw_h))-min(o_Vfc_a(iw_h)); else, rip_sim_h=NaN; end

% Build segment array
seg_arr = cell(1,Ns);
for i=1:Ns
    v=dc_spd(i); fe_i=s2fe(v);
    if v<60,fk=5;elseif v<80,fk=10;else,fk=20;end
    seg_arr{i} = struct(...
        'index',      i-1,...
        'name',       dc_nm{i},...
        'speed_kmh',  v,...
        'fe_Hz',      round(fe_i,2),...
        'fsw_kHz',    fk,...
        'duration_s', dc_dur(i),...
        'Ipeak_expected_A', round(m_index*Vdc/Z_load(fe_i),2));
end

R_out = struct();

R_out.simulation_info = struct(...
    'script_version',    'FC_Inverter_Run_MATLAB',...
    'topology',          'Three-phase three-level Flying Capacitor Inverter',...
    'application',       'EV traction drive (IEEE conference paper)',...
    'time_step_us',      dt*1e6,...
    'total_sim_time_s',  round(Tsim,4),...
    'total_steps',       N,...
    'wall_time_s',       round(elapsed,1));

R_out.circuit_parameters = struct(...
    'Vdc_bus_V',           Vdc_total,...
    'Vdc_half_rail_V',     Vdc,...
    'R_load_ohm',          R_load,...
    'L_load_mH',           L_load*1e3,...
    'C_fc_mF',             C_fc*1e3,...
    'Vc_fc_reference_V',   Vc_ref,...
    'modulation_index',    m_index,...
    'device_model',        'SiC MOSFET Wolfspeed C3M0065090D',...
    'Rds_on_mohm',         Rds_on*1e3,...
    'E_on_ref_uJ',         E_on_ref*1e6,...
    'E_off_ref_uJ',        E_off_ref*1e6,...
    'I_ref_A',             I_ref,...
    'V_ref_V',             V_ref);

R_out.drivetrain = struct(...
    'tyre_radius_m',   r_tyre,...
    'gear_ratio',      gear_ratio,...
    'motor_poles',     P_poles,...
    'formula',         'fe = (P/2) * (v_kmh/3.6 / r_tyre) * (gear_ratio / 2pi)',...
    'fe_at_40kmh_Hz',  round(s2fe(40),2),...
    'fe_at_80kmh_Hz',  round(s2fe(80),2),...
    'fe_at_120kmh_Hz', round(s2fe(120),2),...
    'fe_at_140kmh_Hz', round(s2fe(140),2),...
    'v1_error_note',   'v1 used flat 400 RPM/(km/h) giving ~3200 Hz at 120 km/h; corrected with tyre+gear model');

R_out.drive_cycle_segments = seg_arr;

R_out.electrical_results = struct(...
    'peak_phase_current_A',            round(max(abs(o_ia)),3),...
    'peak_Van_V',                      round(max(abs(o_Van)),2),...
    'Ipeak_expected_urban_40kmh_A',    round(m_index*Vdc/Z_load(fe_u),3),...
    'Ipeak_expected_highway_120kmh_A', round(m_index*Vdc/Z_load(fe_h),3),...
    'Z_load_urban_40kmh_ohm',          round(Z_load(fe_u),4),...
    'Z_load_highway_120kmh_ohm',       round(Z_load(fe_h),4));

R_out.flying_capacitor = struct(...
    'Vc_reference_V',                  Vc_ref,...
    'mean_voltage_phase_a_V',          round(mean(o_Vfc_a),3),...
    'pk_pk_ripple_full_cycle_V',       round(max(o_Vfc_a)-min(o_Vfc_a),3),...
    'pk_pk_ripple_urban_simulated_V',  round(rip_sim_u,3),...
    'pk_pk_ripple_highway_simulated_V',round(rip_sim_h,3),...
    'pk_pk_ripple_predicted_5kHz_V',   round(rip_pred_5,3),...
    'pk_pk_ripple_predicted_10kHz_V',  round(rip_pred_10,3),...
    'pk_pk_ripple_predicted_20kHz_V',  round(rip_pred_20,3),...
    'ripple_formula',                  'DeltaV = I_peak * 0.5 / (2 * fsw * C_fc)',...
    'balancing_method',                'Natural PD-PWM zero-state selection (FIX 2)');

if isnan(THD_u), thd_u_val=[]; ifu_val=[]; else, thd_u_val=round(THD_u,3); ifu_val=round(Ifu,3); end
if isnan(THD_h), thd_h_val=[]; ifh_val=[]; else, thd_h_val=round(THD_h,3); ifh_val=round(Ifh,3); end

R_out.thd = struct(...
    'urban_40kmh', struct(...
        'fe_Hz',        round(fe_u,2),...
        'fsw_kHz',      5,...
        'fsw_fe_ratio', round(5e3/fe_u,1),...
        'thd_percent',  thd_u_val,...
        'I_fund_rms_A', ifu_val),...
    'highway_120kmh', struct(...
        'fe_Hz',        round(fe_h,2),...
        'fsw_kHz',      20,...
        'fsw_fe_ratio', round(20e3/fe_h,1),...
        'thd_percent',  thd_h_val,...
        'I_fund_rms_A', ifh_val));

R_out.losses_W = struct(...
    'note', ['Conduction losses identical (same current both cases). '...
             'Switching losses differ by fsw ratio.'],...
    'scheduled_fsw_per_phase', struct(...
        'conduction_W', round(Pc_s(1),4),...
        'switching_W',  round(Ps_s(1),4),...
        'total_W',      round(Pt_s(1),4)),...
    'fixed_20kHz_per_phase', struct(...
        'conduction_W', round(Pc_f(1),4),...
        'switching_W',  round(Ps_f(1),4),...
        'total_W',      round(Pt_f(1),4)),...
    'all_phases_scheduled_total_W',  round(sum(Pt_s),3),...
    'all_phases_fixed20kHz_total_W', round(sum(Pt_f),3),...
    'switching_loss_saving_W',       round(sum(Pt_f)-sum(Pt_s),3));

R_out.thermal = struct(...
    'T_ambient_C',      T_ambient,...
    'Rth_jc_K_per_W',  Rth_jc,...
    'Rth_ch_K_per_W',  Rth_ch,...
    'tau_thermal_s',   tau_th,...
    'Tj_max_limit_C',  150,...
    'scheduled_fsw', struct(...
        'Tj_steady_state_C',     round(Tj_s,2),...
        'Tj_max_transient_C',    round(max(Tj_tr_s),2),...
        'Tj_margin_to_limit_C',  round(150-max(Tj_tr_s),2)),...
    'fixed_20kHz', struct(...
        'Tj_steady_state_C',     round(Tj_f,2),...
        'Tj_max_transient_C',    round(max(Tj_tr_f),2),...
        'Tj_margin_to_limit_C',  round(150-max(Tj_tr_f),2)),...
    'Tj_saving_from_scheduling_C', round(Tj_f-Tj_s,3));

R_out.figures = {...
    struct('file','fig1_drive_cycle.png',...
           'title','Drive Cycle: Speed, Electrical Frequency, and Switching Frequency Schedule',...
           'description',['Three-panel plot: vehicle speed, fe from corrected drivetrain model, '...
                          'adaptive fsw schedule vs fixed 20kHz baseline.']),...
    struct('file','fig2_output_voltages.png',...
           'title','Three-Level FC Output Voltages: Urban vs Highway Steady State',...
           'description',['Van/Vbn and Vab at urban 40km/h (left) and highway 120km/h (right). '...
                          'Three voltage levels visible (+-400V, 0V).']),...
    struct('file','fig3_phase_currents.png',...
           'title','Three-Phase Output Currents: Full Cycle and Steady-State Zooms',...
           'description',['Full drive cycle ia/ib/ic; urban and highway zooms with expected peak; '...
                          'urban current harmonic spectrum normalised to harmonic order.']),...
    struct('file','fig4_fc_voltages.png',...
           'title','Flying Capacitor Voltages: Balance and Ripple',...
           'description',['All three FC voltages vs 200V reference; urban and highway ripple zooms; '...
                          'bar chart of predicted vs simulated ripple at 5/10/20 kHz.']),...
    struct('file','fig5_thermal.png',...
           'title','Semiconductor Loss and Junction Temperature: Scheduled vs Fixed fsw',...
           'description',['Instantaneous loss trace; Tj transient with 150C limit; '...
                          'stacked bar of avg conduction+switching loss all phases.']),...
    struct('file','fig6_thd_spectra.png',...
           'title','Output Current Harmonic Spectra: Urban and Highway',...
           'description',['Stem plot of harmonic magnitude vs order (to 50th) and full kHz spectrum. '...
                          'Urban 40km/h (5kHz) left; highway 120km/h (20kHz) right. THD annotated.'])};

json_str = jsonencode(R_out, 'PrettyPrint', true);
fid = fopen(fullfile(OUT_DIR,'results.json'),'w');
fprintf(fid,'%s',json_str);
fclose(fid);
fprintf('\nSaved results.json\n');

%% =========================================================================
%  9. SUMMARY.MD
% =========================================================================
fid = fopen(fullfile(OUT_DIR,'summary.md'),'w','n','UTF-8');
fprintf(fid,'# FC Inverter Simulation — Key Findings\n\n');
fprintf(fid,'## Simulation Setup\n');
fprintf(fid,'- Topology: Three-phase three-level Flying Capacitor Inverter\n');
fprintf(fid,'- DC bus: %d V (±%d V rails)\n', Vdc_total, Vdc);
fprintf(fid,'- Load: R=%.1f Ω, L=%.0f mH per phase\n', R_load, L_load*1e3);
fprintf(fid,'- Flying capacitor: %.1f mF, reference voltage %.0f V\n', C_fc*1e3, Vc_ref);
fprintf(fid,'- Device: SiC MOSFET Wolfspeed C3M0065090D (Rds_on=%.0f mΩ)\n', Rds_on*1e3);
fprintf(fid,'- Modulation index: %.1f\n', m_index);
fprintf(fid,'- Simulation step: %.0f µs, total time: %.2f s\n\n', dt*1e6, Tsim);

fprintf(fid,'## Drive Cycle\n');
fprintf(fid,'| Segment | Speed (km/h) | fe (Hz) | fsw (kHz) | Duration (s) | Ipeak expected (A) |\n');
fprintf(fid,'|---------|-------------|---------|-----------|-------------|-------------------|\n');
for i=1:Ns
    v=dc_spd(i); fe_i=s2fe(v);
    if v<60,fk=5;elseif v<80,fk=10;else,fk=20;end
    fprintf(fid,'| %s | %.0f | %.1f | %d | %.2f | %.1f |\n',...
        dc_nm{i},v,fe_i,fk,dc_dur(i),m_index*Vdc/Z_load(fe_i));
end
fprintf(fid,'\n');

fprintf(fid,'## Electrical Frequency Correction (FIX 1)\n');
fprintf(fid,'- Correct model: fe = (P/2) × (v/3.6 / r_tyre) × (gear_ratio / 2π)\n');
fprintf(fid,'- At 40 km/h:  fe = **%.1f Hz** (v1 erroneously: %.0f Hz)\n', fe_u, 40*400*(P_poles/2)/60);
fprintf(fid,'- At 120 km/h: fe = **%.1f Hz** (v1 erroneously: %.0f Hz)\n\n', fe_h, 120*400*(P_poles/2)/60);

fprintf(fid,'## Peak Current\n');
fprintf(fid,'- Simulated peak: **%.2f A**\n', max(abs(o_ia)));
fprintf(fid,'- Expected at urban 40 km/h:    %.2f A\n', m_index*Vdc/Z_load(fe_u));
fprintf(fid,'- Expected at highway 120 km/h: %.2f A\n\n', m_index*Vdc/Z_load(fe_h));

fprintf(fid,'## Flying Capacitor Balance (FIX 2)\n');
fprintf(fid,'- Mean FC voltage (phase A): %.2f V  (reference: %.0f V)\n', mean(o_Vfc_a), Vc_ref);
fprintf(fid,'- Total pk-pk ripple across full cycle: %.2f V\n', max(o_Vfc_a)-min(o_Vfc_a));
fprintf(fid,'- Ripple at urban  (5 kHz, simulated):   %.2f V  | predicted: %.2f V\n', rip_sim_u, rip_pred_5);
fprintf(fid,'- Ripple at highway (20 kHz, simulated): %.2f V  | predicted: %.2f V\n\n', rip_sim_h, rip_pred_20);

fprintf(fid,'## THD (FIX 4: both operating points)\n');
if ~isnan(THD_u)
    fprintf(fid,'- Urban  40 km/h  (fsw=5 kHz,  fsw/fe=%.0f): THD = **%.2f%%**, I_fund = %.2f A rms\n',...
        5e3/fe_u, THD_u, Ifu);
end
if ~isnan(THD_h)
    fprintf(fid,'- Highway 120 km/h (fsw=20 kHz, fsw/fe=%.0f): THD = **%.2f%%**, I_fund = %.2f A rms\n\n',...
        20e3/fe_h, THD_h, Ifh);
end

fprintf(fid,'## Loss and Efficiency (FIX 3: symmetric conduction accounting)\n');
fprintf(fid,'Conduction losses are **identical** for both cases (same current waveform).\n\n');
fprintf(fid,'| | Scheduled fsw | Fixed 20 kHz |\n');
fprintf(fid,'|---|---|---|\n');
fprintf(fid,'| Conduction (per phase) | %.3f W | %.3f W |\n', Pc_s(1), Pc_f(1));
fprintf(fid,'| Switching (per phase)  | %.3f W | %.3f W |\n', Ps_s(1), Ps_f(1));
fprintf(fid,'| Total (per phase)      | %.3f W | %.3f W |\n', Pt_s(1), Pt_f(1));
fprintf(fid,'| Total (all phases)     | %.3f W | %.3f W |\n', sum(Pt_s), sum(Pt_f));
fprintf(fid,'| Switching loss saving  | **%.3f W** | — |\n\n', sum(Pt_f)-sum(Pt_s));

fprintf(fid,'## Thermal Performance\n');
fprintf(fid,'- Ambient: %.0f °C,  Rth_jc=%.2f K/W,  Rth_ch=%.2f K/W,  τ_th=%.0f ms\n',...
    T_ambient, Rth_jc, Rth_ch, tau_th*1e3);
fprintf(fid,'- Tj limit: 150 °C\n\n');
fprintf(fid,'| | Scheduled fsw | Fixed 20 kHz |\n');
fprintf(fid,'|---|---|---|\n');
fprintf(fid,'| Tj steady-state  | %.1f °C | %.1f °C |\n', Tj_s, Tj_f);
fprintf(fid,'| Tj max transient | %.1f °C | %.1f °C |\n', max(Tj_tr_s), max(Tj_tr_f));
fprintf(fid,'| Margin to 150 °C | %.1f °C | %.1f °C |\n', 150-max(Tj_tr_s), 150-max(Tj_tr_f));
fprintf(fid,'| Tj saving        | **%.2f °C** | — |\n\n', Tj_f-Tj_s);

fprintf(fid,'## Output Files\n');
fprintf(fid,'| File | Contents |\n');
fprintf(fid,'|------|----------|\n');
fprintf(fid,'| fig1_drive_cycle.png | Speed / fe / fsw schedule |\n');
fprintf(fid,'| fig2_output_voltages.png | Phase and line voltages at urban and highway |\n');
fprintf(fid,'| fig3_phase_currents.png | Three-phase currents and harmonic spectrum |\n');
fprintf(fid,'| fig4_fc_voltages.png | FC balance, ripple zoom, predicted vs simulated |\n');
fprintf(fid,'| fig5_thermal.png | Loss traces and junction temperature |\n');
fprintf(fid,'| fig6_thd_spectra.png | THD spectra at urban and highway |\n');
fprintf(fid,'| results.json | Complete structured numerical results |\n');
fprintf(fid,'| summary.md | This document |\n');
fclose(fid);
fprintf('Saved summary.md\n');

%% =========================================================================
%  10. FIGURES  (saved as PNG, no display required)
% =========================================================================
CL=struct('bl','#1F3864','rd','#C0392B','gr','#1E8449','or','#D68910',...
          'pu','#6C3483','gy','#7F8C8D','tl','#148F77');

function save_fig(fig, fname, OUT_DIR)
    exportgraphics(fig, fullfile(OUT_DIR, fname), 'Resolution', 150);
    fprintf('Saved %s\n', fname);
    close(fig);
end

% ── Fig 1: Drive Cycle ────────────────────────────────────────────────────
fig=figure('Visible','off','Position',[20 20 1100 600],'Color','w');
sgtitle('Drive Cycle: Speed \rightarrow f_e \rightarrow f_{sw} Schedule','FontSize',11,'FontWeight','bold');

subplot(3,1,1);
area(dc_t,dc_v,'FaceColor',CL.tl,'FaceAlpha',0.25,'EdgeColor',CL.tl,'LineWidth',1.2);
ylabel('Speed (km/h)'); title('Vehicle Speed Profile'); grid on;
xlim([0 dc_t(end)]); ylim([0 165]);
tc_acc=0; for i=1:Ns, xline(tc_acc,':','Color',CL.gy,'Alpha',0.5); tc_acc=tc_acc+dc_dur(i); end

subplot(3,1,2);
plot(dc_t,dc_fe,'Color',CL.bl,'LineWidth',1.5);
ylabel('f_e (Hz)'); grid on; xlim([0 dc_t(end)]);
title(sprintf('Electrical Frequency  [f_e=(P/2)\\cdotn/60  |  P=%d, gear=%.2f:1, r_{tyre}=%.3fm]',...
    P_poles,gear_ratio,r_tyre));

subplot(3,1,3);
stairs(dc_t,dc_fsw/1e3,'Color',CL.or,'LineWidth',2);
ylabel('f_{sw} (kHz)'); xlabel('Time (s)'); title('Adaptive Switching Frequency Schedule');
grid on; xlim([0 dc_t(end)]); ylim([0 25]);
yline(20,'--','Color',CL.gy,'Label','Fixed 20kHz','LabelHorizontalAlignment','left','LineWidth',1);
save_fig(fig,'fig1_drive_cycle.png',OUT_DIR);

% ── Fig 2: Output Voltages ────────────────────────────────────────────────
fig=figure('Visible','off','Position',[20 20 1200 580],'Color','w');
sgtitle('Three-Level FC Output Voltages (Urban vs Highway)','FontSize',11,'FontWeight','bold');
for col=1:2
    if col==1, iw=iw_u; tw=tw_u; few=few_u; flab=sprintf('Urban 40km/h  f_e=%.1fHz  f_{sw}=5kHz',few_u);
    else,      iw=iw_h; tw=tw_h; few=few_h; flab=sprintf('Highway 120km/h  f_e=%.1fHz  f_{sw}=20kHz',few_h); end
    if isempty(iw), continue; end
    subplot(2,2,col);
    plot(tw,o_Van(iw),'Color',CL.bl,'LineWidth',1.2); hold on;
    plot(tw,o_Vbn(iw),'Color',CL.rd,'LineWidth',1.0,'LineStyle','--');
    yline(Vdc,'--k','LineWidth',0.5); yline(-Vdc,'--k','LineWidth',0.5); yline(0,':k','LineWidth',0.5);
    ylabel('V (V)'); title(flab); legend('Van','Vbn'); grid on;
    subplot(2,2,col+2);
    plot(tw,o_Van(iw)-o_Vbn(iw),'Color',CL.pu,'LineWidth',1.2);
    ylabel('V_{ab} (V)'); xlabel('Time (ms)'); title('Line Voltage V_{ab}');
    grid on; yticks([-800,-400,0,400,800]);
end
save_fig(fig,'fig2_output_voltages.png',OUT_DIR);

% ── Fig 3: Phase Currents ─────────────────────────────────────────────────
fig=figure('Visible','off','Position',[20 20 1200 580],'Color','w');
sgtitle('Three-Phase Output Currents  (R=1.3\Omega, L=2mH)','FontSize',11,'FontWeight','bold');

subplot(2,2,1);
plot(dc_t,o_ia,'Color',CL.bl,'LineWidth',0.5); hold on;
plot(dc_t,o_ib,'Color',CL.rd,'LineWidth',0.5);
plot(dc_t,o_ic,'Color',CL.gr,'LineWidth',0.5);
ylabel('I (A)'); title('Full Drive Cycle'); grid on; xlim([0 dc_t(end)]);
legend('i_a','i_b','i_c','Location','best');

iw_list = {iw_u, iw_h}; tw_list={tw_u,tw_h};
few_list=[few_u,few_h]; fsw_lbl=[5,20]; lbl_list={'Urban 40km/h','Highway 120km/h'};
spidx=[2,3];
for col=1:2
    iw=iw_list{col}; tw=tw_list{col};
    if isempty(iw), continue; end
    Ipk_exp=m_index*Vdc/Z_load(few_list(col));
    subplot(2,2,spidx(col));
    plot(tw,o_ia(iw),'Color',CL.bl,'LineWidth',1.4); hold on;
    plot(tw,o_ib(iw),'Color',CL.rd,'LineWidth',1.4);
    plot(tw,o_ic(iw),'Color',CL.gr,'LineWidth',1.4);
    ylabel('I (A)'); xlabel('Time (ms)');
    title(sprintf('%s  f_e=%.1fHz  f_{sw}=%dkHz  I_{pk,exp}=%.1fA',...
        lbl_list{col},few_list(col),fsw_lbl(col),Ipk_exp));
    grid on; legend('i_a','i_b','i_c','Location','best');
end
subplot(2,2,4);
if numel(idx_u40)>100
    sg=o_ia(idx_u40); Nsg=numel(sg);
    fa=(0:Nsg-1)/(Nsg*dt); Ima=abs(fft(sg))/(Nsg/2);
    plot(fa(1:Nsg/2)/fe_u, Ima(1:Nsg/2),'Color',CL.tl,'LineWidth',1.0);
    xlabel('Harmonic order'); ylabel('A'); title('Urban Current Spectrum'); grid on; xlim([0 40]);
    xline(dc_fsw(idx_u40(1))/fe_u,'--r','f_{sw}/f_e','LineWidth',1);
end
save_fig(fig,'fig3_phase_currents.png',OUT_DIR);

% ── Fig 4: FC Voltages ────────────────────────────────────────────────────
fig=figure('Visible','off','Position',[20 20 1200 520],'Color','w');
sgtitle('Flying Capacitor Voltages  (V_{ref} = 200V)','FontSize',11,'FontWeight','bold');

subplot(2,2,1);
plot(dc_t,o_Vfc_a,'Color',CL.bl,'LineWidth',0.6); hold on;
plot(dc_t,o_Vfc_b,'Color',CL.rd,'LineWidth',0.6);
plot(dc_t,o_Vfc_c,'Color',CL.gr,'LineWidth',0.6);
yline(Vc_ref,'--k','V_{ref}=200V','LineWidth',1.2,'LabelHorizontalAlignment','left');
ylabel('V_{fc} (V)'); title('All Phases — Full Cycle'); grid on; xlim([0 dc_t(end)]);
legend('V_{fc,a}','V_{fc,b}','V_{fc,c}');

for col=1:2
    if col==1, iw=iw_u; flab='Urban 40km/h (f_{sw}=5kHz)';
    else,      iw=iw_h; flab='Highway 120km/h (f_{sw}=20kHz)'; end
    if isempty(iw), continue; end
    subplot(2,2,col+1);
    tfc=(dc_t(iw)-dc_t(iw(1)))*1e3;
    plot(tfc,o_Vfc_a(iw),'Color',CL.bl,'LineWidth',1.4);
    yline(Vc_ref,'--k','V_{ref}','LineWidth',1.2);
    rip=max(o_Vfc_a(iw))-min(o_Vfc_a(iw));
    title(sprintf('%s — Ripple: %.2fV pk-pk',flab,rip));
    ylabel('V_{fc,a} (V)'); xlabel('Time (ms)'); grid on;
end
subplot(2,2,4);
fsw_v=[5,10,20]; rip_pred_v=[rip_pred_5,rip_pred_10,rip_pred_20];
bar(fsw_v,rip_pred_v,'FaceColor',CL.tl,'FaceAlpha',0.55); hold on;
rip_sim_vals=[rip_sim_u, NaN, rip_sim_h];
scatter(fsw_v,rip_sim_vals,80,[192 57 43]/255,'filled');  % hex2rgb(CL.rd)
xlabel('f_{sw} (kHz)'); ylabel('\DeltaV_{fc} (V)');
title('FC Ripple: Predicted vs Simulated'); grid on;
legend('Predicted','Simulated','Location','ne');
save_fig(fig,'fig4_fc_voltages.png',OUT_DIR);

% ── Fig 5: Thermal ────────────────────────────────────────────────────────
fig=figure('Visible','off','Position',[20 20 1200 660],'Color','w');
sgtitle('Semiconductor Loss & Junction Temperature: Scheduled vs Fixed f_{sw}',...
    'FontSize',11,'FontWeight','bold');

subplot(3,1,1);
plot(dc_t,Pi_s,'Color',CL.bl,'LineWidth',0.7); hold on;
if COMPARE_FIXED_FSW
    plot(dc_t,Pi_f,'Color',CL.rd,'LineWidth',0.7,'LineStyle','--');
    legend('Scheduled f_{sw}','Fixed 20kHz','Location','best');
end
ylabel('P_{loss} (W)'); title('Instantaneous Loss — Phase A'); grid on; xlim([0 dc_t(end)]);

subplot(3,1,2);
plot(dc_t,Tj_tr_s,'Color',CL.bl,'LineWidth',1.5); hold on;
if COMPARE_FIXED_FSW
    plot(dc_t,Tj_tr_f,'Color',CL.rd,'LineWidth',1.5,'LineStyle','--');
    legend(sprintf('Scheduled (T_{j,max}=%.1f°C)',max(Tj_tr_s)),...
           sprintf('Fixed 20kHz (T_{j,max}=%.1f°C)',max(Tj_tr_f)),'Location','best');
end
yline(150,'--','Color',CL.rd,'Label','T_{j,max}=150°C','LineWidth',1.2);
ylabel('T_j (°C)'); title(sprintf('Junction Temperature (\\tau_{th}=%.0fms)',tau_th*1e3));
grid on; xlim([0 dc_t(end)]);

subplot(3,1,3);
cats=categorical({'Scheduled f_{sw}','Fixed 20kHz'});
lm=[sum(Pc_s),sum(Ps_s); sum(Pc_f),sum(Ps_f)];
bh=bar(cats,lm,'stacked');
bh(1).FaceColor=CL.bl; bh(2).FaceColor=CL.or;
ylabel('Average Power (W)');
title('Total Loss — All 3 Phases  (Conduction=equal; Switching differs by f_{sw})');
legend('Conduction','Switching','Location','best'); grid on;
save_fig(fig,'fig5_thermal.png',OUT_DIR);

% ── Fig 6: THD Spectra ────────────────────────────────────────────────────
fig=figure('Visible','off','Position',[20 20 1200 500],'Color','w');
sgtitle('Output Current Harmonic Spectrum — Urban and Highway','FontSize',11,'FontWeight','bold');

seg_info = {idx_u40, fe_u, 5e3, THD_u, CL.bl, 'Urban 40km/h';
            idx_h120, fe_h, 20e3, THD_h, CL.tl, 'Highway 120km/h'};
for col=1:2
    idx_seg=seg_info{col,1}; fe_seg=seg_info{col,2}; fsw_seg=seg_info{col,3};
    thd_val=seg_info{col,4}; clr=seg_info{col,5}; lbl_str=seg_info{col,6};
    if isnan(thd_val) || isempty(idx_seg), continue; end
    sg=o_ia(idx_seg); Nsg=numel(sg);
    fa=(0:Nsg-1)/(Nsg*dt); Im=abs(fft(sg))/(Nsg/2);
    half=floor(Nsg/2);

    subplot(2,2,col);
    stem(fa(1:half)/fe_seg, Im(1:half),'filled','Color',clr,'LineWidth',0.7,'MarkerSize',3);
    xlim([0 50]); xlabel('Harmonic order'); ylabel('I (A)');
    title(sprintf('%s  f_e=%.1fHz  THD=%.2f%%', lbl_str, fe_seg, thd_val)); grid on;
    xline(fsw_seg/fe_seg,'--r','f_{sw}/f_e','LineWidth',1);

    subplot(2,2,col+2);
    f_max=min(5*fsw_seg,80e3);
    mask=fa(1:half)<f_max;
    plot(fa(mask)/1e3, Im(mask),'Color',clr,'LineWidth',1.0);
    xlabel('Frequency (kHz)'); ylabel('I (A)');
    title(sprintf('Full Spectrum to %.0fkHz', f_max/1e3)); grid on;
    xline(fsw_seg/1e3,'--r',sprintf('f_{sw}=%dkHz',fsw_seg/1e3),'LineWidth',1);
end
save_fig(fig,'fig6_thd_spectra.png',OUT_DIR);

fprintf('\n=== Done. All outputs in ''%s'' ===\n', OUT_DIR);
fprintf('  results.json  — structured numerical data\n');
fprintf('  summary.md    — markdown findings for agent\n');
fprintf('  fig1-fig6     — PNG figures (150 dpi)\n');

%% =========================================================================
%  LOCAL FUNCTIONS
% =========================================================================
function sw_level = pdpwm(v_ref, C_up, C_lo)
    % Returns output level {-1, 0, +1} only.
    % FC zero-state selection is handled separately in the main loop.
    if v_ref > C_up,     sw_level = +1;
    elseif v_ref < C_lo, sw_level = -1;
    else,                sw_level =  0;
    end
end

function [THD, I_fund_rms, f_axis, Ia_mag] = compute_thd(signal, fe, dt, label)
    Nwin = 2^nextpow2(round(20/fe/dt));
    Nwin = min(Nwin, numel(signal));
    seg  = signal(end-Nwin+1:end);
    N    = numel(seg);
    f_axis = (0:N-1)/(N*dt);
    Ifft   = abs(fft(seg))/(N/2);
    Ifft(1)= Ifft(1)/2;
    [~,k1] = min(abs(f_axis(1:N/2)-fe));
    I_fund_rms = Ifft(k1)/sqrt(2);
    Ih2=0;
    for h=2:50
        [~,kh]=min(abs(f_axis(1:N/2)-h*fe));
        Ih2=Ih2+Ifft(kh)^2;
    end
    THD=100*sqrt(Ih2)/max(Ifft(k1),1e-9);
    Ia_mag=Ifft(1:N/2);
    fprintf('  %-22s  fe=%6.1fHz  I_fund=%6.2fA rms  THD=%6.2f%%\n',...
        label,fe,I_fund_rms,THD);
end

function [iw,tw,few] = sswin(idx_seg, Ncyc, dc_fe_in, dc_t_in, dt_in, Ntot)
    if isempty(idx_seg), iw=[]; tw=[]; few=NaN; return; end
    few   = dc_fe_in(idx_seg(1));
    nskip = round(numel(idx_seg)*0.5);
    nshow = min(round(Ncyc/few/dt_in), numel(idx_seg)-nskip);
    i0    = idx_seg(1)+nskip;
    i1    = min(i0+nshow-1, Ntot);
    iw    = i0:i1;
    tw    = (dc_t_in(iw)-dc_t_in(iw(1)))*1e3;
end
