"""Interactive mathematical rope model. Start with: streamlit run app.py"""
import json
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import streamlit as st
from physics import TestCase, Motor, G, to_dict, from_dict, simulate, quasi_static_tension_kgf

st.set_page_config(page_title="Alex Run — Rope physics", layout="wide")
st.title("Alex Run — physics-based rope validation")
st.caption("0005 pulls, 0003 brakes. Two load cells on 0005 measure mounting reaction.")
st.warning("All initial motor/drum/friction numbers are ILLUSTRATIVE, not measured. Do not use the predictions as safety limits or a calibrated force certificate.")

if "conf" not in st.session_state:
    st.session_state.conf=to_dict(TestCase())

with st.sidebar:
    st.header("Motor and bench inputs")
    uploaded_profile=st.file_uploader("Import saved parameters (JSON)",type="json")
    if uploaded_profile and st.button("Load selected profile"):
        try:
            st.session_state.conf=to_dict(from_dict(json.loads(uploaded_profile.getvalue())))
            for key in list(st.session_state.keys()):
                if key.startswith("edit_"): del st.session_state[key]
            st.rerun()
        except Exception as e: st.error(str(e))
    c=st.session_state.conf
    def number(title, key, group=None, scale=1., step=.1, low=None, high=None, fmt="%.3f", hint=None):
        d=c if group is None else c[group]
        args=dict(value=float(d[key])*scale,step=float(step),format=fmt,key="edit_"+(group or "case")+"_"+key,help=hint)
        if low is not None: args["min_value"]=float(low)
        if high is not None: args["max_value"]=float(high)
        actual=st.number_input(title,**args)
        d[key]=actual/scale
    with st.expander("Force commands and run",expanded=True):
        number("Pull setpoint 0005 (kgf)","pull_set_kgf",step=.5,low=0)
        number("Brake setpoint 0003 (kgf)","brake_set_kgf",step=.5,low=0)
        number("Pull friction compensation (+kgf)","pull_compensation_kgf",step=.25)
        number("Brake compensation (signed kgf)","brake_compensation_kgf",step=.25,
               hint="Negative means the motor commands less brake torque because passive friction assists braking.")
        number("Active pulling duration (s)","duration_s",low=.1,step=.5)
        number("Optional post-command coast (s)","coast_s",low=0,step=.1)
        number("Command rise time (s)","command_ramp_s",low=0,step=.02)
        number("Integration step (s)","dt_s",low=.001,high=.05,step=.001,fmt="%.3f")
        number("Initial speed (m/s)","initial_speed_m_s",low=0,step=.1)
    for group,label in [("pull","0005 — pulling motor"),("brake","0003 — braking motor")]:
        with st.expander(label):
            number("Effective drum radius (mm)","radius_m",group,scale=1000,step=1,low=1,fmt="%.1f",
                   hint="Measure from axle centre to rope centreline; model assumes fixed radius.")
            number("Motor revolutions per drum revolution","gear_ratio",group,step=.5,low=.1)
            number("Kt motor torque constant (Nm/A)","torque_constant_nm_a",group,step=.005,low=.001,fmt="%.4f")
            number("Motor rotor inertia (kg·m²)","motor_inertia_kgm2",group,step=.0001,low=0,fmt="%.6f")
            number("Drum inertia (kg·m²)","drum_inertia_kgm2",group,step=.0001,low=0,fmt="%.6f")
            number("Passive Coulomb friction (kgf)","coulomb_kgf",group,step=.1,low=0)
            number("Viscous drag coefficient (N·s/m)","viscous_ns_m",group,step=.5,low=0)
            number("Maximum Iq current (A)","max_iq_a",group,step=1,low=.1)
            number("Drivetrain efficiency (0–1)","powertrain_efficiency",group,step=.05,low=.01,high=1)
            number("Motor winding resistance (ohm)","winding_resistance_ohm",group,step=.005,low=0,fmt="%.4f")
            number("Motor back-EMF coefficient (V·s/rad)","back_emf_vs_rad",group,step=.005,low=0,fmt="%.4f")
            number("Max usable DC voltage fraction","modulation_factor",group,step=.05,low=.01,high=1)
            number("Pack open-circuit voltage (V)","battery_ocv_v",group,step=1,low=1)
            number("Pack and wiring resistance (ohm)","battery_resistance_ohm",group,step=.02,low=0)
            number("Regenerative energy efficiency (0–1)","regen_efficiency",group,step=.05,low=0,high=1)
    with st.expander("Rope and two force sensors"):
        number("Rope effective moving mass (kg)","rope_mass_kg",low=0,step=.02)
        number("Load cell vertical separation (m)","lc_separation_m",low=.01,step=.02)
        number("Rope height relative to LC midpoint (m)","rope_vertical_offset_m",step=.01)
        number("Case-motor-torque moment coefficient","housing_torque_coupling",step=.1,
               hint="Set 0 unless the physical load path and torque couple are known.")
        number("Combined load-cell gain","sensor_gain",low=.01,step=.01)
        number("Combined load-cell tare (kgf)","sensor_offset_kgf",step=.05)
        number("Static calibration uncertainty ± (kgf)","static_calibration_error_kgf",low=0,step=.05)
    try:
        test=from_dict(c)
        st.download_button("Save all parameters (JSON)",json.dumps(to_dict(test),indent=2),
                           file_name="alex_run_parameters.json",mime="application/json",use_container_width=True)
    except Exception as err:
        st.error("Invalid parameters: "+str(err)); st.stop()

try: data=simulate(test)
except Exception as err:
    st.error("Simulation error: "+str(err)); st.stop()

active=data[data.time_s<test.duration_s]
middle=active[(active.time_s>.3*test.duration_s)&(active.time_s<.85*test.duration_s)]
if middle.empty: middle=active
boxes=st.columns(5)
boxes[0].metric("Peak speed",f"{active.speed_m_s.max():.2f} m/s")
boxes[1].metric("LC sum (mid-run median)",f"{middle.lc_sum_pred_kgf.median():.2f} kgf")
boxes[2].metric("Pull motor peak Iq",f"{active.iq_pull_a.max():.1f} A")
boxes[3].metric("Pull bus minimum",f"{active.vbus_pull_v.min():.1f} V")
boxes[4].metric("Equivalent moving mass",f"{test.pull.equivalent_mass_kg+test.brake.equivalent_mass_kg+test.rope_mass_kg:.2f} kg")

if data.pull_voltage_limited.any():
    st.warning("Pulling motor is torque-limited by an approximate back-EMF/voltage/current envelope.")
if data.pull_bus_overload.any() or data.brake_bus_overload.any():
    st.error("Pack power constraint exceeded. DC voltage/current predictions in the affected region are invalid.")

def plot(traces, name, unit):
    fig=go.Figure()
    for column,label,color,dash in traces:
        fig.add_trace(go.Scatter(x=data.time_s,y=data[column],mode="lines",name=label,
                                 line=dict(color=color,width=2,dash=dash)))
    fig.update_layout(template="plotly_white",title=name,hovermode="x unified",height=390,
        xaxis_title="Time (s)",yaxis_title=unit,legend=dict(orientation="h",y=-.24),
        margin=dict(l=20,r=20,b=35,t=55))
    return fig

t1,t2,t3,t4=st.tabs(["Expected force / speed","Electrical limits","Compare actual recording","Equations and assumptions"])
with t1:
    st.plotly_chart(plot([
        ("motor_pull_kgf","0005 motor force","#f97316","solid"),
        ("motor_brake_kgf","0003 motor force","#10b981","solid"),
        ("lc_sum_pred_kgf","Predicted LC sum (0005)","#2563eb","solid"),
        ("command_pull_kgf","0005 compensated setpoint","#f97316","dash"),
        ("command_brake_kgf","0003 compensated setpoint","#10b981","dash")],
        "Predicted forces — analogous to your eight-run plot","kgf"),use_container_width=True)
    st.plotly_chart(plot([
        ("tension_at_0005_kgf","Tension at pull 0005","#2563eb","solid"),
        ("tension_at_0003_kgf","Tension at brake 0003","#8b5cf6","solid"),
        ("lc1_pred_kgf","Load cell 1","#f59e0b","dash"),
        ("lc2_pred_kgf","Load cell 2","#475569","dash")],
        "Tension along rope and force redistribution","kgf"),use_container_width=True)
    st.plotly_chart(plot([
        ("speed_m_s","Rope speed","#262626","solid"),
        ("acceleration_m_s2","Rope acceleration","#d97706","dash")],
        "Simulated rope motion","m/s and m/s²"),use_container_width=True)
    st.caption("The individual load-cell split is hypothetical until sensor positions and motor case torque paths are identified. The load-cell sum is the modeled horizontal reaction.")
with t2:
    st.plotly_chart(plot([
        ("iq_pull_a","0005 motor Iq","#f97316","solid"),
        ("iq_brake_a","0003 motor Iq magnitude","#10b981","solid"),
        ("ibus_pull_a","0005 battery current","#2563eb","dash"),
        ("ibus_brake_a","0003 battery current","#8b5cf6","dash")],
        "Iq current is NOT battery current","A"),use_container_width=True)
    st.plotly_chart(plot([
        ("vbus_pull_v","0005 battery","#f97316","solid"),
        ("vbus_brake_v","0003 battery","#10b981","solid")],
        "DC pack sag and regeneration (simplified)","V"),use_container_width=True)
    st.caption("Battery model is steady-state power balance. It does not model the BMS, brake resistor, bus capacitor or full ODrive FOC.")
with t3:
    st.write("Upload a synchronized CSV/XLSX with time, measured speed, and optionally two load-cell readings or their sum. If 0003 and 0005 logs use different clocks, align them before inferring force balance.")
    file=st.file_uploader("Actual validation log",type=["csv","xlsx"])
    if file:
        try:
            measured=pd.read_excel(file,engine="openpyxl") if file.name.endswith(".xlsx") else pd.read_csv(file)
            measured.columns=[str(x).strip() for x in measured.columns]
            allcols=list(measured.columns)
            def pick(label,keys,optional=True):
                opts=(["(none)"] if optional else [])+allcols
                pref=next((x for x in allcols if any(key in x.lower() for key in keys)),None)
                return st.selectbox(label,opts,index=opts.index(pref) if pref in opts else 0)
            a,b,c,d=st.columns(4)
            with a: tc=pick("Time seconds",["time_s","time","elapsed"],False)
            with b: sc=pick("Speed (m/s)",["speed_m_s","speed","velocity"])
            with c: lc=pick("Load-cell sum (kgf)",["lc_sum","rope_force","rope force"])
            with d: rc=pick("Run number / ID",["run_id","run","test"])
            if rc!="(none)":
                runs=measured[rc].dropna().astype(str).unique().tolist()
                chosen=st.selectbox("Recorded run",runs)
                measured=measured[measured[rc].astype(str)==chosen]
            if lc=="(none)":
                aa,bb=st.columns(2)
                with aa: l=pick("First load cell (kgf)",["lc_left","lc1"])
                with bb: r=pick("Second load cell (kgf)",["lc_right","lc2"])
                if l!="(none)" and r!="(none)":
                    measured["_loadsum"]=pd.to_numeric(measured[l],errors="coerce")+pd.to_numeric(measured[r],errors="coerce")
                    lc="_loadsum"
            measured["_t"]=pd.to_numeric(measured[tc],errors="coerce")
            measured=measured.dropna(subset=["_t"]).sort_values("_t")
            time_shift=st.number_input("Additional time shift (s)",value=0.,step=.05)
            measured["_t"]=measured["_t"]-measured["_t"].iloc[0]+time_shift
            fig=go.Figure()
            if lc!="(none)":
                fig.add_trace(go.Scatter(x=measured._t,y=pd.to_numeric(measured[lc],errors="coerce"),name="Measured LC",line=dict(color="#2563eb")))
                fig.add_trace(go.Scatter(x=data.time_s,y=data.lc_sum_pred_kgf,name="Predicted LC",line=dict(color="#2563eb",dash="dash")))
            if sc!="(none)":
                fig.add_trace(go.Scatter(x=measured._t,y=pd.to_numeric(measured[sc],errors="coerce"),name="Measured speed",line=dict(color="#262626")))
                fig.add_trace(go.Scatter(x=data.time_s,y=data.speed_m_s,name="Predicted speed",line=dict(color="#262626",dash="dash")))
            fig.update_layout(template="plotly_white",height=440,hovermode="x unified",xaxis_title="Aligned relative time (s)",yaxis_title="m/s or kgf")
            st.plotly_chart(fig,use_container_width=True)
            if lc!="(none)":
                sample=measured[["_t",lc]].copy()
                sample[lc]=pd.to_numeric(sample[lc],errors="coerce")
                sample=sample.dropna().groupby("_t",as_index=False).mean()
                use=data[(data.time_s>=sample._t.iloc[0])&(data.time_s<=sample._t.iloc[-1])&(data.time_s<test.duration_s)]
                if len(use):
                    actual=np.interp(use.time_s,sample._t,sample[lc])
                    st.metric("LC residual RMS",f"{np.sqrt(np.mean((use.lc_sum_pred_kgf.to_numpy()-actual)**2)):.3f} kgf")
            st.dataframe(measured.head(200),use_container_width=True)
        except Exception as err: st.error(f"Could not parse recording: {err}")
    st.download_button("Download predicted signals (CSV)",data.to_csv(index=False),
                       file_name="alex_run_expected_signals.csv",mime="text/csv")
with t4:
    st.latex(r"F_P-F_B-f_P(v)-f_B(v)=(J_P^*/r_P^2+J_B^*/r_B^2+m_r)a")
    st.latex(r"T_P=F_P-f_P(v)-(J_P^*/r_P^2)a")
    st.latex(r"T_B=F_B+f_B(v)+(J_B^*/r_B^2)a")
    st.latex(r"J_i^*=G_i^2J_{motor,i}+J_{drum,i}")
    st.latex(r"F_{motor,i}=\eta_iG_iK_{t,i}I_{q,i}/r_i")
    st.latex(r"LC_1+LC_2=g_sT_P/g+b_s")
    st.latex(r"LC_{1,2}=(LC_1+LC_2)/2\pm M_z/(gh)")
    st.write(f"Brake-side quasi-static tension at constant speed: **{quasi_static_tension_kgf(test):.2f} kgf**. This is NOT guaranteed when the torque commands accelerate the rope.")
    st.write(f"Static calibration uncertainty allowance: **±{test.static_calibration_error_kgf:.2f} kgf**.")
    st.markdown("**Limitations:** no changing drum radius, rope slack, elasticity, active speed control, backlash, thermal dynamics, braking voltage regulation or sensor bandwidth effects. Use measured parameters and additional runs to validate.")
