"""SI-unit, rigid inextensible rope model for Alex Run 0005 pulling / 0003 braking.
Commands are kgf-equivalent motor forces, not measured rope tension.
The default parameters are illustrative; calibrate before interpreting results.
"""
from dataclasses import dataclass, field, asdict
import math
import numpy as np
import pandas as pd

G = 9.80665


@dataclass
class Motor:
    radius_m: float = 0.025
    gear_ratio: float = 1.0
    torque_constant_nm_a: float = 0.055
    motor_inertia_kgm2: float = 0.0001
    drum_inertia_kgm2: float = 0.0002
    coulomb_kgf: float = 1.0
    viscous_ns_m: float = 0.2
    max_iq_a: float = 60.0
    winding_resistance_ohm: float = 0.0447
    back_emf_vs_rad: float = 0.055
    modulation_factor: float = 0.55
    powertrain_efficiency: float = 0.95
    battery_ocv_v: float = 48.1
    battery_resistance_ohm: float = 0.12
    regen_efficiency: float = 0.85

    def validate(self):
        for key in ("radius_m","gear_ratio","torque_constant_nm_a","max_iq_a",
                    "modulation_factor","powertrain_efficiency","battery_ocv_v"):
            if getattr(self,key) <= 0: raise ValueError(f"{key} must be positive")
        for key in ("motor_inertia_kgm2","drum_inertia_kgm2","coulomb_kgf",
                    "viscous_ns_m","winding_resistance_ohm","back_emf_vs_rad",
                    "battery_resistance_ohm"):
            if getattr(self,key) < 0: raise ValueError(f"{key} must be nonnegative")
        if self.modulation_factor > 1 or self.powertrain_efficiency > 1:
            raise ValueError("efficiency and modulation factor must be <= 1")

    @property
    def equivalent_mass_kg(self):
        return (self.motor_inertia_kgm2*self.gear_ratio**2 +
                self.drum_inertia_kgm2)/self.radius_m**2

    def iq_for_force(self,force_n):
        return force_n*self.radius_m/(self.powertrain_efficiency *
                                      self.gear_ratio*self.torque_constant_nm_a)

    def force_from_iq(self,iq_a):
        return iq_a*self.torque_constant_nm_a*self.gear_ratio*self.powertrain_efficiency/self.radius_m

    def iq_limit_at_speed(self,v,bus_v):
        omega=abs(v)*self.gear_ratio/self.radius_m
        available=max(0.,self.modulation_factor*bus_v-self.back_emf_vs_rad*omega)
        limit=(available/self.winding_resistance_ohm
               if self.winding_resistance_ohm else (float("inf") if available else 0.))
        return max(0.,min(self.max_iq_a,limit))

    def friction(self,v):
        return (self.coulomb_kgf*G*(1 if v>=0 else -1) +
                self.viscous_ns_m*v)


@dataclass
class TestCase:
    pull: Motor = field(default_factory=Motor)
    brake: Motor = field(default_factory=Motor)
    pull_set_kgf: float = 7.
    brake_set_kgf: float = 4.
    pull_compensation_kgf: float = 1.
    brake_compensation_kgf: float = -1.
    rope_mass_kg: float = .02
    duration_s: float = 3.
    coast_s: float = 0.
    dt_s: float = .005
    command_ramp_s: float = .08
    initial_speed_m_s: float = 0.
    lc_separation_m: float = .24
    rope_vertical_offset_m: float = 0.
    housing_torque_coupling: float = 0.
    sensor_offset_kgf: float = 0.
    sensor_gain: float = 1.
    static_calibration_error_kgf: float = .2

    def validate(self):
        self.pull.validate(); self.brake.validate()
        for key in ("duration_s","dt_s","lc_separation_m","sensor_gain"):
            if getattr(self,key)<=0: raise ValueError(f"{key} must be positive")
        for key in ("rope_mass_kg","coast_s","command_ramp_s","static_calibration_error_kgf",
                    "pull_set_kgf","brake_set_kgf"):
            if getattr(self,key)<0: raise ValueError(f"{key} must be nonnegative")
        if self.dt_s>.05: raise ValueError("dt_s must be <= 0.05")


def to_dict(c): return asdict(c)


def from_dict(d):
    d=dict(d); d["pull"]=Motor(**d.get("pull",{}))
    d["brake"]=Motor(**d.get("brake",{}))
    case=TestCase(**d); case.validate(); return case


def battery_bus(ocv,r,power_w):
    """Algebraic V=Voc-I*R, P=V*I. Negative power is regeneration."""
    if r<1e-12: return ocv,power_w/ocv,False
    disc=ocv*ocv-4*r*power_w
    overload=disc<0
    v=(ocv+math.sqrt(max(0.,disc)))/2
    return v,power_w/max(v,1e-9),overload


def simulate(c:TestCase)->pd.DataFrame:
    c.validate(); p,b=c.pull,c.brake
    mp,mb=p.equivalent_mass_kg,b.equivalent_mass_kg
    mtotal=mp+mb+c.rope_mass_kg
    if mtotal<1e-6: raise ValueError("Nonzero inertia/mass is required")
    n=int(math.ceil((c.duration_s+c.coast_s)/c.dt_s))+1
    if n>100000: raise ValueError("Too many steps")
    v=max(0.,c.initial_speed_m_s); x=0.
    cp=cb=0.; vp=p.battery_ocv_v; vb=b.battery_ocv_v; rows=[]
    for k in range(n):
        t=k*c.dt_s; active=t<c.duration_s
        fp_goal=max(0.,c.pull_set_kgf+c.pull_compensation_kgf)*G if active else 0.
        fb_goal=max(0.,c.brake_set_kgf+c.brake_compensation_kgf)*G if active else 0.
        ramp=(min(1.,c.dt_s/c.command_ramp_s) if c.command_ramp_s else 1.)
        cp+=(fp_goal-cp)*ramp; cb+=(fb_goal-cb)*ramp
        ip=min(p.iq_for_force(cp),p.iq_limit_at_speed(v,vp))
        ib=min(b.iq_for_force(cb),b.max_iq_a)
        fp=p.force_from_iq(ip); fb=b.force_from_iq(ib)
        if v<1e-4 and fp-fb <= (p.coulomb_kgf+b.coulomb_kgf)*G:
            v=0.; a=0.
            tp=tb=max(0.,min(fp,fb+(p.coulomb_kgf+b.coulomb_kgf)*G))
        else:
            f_p=p.friction(v); f_b=b.friction(v)
            a=(fp-fb-f_p-f_b)/mtotal
            if v<=0 and a<0: a=0.
            tp=max(0.,fp-f_p-mp*a)
            tb=max(0.,fb+f_b+mb*a)
        ep=max(0.,fp*v/max(p.powertrain_efficiency,.01)
                +1.5*ip*ip*p.winding_resistance_ohm)
        eb=-fb*v*b.regen_efficiency+1.5*ib*ib*b.winding_resistance_ohm
        vp,ibp,op=battery_bus(p.battery_ocv_v,p.battery_resistance_ohm,ep)
        vb,ibb,ob=battery_bus(b.battery_ocv_v,b.battery_resistance_ohm,eb)
        mz=tp*c.rope_vertical_offset_m+c.housing_torque_coupling*ip*p.torque_constant_nm_a*p.gear_ratio
        summed=c.sensor_gain*tp/G+c.sensor_offset_kgf
        delt=mz/(G*c.lc_separation_m)
        rows.append(dict(time_s=t,speed_m_s=v,acceleration_m_s2=a,travel_m=x,
            command_pull_kgf=cp/G,command_brake_kgf=cb/G,
            motor_pull_kgf=fp/G,motor_brake_kgf=fb/G,
            tension_at_0005_kgf=tp/G,tension_at_0003_kgf=tb/G,
            lc1_pred_kgf=summed/2+delt,lc2_pred_kgf=summed/2-delt,
            lc_sum_pred_kgf=summed,iq_pull_a=ip,iq_brake_a=ib,
            vbus_pull_v=vp,vbus_brake_v=vb,ibus_pull_a=ibp,ibus_brake_a=ibb,
            pull_voltage_limited=ip+1e-6<p.iq_for_force(cp),
            pull_bus_overload=op,brake_bus_overload=ob))
        v=max(0.,v+a*c.dt_s)
        x+=v*c.dt_s
    return pd.DataFrame(rows)


def quasi_static_tension_kgf(c):
    """Valid ONLY when speed is held constant by an external controller."""
    return max(0.,c.brake_set_kgf+c.brake_compensation_kgf)+c.brake.coulomb_kgf
