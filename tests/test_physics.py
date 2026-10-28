import numpy as np
import pytest
from physics import Motor, TestCase, G, simulate, battery_bus, from_dict, to_dict


def ideal_motor():
    return Motor(radius_m=.025, gear_ratio=1.,torque_constant_nm_a=.1,
        motor_inertia_kgm2=.002,drum_inertia_kgm2=.001,
        coulomb_kgf=0.,viscous_ns_m=0., max_iq_a=500.,
        winding_resistance_ohm=0.,back_emf_vs_rad=0.,
        powertrain_efficiency=1.,battery_ocv_v=1000.,
        battery_resistance_ohm=0.)


def scenario(pull=7,brake=4,rope_mass=.2):
    return TestCase(pull=ideal_motor(),brake=ideal_motor(),
        pull_set_kgf=pull,brake_set_kgf=brake,
        pull_compensation_kgf=0,brake_compensation_kgf=0,
        rope_mass_kg=rope_mass,duration_s=.2,coast_s=0.,
        command_ramp_s=0.,dt_s=.001)


def test_rigid_tension_balance():
    c=scenario()
    d=simulate(c)
    row=d.iloc[30]
    mp=c.pull.equivalent_mass_kg;mb=c.brake.equivalent_mass_kg
    acceleration=3*G/(mp+mb+c.rope_mass_kg)
    assert row.acceleration_m_s2==pytest.approx(acceleration,abs=1e-7)
    assert row.tension_at_0005_kgf==pytest.approx(7-mp*acceleration/G)
    assert row.tension_at_0003_kgf==pytest.approx(4+mb*acceleration/G)
    assert (row.tension_at_0005_kgf-row.tension_at_0003_kgf)*G==pytest.approx(
        c.rope_mass_kg*acceleration)
    assert row.lc1_pred_kgf+row.lc2_pred_kgf==pytest.approx(row.lc_sum_pred_kgf)


def test_equal_force_differences_predict_equal_speeds_but_not_tension():
    a=simulate(scenario(7,4,0))
    b=simulate(scenario(8,5,0))
    assert np.allclose(a.speed_m_s,b.speed_m_s)
    assert np.allclose(b.lc_sum_pred_kgf.iloc[:-1]-a.lc_sum_pred_kgf.iloc[:-1],1.)
    assert a.lc_sum_pred_kgf.max()!=pytest.approx(3.)


def test_mounting_moment_changes_individual_sensors_not_sum():
    c=scenario()
    c.rope_vertical_offset_m=.08
    d=simulate(c); row=d.iloc[30]
    assert row.lc1_pred_kgf!=pytest.approx(row.lc2_pred_kgf)
    assert row.lc_sum_pred_kgf==pytest.approx(row.tension_at_0005_kgf)


def test_battery_signs():
    v,i,over=battery_bus(48,.1,200)
    assert v<48 and i>0 and not over
    v,i,over=battery_bus(48,.1,-200)
    assert v>48 and i<0 and not over
    _,_,over=battery_bus(48,1.,1000)
    assert over


def test_settings_roundtrip_and_input_guardrails():
    c=TestCase()
    assert from_dict(to_dict(c)).pull.radius_m==c.pull.radius_m
    with pytest.raises(ValueError): from_dict({"brake":{"radius_m":0}})
    with pytest.raises(ValueError): simulate(TestCase(dt_s=.1))
