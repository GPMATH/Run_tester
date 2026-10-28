# Alex Run tester — Physics-based rope validation

A configurable **motor → drum → rope → load-cell** model for the two-device Alex Run test bench.

**Device assignments:** 0005 is pulling/assistive and carries two load cells on its mounting structure; 0003 is the resistive braking device. The model distinguishes motor electromagnetic effort, actual rope tension at each end, reaction force at the load-cell mount, motor Iq, DC battery current and battery voltage.

## Run locally

Requires Python 3.11 or newer.

```bash
git clone https://github.com/GPMATH/Run_tester.git
cd Run_tester
git switch feature/rope-physics-validation
python -m venv .venv

# Windows
.venv\Scripts\activate

# macOS/Linux: source .venv/bin/activate
python -m pip install -r requirements.txt
streamlit run app.py
```

The browser dashboard lets you enter **separate motor data for 0005 and 0003**: drum radius, gearbox ratio, Kt, rotor/drum inertia, Coulomb and viscous drag, q-axis current limits, voltage/back-EMF approximations, transmission efficiency, battery and wiring impedance, friction compensation, rope mass, duration, and two-load-cell mounting geometry. Enter **actual measured hardware parameters**; the defaults are illustrative and must not be treated as identified from the eight runs.

Tabs show:
- Predicted motor forces, measured reaction-equivalent load-cell curves, and speeds, analogous to the October 27 eight-panel graph;
- Current/voltage limitations, battery sag and braking regeneration;
- CSV/XLSX upload with manual column mapping and a time-shift setting for comparison with measured speed/force;
- Mechanical equations and the assumptions under which they are valid.

At the bottom, the **four-condition cross-validation** compares predicted speed and LC force against editable baseline observations for 6/2, 7/4, 8/5 and 9/6 with **one common physical parameter set**, plus ±10% parameter sensitivity.

Run mathematical consistency tests:

```bash
python -m pytest -q tests
```

## The key physical distinction

**6 kgf pull − 2 kgf brake ≠ automatically 4 kgf measured rope tension.**

For positive rope motion:

```text
0005 PULL                               0003 BRAKE
motor Fp --> [rotor/drum] --- ROPE ---> [rotor/drum] <-- motor Fb
             | two load cells on mounting structure |
```

For fixed effective drum radii:

```text
Mp = (J_drum,p + Gp² * J_motor,p)/rp²
Mb = (J_drum,b + Gb² * J_motor,b)/rb²

a   = (Fp - Fb - friction_p(v) - friction_b(v)) / (Mp + Mb + m_rope)
Tp  = Fp - friction_p(v) - Mp*a       [tension at 0005]
Tb  = Fb + friction_b(v) + Mb*a       [tension at 0003]
Tp - Tb = m_rope*a
LC1 + LC2 ≈ Tp / 9.80665             [kgf, if all reaction is carried]
```

For 0005 sensors separated vertically, individual LC values can change in opposite directions because mounting moments redistribute reaction load, while their sum stays approximately constant.

The **static 4 kgf ±0.2 kgf** check is useful but does not establish dynamic rope-tension accuracy or prove that force-command differences should equal the sensor sum.

## Diagnostic insight from the October 27 data

The compensated current-derived motor efforts for 7/4, 8/5, and 9/6 are approximately 8/3, 9/4, 10/5 kgf respectively: the difference is about 5 kgf in each case. A perfect constant-torque, identical-loss model predicts very similar acceleration/speed histories. Actual measured peak speeds (~6.57, 6.33, 5.89 m/s) differ. Model current/voltage limits, real torque trajectories, friction as a function of speed and possibly changing drum radius must be tested against the independent data.

The 6/2 test reaches ~10.44 m/s and is especially important for pack voltage sag and regen safety review.

## Design specification

Read [docs/MODEL.md](docs/MODEL.md) for the detailed mathematical derivation, motor-current conversion, sensor force/moment model, battery model, limitations, uncertainty propagation and parameter-identification test plan.

## Known limitations (important)

This version models **rigid non-slipping rope and a fixed effective drum radius**. It does **not** simulate a real ODrive firmware control loop, spool radius changing across layers, rope compliance/slack, brake resistor/BMS behavior, active stopping controller, or structural modal vibration. The two load-cell *split* is illustrative until mounting geometry and torque paths have been measured. Braking generator torque has only a current limit, not a full voltage/thermal protection model.

**Do not use predicted forces or pack voltages as safe operating limits without physical validation.**

## Development

Work is isolated in `feature/rope-physics-validation` to avoid impacting production software. Raw measurement logs and Excel workbooks are excluded by `.gitignore`.
