# Alex Run rope force model — engineering specification

**Hardware:** Alex Run 0005 is the pulling/assistive actuator with two horizontal load-cell reactions on its mount; Alex Run 0003 is the braking/resistive actuator. One rope connects their drums. Positive rope displacement is toward 0005.

**Status:** an explanatory, parameterized **one-degree-of-freedom rigid-rope model**. It is not calibrated to the production hardware. Source measurements and setpoint-to-motor-force transformations have not been confirmed against the BE/FE firmware source. Do not use this model to certify mechanical safety limits.

## 1. Notation and units

| Symbol | Meaning | Unit |
| --- | --- | --- |
| \(v, a, x\) | Rope speed, acceleration, pulled travel | m/s, m/s², m |
| \(T_P, T_B\) | Rope tension at pulling 0005 and braking 0003 drum | N |
| \(F_P, F_B\) | Tangential **motor electromagnetic** drive force and counterforce | N |
| \(F_{c,i}, B_i\) | Coulomb drag, viscous rope-velocity drag coefficient | N, N·s/m |
| \(J_{m,i}, J_{d,i}\) | Motor rotor and drum inertia | kg·m² |
| \(r_i\) | Effective rope-centreline drum radius | m |
| \(G_i\) | Gear ratio, motor shaft revolutions per drum revolution | 1 |
| \(I_{q,i}, K_{t,i}\) | Motor q-axis current, torque constant **for that current convention** | A, Nm/A |
| \(\eta_i\) | Mechanical torque transmission efficiency (simplified) | 1 |
| \(m_r\) | Rope effective moving mass between the drums | kg |
| \(g\) | Standard gravitational acceleration: 9.80665 | m/s² |
| \(LC_1,LC_2\) | Load-cell readings converted to kgf | kgf |

A displayed *kg* in the logger is treated as a **kgf force equivalent**, not a moving mass. \(1~\mathrm{kgf}=9.80665~\mathrm{N}\). Never subtract kgf from inertia (kg) without acceleration.

## 2. Drum/motor conversion

For each motor coupled via a fixed transmission:

\[
\omega_m = G\frac{v}{r},\quad
J^*=G^2 J_m+J_d,\quad
M_{eq}=\frac{J^*}{r^2}.
\]

The conversion from motor current to tangential electromagnetic effort, in the pulling direction or as positive brake resistance magnitude, is approximately

\[
F_{motor}=\frac{\eta G K_t I_q}{r}.
\]

**Check actual ODrive definitions.** Motor Kt depends on peak-vs-RMS and q-axis-current convention. Likewise, gearbox *generating* efficiency differs from motoring efficiency. Measured \(I_q\) is not DC battery current. Both user-supplied current conversion and voltage limitation are therefore adjustable.

A force *setting* is not a load-cell measurement. The sample log has approximately:

- 7/4 UI → 8/3 current-derived motor-force commands;
- 8/5 UI → 9/4;
- 9/6 UI → 10/5;
- 6/2 UI → 7/1.

These logs are consistent with **+1 kgf pull command compensation and −1 kgf braking command compensation**, but that mapping is a **hypothesis**, not yet verified against the control source. The model keeps commanded force, motor-produced force, and mechanical friction as separate variables so that compensation is not double counted.

## 3. Mechanical equations

For positive rope velocity, approximate each device's passive mechanical drag as:

\[
f_i(v)=F_{c,i}+B_i v.
\]

At a stationary rope there is static stiction with a *range* of possible reaction forces. Its value is not uniquely identifiable from a drive setpoint alone.

For the pulling drum 0005, from Newton's second law applied to its rotating parts:

\[
\boxed{F_P-T_P-f_P(v)=\frac{J_P^*}{r_P^2}a}.
\]

For the braking drum 0003:

\[
\boxed{T_B-F_B-f_B(v)=\frac{J_B^*}{r_B^2}a}.
\]

For rope mass, distributed across the run:

\[
T_P-T_B=m_r a.
\]

Add the three to obtain the single rope acceleration:

\[
\boxed{a=
\frac{F_P-F_B-f_P(v)-f_B(v)}
{J_P^*/r_P^2+J_B^*/r_B^2+m_r}}.
\]

Then compute the expected pull-side load-cell tension:

\[
\boxed{T_P=F_P-f_P(v)-(J_P^*/r_P^2)a}.
\]

And independently from the braking side:

\[
\boxed{T_B=F_B+f_B(v)+(J_B^*/r_B^2)a}.
\]

The two ends need not have exactly the same tension when \(m_r a\neq 0\). Negligible rope mass gives \(T_P\approx T_B\).

**Key insight:** The UI difference 6−2=4 kgf is *net applied force before losses and inertia*, not a direct prediction for either rope tension.

### When is the load-cell sum 4 kgf?

If rope speed is constant (\(a=0\)), sensors carry all horizontal reaction, and the real brake resistance including passive drag equals 4 kgf, then the load cells on 0005 should sum to roughly 4 kgf. A static deadweight test reading 4±0.2 kgf checks calibration at *that* point only; it does not measure dynamic losses.

## 4. Two load cells: mounting reaction and bending moment

Under ideal mounting (no bypass path, no frame contact and no significant sensor misalignment):

\[
LC_1+LC_2\approx\frac{T_P}{g}.
\]

For vertically separated horizontal sensors, distance \(h\) between sensor axes, total applied moment \(M_z\) relative to the mount midpoint:

\[
LC_1=\frac{T_P}{2g}+\frac{M_z}{gh},\quad
LC_2=\frac{T_P}{2g}-\frac{M_z}{gh}.
\]

Then \(LC_1-LC_2=2M_z/(gh)\) even when the total remains constant. Moment includes off-centre rope load and motor reaction couples, whose sign and force transfer depend on the **actual** mechanical assembly. The app's moment coefficient is deliberately optional and must not be fitted without measured geometry.

Do not use the individual load cells as independent rope tensions. Do not subtract the brake setpoint from their sum. Check a second static deadweight at a different loading height: the individual load cells should redistribute while the sum stays near the reference load, provided they carry all load.

Measurement gain/tare model used in the app:

\[
(LC_1+LC_2)_{reported}=g_s\frac{T_P}{g}+b_s.
\]

The October 27 static test tolerance ~±0.2 kgf at 4 kgf equals ~±5% at that operating point. It is not enough to explain several kgf of dynamic discrepancy and does not establish linearity, hysteresis, or dynamic bandwidth.

## 5. Electrical limitation and battery model

The initial numerical model limits motoring \(I_q\) by:

\[
\omega_m=Gv/r,\quad
I_{q,max}(v)\approx\min\left(I_{limit},
\frac{[mV_{bus}-K_e|\omega_m|]_+}{R}\right).
\]

Here \(m\) is a user-specified usable bus modulation fraction, \(K_e\) a configurable voltage-per-rad/s estimate and \(R\) the phase resistance. **This is a steady-state, simplified voltage budget**, not exact ODrive field-oriented control. It excludes inductive cross-coupling, d-axis weakening, PWM dynamics, inverter thermal throttling, firmware velocity limits and torque-command slew behavior.

The DC battery is modeled approximately as \(V_{bus}=V_{oc}-I_{bus}R_{pack}\) with \(P\approx V_{bus} I_{bus}\). Thus

\[
V_{bus}=\frac{V_{oc}+\sqrt{V_{oc}^2-4R_{pack}P}}{2}.
\]

Negative \(P\) during braking gives negative \(I_{bus}\) and bus voltage rise. A negative discriminant indicates the assumed algebraic pack cannot supply the requested power: the app **flags this as invalid**, instead of treating the clamped square root as an accurate electrical prediction.

The braking current cap is modeled separately; detailed braking voltage regulation, battery acceptance limits, regen resistor and DC-bus transients must be added only with actual ODrive configuration.

## 6. Why the four tests are diagnostically useful

From the plotted motor-force estimates, 7/4, 8/5 and 9/6 all have roughly \(F_P-F_B\approx5\) kgf. In a perfect constant-torque model with identical loss coefficients, same drum radii and same inertia, **their speed traces should match**. Their observed maximum speeds decrease from ~6.57 to ~6.33 and ~5.89 m/s. The explanation cannot be found by subtracting the UI commands alone. Investigate the actual torque/current envelopes, speed-dependent drag, varying spool radius, physical force-vs-command conversion, and controls.

The 6/2 test has a larger force difference (~6 kgf), making its higher speed physically plausible. Its current and bus sag make it a useful stress case.

Do not estimate Coulomb friction and inertia simultaneously from only one acceleration profile: their effects can be confounded. Multiple constant-speed and deliberately varied-acceleration tests are needed for unique identification.

## 7. Measurement and parameter identification plan

1. **Static sensor check:** apply calibrated forces 0, 2, 4, 6, 8 kgf at the rope height, in both directions. Measure LC1, LC2, their sum, hysteresis, repeatability and mounting-bypass errors. Repeat at two heights. Do not use force commands as deadweight references.
2. **No-command passive friction:** with driver output disabled in a safely supported way and motor/brake modes understood, move each unpowered drum through multiple slow velocities. Note that gear cogging, back-driving a powered motor and electrical damping can alter drag.
3. **Constant-speed tests:** command speed control on the pulling side and set several brake torques. Under \(a\approx0\), \(T_B\approx F_B+F_{c,B}+B_Bv\). Collect real \(I_q\), tension and velocity.
4. **Controlled acceleration tests:** use at least three acceleration profiles and several force levels. Regress \(T_B-F_B-f_B(v)\) against \(a\) to infer \(J_B^*/r_B^2\). Confirm similarly from pulling end.
5. **Cross-validation:** fit to a subset of runs, freeze parameters and predict *all* remaining force/speed traces. Report 95% prediction intervals if enough independent repeats exist.
6. **Timestamp alignment:** if logger 0003/0005 share no true synchronized clock, align from a common hardware trigger or independent encoder correlation. Do not compare timestamp-mismatched instantaneous rows as a force balance.
7. **Frequency-domain validation:** sample actual load-cell *new readings* fast enough to observe rope force oscillations. Nominal acquisition-loop rate alone is not sensor bandwidth.
8. **Electrical safety:** log pack and driver voltage, Iq vs battery current, regen, temperature and clipping flags, especially the 6/2 high-speed case.

### Suggested parameter-estimation equations

Let \(y_P=F_P-T_P\), \(y_B=T_B-F_B\); for positive rope motion:

\[
y_P=F_{c,P}+B_Pv+M_Pa,\quad
y_B=F_{c,B}+B_Bv+M_Ba.
\]

With independent calibrated current-derived \(F_i\), synchronized load-cell tension and filtered differentiated speed, fit each via bounded least squares with nonnegative \(F_c,B,M\). Propagate uncertainty, examine residual autocorrelation and identifiability (condition number), and **never** fit a parameter from the same test then claim independent validation.

### Uncertainty

At minimum, propagate errors from \(K_t\), current zero/gain, \(r\), \(J\), LC gain/tare, clock alignment and differentiation. First-order uncertainty for \(F=\eta GK_t I_q/r\) can be approximated as:

\[
\left(\frac{\sigma_F}{F}\right)^2\approx
\left(\frac{\sigma_\eta}{\eta}\right)^2+
\left(\frac{\sigma_G}{G}\right)^2+
\left(\frac{\sigma_{Kt}}{K_t}\right)^2+
\left(\frac{\sigma_I}{I_q}\right)^2+
\left(\frac{\sigma_r}{r}\right)^2
\]

for independent small uncertainties. The radius, Kt and Iq factors are often correlated; use covariance or Monte Carlo otherwise.

## 8. Implemented scope and open work

**Implemented:** rigid-rope integration; compensated commands; motor/drum inertia and gear ratios; Coulomb/viscous drag; approximate motoring current/voltage limits; separate bus sag and regen power; pull- and brake-side tension; mount LC force/moment split; JSON parameter profiles; interactive plotting; custom CSV/XLSX overlays; four-test comparison; invariant tests.

**Not yet implemented:** multi-layer drum variable radius with a validated winding model; actual ODrive torque/regen limits and controllers; separate finite rope stiffness/damping and slack; mechanical backlash; BMS protection; logged-current phase conventions; time-alignment automation between arbitrary raw device clocks; full uncertainty distributions or calibrated identification. These are extensions, not secretly assumed features.

## 9. Safe validation and example use

Run `python -m pip install -r requirements.txt` and `streamlit run app.py`. Change radius, Kt, gear ratio, inertia, passive friction, drag, current, pack impedance and force compensation to **measured** values. Compare predicted speed and tension **simultaneously** across the four independent UI test conditions. Save JSON profiles for traceability and export simulated CSV.

Never fit friction compensation by subtracting an arbitrary constant from both load cells to force a match to the UI difference.
