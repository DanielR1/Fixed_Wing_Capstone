# algorithm.md — Guidance & Control Loop Reference

Detailed reference for the fixed-wing tailsitter guidance → attitude-control → allocation →
dynamics pipeline. Referenced from [CLAUDE.md](CLAUDE.md). File/line citations point at the
implementation so this doc and the code can be kept in sync.

---

## 0. Loop overview

One simulation tick (`scripts/main.py`, the `while running:` loop, 500 Hz):

```
        ┌─────────────┐   state    ┌────────────┐  a_com   ┌────────────┐  [T1,T2,δ1,δ2]  ┌────────────┐
 state ─►│ NAVIGATION  ├──(x̂)──────►│  GUIDANCE  ├─────────►│  CONTROL   ├────────────────►│  DYNAMICS  ├─► state'
        │ (estimate)  │            │ (a command)│          │ (attitude  │                 │ (truth     │
        └─────────────┘            └────────────┘          │  + alloc)  │                 │  model)    │
                                                           └────────────┘                 └────────────┘
```

| Stage | File | Output |
|-------|------|--------|
| Navigation | `flight_code/navigation.py` | estimated state x̂ (currently = true state) |
| Guidance | `flight_code/guidance/…` | commanded acceleration `a_com` ∈ ℝ³ (global) |
| Aero feed-forward | `flight_code/aero_comp.py` | `a_com` minus predicted aero acceleration |
| Control | `flight_code/control.py` | desired quaternion → torque → `[T1,T2,δ1,δ2]` |
| Dynamics | `truth_model/dynamics.py`, `aero.py` | propagated state |

Loop rate `SIMULATION_RATE = 500 Hz`, `dt = 0.002 s`. Integration is forward Euler.
The loop stops at `t > CRASH_CHECK_TIME (3.0 s)` or on crash (`z > −MIN_ALTITUDE`, NED ground).

---

## 1. Notation & frames

**Convention: NED world + FRD body.**

- **World frame G (NED):** x = North, y = East, **z = Down**. Gravity `g = 9.8 m/s²` acts along **+z**. Altitude = −z.
- **Body frame B (FRD):** x = forward (**nose / thrust axis**), y = right, z = down.
- **Attitude quaternion** `q = [qw, qx, qy, qz]`, scalar-first, **body→world**.
  `R = quat_to_R(q)` rotates a body vector into world: `v_G = R · v_B`.
  Identity `[1,0,0,0]` = level forward flight heading North; nose-up hover ≈ `[0.7071,0,0.7071,0]` (+90° about body-y).
- **Angular velocity** `ω = [wx, wy, wz]` in body axes (rad/s).
- **Aero angles:** `alpha = atan2(w_b, u_b)`, `beta = atan2(v_b, √(u_b²+w_b²))`, with
  `v_B = Rᵀ v_G` (`helper_funcs.compute_alpha_beta`). **Stored in degrees** (the XFLR5 table
  lookup keys on degrees); converted to radians at every trig site (`aero.py`, `aero_comp.py`).

**State vector** `state` (15 elements; dynamics integrate [0:13], `compute_alpha_beta` fills [13:15]):

```
[ x  y  z | vx vy vz | qw qx qy qz | wx wy wz | alpha beta ]
   0  1  2    3  4  5    6  7  8  9   10 11 12     13    14
  ─ pos(NED) ─ vel(NED) ─ quat B→G ─  ─ ω(B) ─   ─ deg ──
```

**Control vector** `ctrl_in = [T1, T2, delta1, delta2]`: motor thrusts (N, L=1/R=2) and elevon
deflections (rad, trailing-edge-down positive).

---

## 2. Guidance — commanded acceleration

`guidance.Guidance.get_a_com(state, t)` dispatches on `guid_mode`. The active mode is
`"basic_guidance"` (set in `main.py`). Legacy modes (`waypoint`, `polynomial`) are not wired up.

### 2.1 Basic guidance (`basic_guidance.get_a_com_bg`)

An open-loop **acceleration schedule along +x**, with PD tracking of the implied position/velocity:

| Phase | Time (s) | a_des (m/s²) | v_des (m/s) | pos_des (m) |
|-------|----------|--------------|-------------|-------------|
| 1 accel | 0 ≤ t < 1 | [1, 0, 0] | [t, 0, 0] | [0.5 t², 0, 0] |
| 2 cruise | 1 ≤ t < 4 | [0, 0, 0] | [1, 0, 0] | [0.5 + (t−1), 0, 0] |
| 3 decel | 4 ≤ t < 5 | [−1, 0, 0] | [1−(t−4), 0, 0] | [3.5 + (t−4) − 0.5(t−4)², 0, 0] |
| 4 hold | t ≥ 5 | 0 | 0 | [4, 0, 0] |

Feedback (PD on tracking error), with gains `Kp_BG = diag(12,12,12)`, `Kd_BG = diag(4,4,4)`:

```
e_p = state[0:3] − pos_des
e_v = state[3:6] − v_des
a_com = a_des − Kp_BG · e_p − Kd_BG · e_v
```

> **Note:** no gravity term is added here (the legacy commented guidance added `+[0,0,g]`).
> The downstream controller therefore receives an `a_com` that does not explicitly include
> hover thrust to counter gravity. See §8.

### 2.2 Legacy polynomial trajectory (`polynomial_traj.py`)

7th-order minimum-something splines through a "gate" (approach + departure halves), solved from
boundary conditions `A c = b` per axis. **Currently unused** and references undefined symbols
(`np`, `trp`, `x0`, `g`, `tf`, …). Kept for reference / future waypoint guidance.

---

## 3. Aero feed-forward (`aero_comp.aero_comp`)

Before converting `a_com` into attitude, subtract the acceleration the airframe is predicted to
produce on its own (so the actuators only supply the *remainder*). Flat-plate model, longitudinal
only (no side force), parameters `CD_MAX_FLAT = 1.2`, `CD_0_FLAT = 0.05`:

```
V   = ‖v_G‖
αr  = radians(alpha)                   # stored in degrees, converted for trig
CX  = CD0 · cos(αr)                     # axial coeff
CZ  = (CDmax + CD0) · sin(αr)           # normal coeff
Fx_body = −½ ρ V² S · CX
Fz_body = −½ ρ V² S · CZ               # NED/FRD: +α normal force acts up (−z)
a_aero_G = R(q) · [Fx_body, 0, Fz_body]
a_com  ←  a_com − a_aero_G
```

> Note this still subtracts an aero **force** (N) from an acceleration command (m/s²) without
> dividing by mass — a separate pre-existing issue (see §8).

---

## 4. Acceleration → desired attitude quaternion (`control.py`)

First the guidance **kinematic** acceleration command is converted into a **thrust specific-force**
command by compensating known forces: subtract the predicted aero specific force (§3), then remove
gravity (NED gravity is +g along +z, so the thrust must supply −g along z to hold altitude):

```
a_com ← aero_comp(a_com, state)        # §3: subtract predicted aero
a_com ← a_com − [0, 0, g]              # gravity feed-forward, NED (hover ⇒ a_com = [0,0,−g] = up)
```

Gravity comp lives in the controller (not guidance) so guidance stays a pure kinematic command,
reusable across all guidance modes. The controller then builds two candidate desired attitudes
from this `a_com` and blends them by airspeed.

### 4.1 Commanded thrust magnitude

```
T_com_hover = m · ‖a_com‖           # m = MASS = 0.829 kg
a_hat       = a_com / ‖a_com‖
T_com       = T_com_hover            # passed to allocation as total thrust
```

### 4.2 Hover desired quaternion (thrust-vectoring / shortest arc)

Shortest-arc quaternion mapping the body thrust axis `e1 = [1,0,0]` onto `a_hat`
(in code the variable is still named `e3`, but it is set to `[1,0,0]`):

```
q_d_hover_r = 1 + e1·a_hat
q_d_hover_v = e1 × a_hat
q_d_hover   = [q_d_hover_r, q_d_hover_v] / √(2 (1 + e1·a_hat)),  then normalized
```

This makes `R(q_d_hover) · e1 = a_hat`, i.e. it points the **body x-axis (the thrust axis)** along
the commanded thrust direction — consistent with the dynamics/allocation thrust model. In hover
`a_hat = [0,0,−1]` (NED up), giving `q_d_hover ≈ [0.7071, 0, 0.7071, 0]` (nose-up).

### 4.3 Fixed-wing desired quaternion (coordinated bank-to-turn)

Only when `V > 0.1 m/s` (else `q_d_fw = q_d_hover`):

```
psi      = atan2(vy, vx)                          # flight-path heading
lat_dir  = [−sin psi, cos psi, 0]                 # horizontal, ⟂ to ground track
a_lat    = a_com · lat_dir                        # lateral accel demand
phi_d    = atan2(a_lat, g)                         # desired bank angle
theta_d  = quat_to_euler_ZXY(q_d_hover)[1]         # borrow pitch from hover solution
q_d_fw   = euler_ZXY_to_quat(phi_d, theta_d, psi)  # ZXY: yaw(Z)→roll(X)→pitch(Y)
```

### 4.4 Airspeed blend (SLERP)

```
V_min = 2.0,  V_max = 10.0
w   = clip((V − V_min) / (V_max − V_min), 0, 1)
q_d = slerp(q_d_hover, q_d_fw, w)        # w=0 → pure hover, w=1 → pure fixed-wing
```

`slerp` (`quaternion_helpers.slerp`) takes the shorter path (negates `q2` if dot < 0) and falls
back to normalized lerp when the quaternions are within ~1.8° (dot > 0.9995).

---

## 5. Attitude control — sliding-mode-style torque law (`control.py`)

Quaternion error (desired→actual), with `q_d* ` the conjugate of `q_d`:

```
q_e      = q_d* ⊗ q                       # quat_mult(q_d_star, q)
q_e_vec  = q_e[1:4]
q_e_dot  = ½ · q_e ⊗ [0, ω]               # quaternion kinematics of the error
```

Commanded body torque (gains `Kp_ATTITUDE = diag(3.7)`, `Kd_ATTITUDE = diag(0.19)`,
`LAMBDA_ATTITUDE = diag(0.2)`):

```
τ = −sign(q_e,0)·(Kp · q_e_vec) − (Kd · ω) + sign(q_e,0)·(λ · q_e_dot[1:4])
      └ proportional, ───┘        └ rate ┘      └ sliding-surface λ term ┘
        sign(q_e,0) picks the short-rotation hemisphere (avoids unwinding)
```

`τ = [τx (roll), τy (pitch), τz (yaw)]`. An integral term is scaffolded but commented out.

---

## 6. Control allocation — torque + thrust → `[T1, T2, δ1, δ2]`

Two stages. Parameters: `l_y = THRUST_MOMENT_ARM_Y_m = 0.14 m`, `cx = MOMENT_COEFF_X = 0.144`,
`cy = MOMENT_COEFF_Y = 0.0616`. Indices: 1 = left, 2 = right.

### 6.1 Stage A1 — yaw moment + total thrust → motor thrusts

Yaw comes from differential thrust; total thrust is the sum:

```
[ τz   ]   [ l_y   −l_y ] [ T1 ]
[ T_com] = [  1      1  ] [ T2 ]      ⇒   [T1, T2] = A1⁻¹ [τz, T_com]
```

Clamp each motor: `T ← clip(T, 0.1, MAX_THRUST_ONE_MOTOR_N = 6.62 N)`.

### 6.2 Stage A2 — roll & pitch moments → elevon deflections

Elevon effectiveness is proportional to the **thrust through that elevon** (slipstream) times its
deflection. Using the *clamped* `T1, T2` from A1:

```
[ τx ]   [  cx·T1   −cx·T2 ] [ δ1 ]
[ τy ] = [ −cy·T1   −cy·T2 ] [ δ2 ]      ⇒   [δ1, δ2] = A2⁻¹ [τx, τy]
```

- Row 1 (roll τx): elevons act **differentially** (+cx·T1, −cx·T2).
- Row 2 (pitch τy): elevons act **together** (−cy·T1, −cy·T2).

Clamp each deflection to `[MIN_DEFLECTION_TED_RAD, MAX_DEFLECTION_TED_RAD] = ±20°`.

> A2 depends on `T1, T2`; if a motor is near the 0.1 N floor, `A2` becomes ill-conditioned
> (small slipstream → large/clipped deflection demand). The matrix is singular if `T1·T2 = 0`.

Output: `ctrl_in = [T1, T2, delta1, delta2]`.

---

## 7. Truth-model dynamics (`truth_model/`)

The simulated plant. **Not** part of the deployed flight code.

### 7.1 Equations of motion (`dynamics.rates`)

```
R          = quat_to_R(q)
FM_aero    = aero.getAeroForcesMoments(state, ctrl_in)         # [Fx,Fy,Fz, 0,My,0] body
FM_control = get_control_forces_moments_body(ctrl_in)          # [T1+T2,0,0, τx,τy,τz] body
FM_Total   = FM_aero + FM_control                              # aero + thrust/control, body frame

# Linear:  v̇_G = R · F_body / m ,  then  v̇z += g   (NED: gravity along +z)
# Quaternion kinematics:  q̇ = ½ Ω(ω) · q
# Rotational (Euler):     ω̇ = J⁻¹ ( τ − ω × Jω )
```

State derivative `[ẋ, ẏ, ż, v̇x, v̇y, v̇z, q̇w, q̇x, q̇y, q̇z, ω̇x, ω̇y, ω̇z]`.

### 7.2 Control forces & moments (`get_control_forces_moments_body`)

Mirror of the allocation model — this is the "plant truth" the allocator is inverting:

```
τx (roll)  =  cx·T1·δ1 − cx·T2·δ2
τy (pitch) = −cy·T1·δ1 − cy·T2·δ2
τz (yaw)   =  l_y·T1 − l_y·T2
Fx         =  T1 + T2            (along body +x; Fy = Fz = 0)
```

### 7.3 Aerodynamics (`aero.getAeroForcesMoments`)

Lift/drag/pitching-moment from a blended model vs `alpha` (degrees):

- **−5° ≤ α ≤ 18°:** interpolate `CL, CD, Cm` from `AERO_XFLR5.tsv` (`np.interp`).
- **−10°…−5° and 18°…24.6°:** linear blend between XFLR5 and flat-plate.
- **Otherwise (high |α|):** flat-plate: `CL = ½ CDmax sin2α`, `CD = CDmax sin²α + CD0`, `Cm = 0`.

```
L  = ½ ρ V² S · CL ,  D = ½ ρ V² S · CD ,  M = ½ ρ V² S · c_mac · Cm
Fx = −D cos α + L sin α      # body axial (FRD)
Fz = −D sin α − L cos α      # body normal; lift acts toward −z (up)
return [Fx, 0, Fz, 0, M, 0]   # verify Cm sign = nose-up positive about +y
```

`AERO_XFLR5.tsv` columns: `alpha, Beta, CL, CDi, CDv, CD, CY, Cl, Cm, Cn, Cni, QInf, XCP`
(the code reads columns 0=alpha, 2=CL, 5=CD, 8=Cm).

### 7.4 Integration (`dynamics.propagate`)

Forward **Euler**: `state[0:13] += dt · rates(...)`. Then renormalize the quaternion and recompute
`alpha, beta`. Gaussian process-noise hooks exist but are commented out. `# Do RK4 later`.

---

## 8. Open questions & known discrepancies

**Resolved** (kept here for history): frame convention (**NED world + FRD body** throughout —
gravity +z §7.1, lift toward −z §7.3, gravity feed-forward −g §4, hover start attitude); thrust-axis
convention (hover quaternion maps body-x → `a_hat`, §4.2); force sign in `rates`
(`FM_aero + FM_control`, §7.1); alpha/beta units (degrees for table lookup, radians for all trig);
and `plotter.py` (correct 20-column map, altitude plotted as −z, broken animation removed).
The items below are still open — **confirm intent before changing; several are convention/design
choices, not obvious bugs.**

1. **Guidance z-setpoint = 0.** `basic_guidance` only shapes the x (North) axis; its implied
   position setpoint is `[…, 0, 0]`, and in NED **z=0 is the ground**, so from the `z = −1 m`
   (1 m altitude) start the position PD commands a descent into the ground — why the sim descends
   and trips the crash check. For an altitude-hold/hover test set the z setpoint to the target
   altitude (e.g. `−1`).

2. **Verify `aero.py` Cm sign.** Lift/drag are now FRD-correct (§7.3), but confirm the XFLR5 `Cm`
   column is nose-up-positive about +y before trusting the pitch response.

3. **`aero_comp` units.** §3 subtracts an aero **force** (N) from an acceleration command (m/s²)
   without dividing by mass — likely should be `a_com − a_aero_global / m`.

4. **FW bank-to-turn (§4.3) unvalidated in NED.** The coordinated-turn construction (`phi_d`,
   ZXY-Euler `q_d_fw`) was carried over unchanged; validate signs in forward flight (only active
   above the `V_min = 2 m/s` blend).

5. **Mass mismatch (deferred).** `config.MASS = 0.829 kg` vs CSV "Total 560 g" (CSV row is itself
   inconsistent: 560 g ≠ 1.653 lb). Scales every force↔acceleration conversion.

6. **Prop size (deferred).** 6-inch props described vs **5 in** in the CSV.

7. **Moment coefficients are placeholders.** `cx = 0.144`, `cy = 0.0616` ("DERIVED GEMINI 4/28"),
   assume radian deflections with CG unspecified. Replace with a derived slipstream/elevon model
   once CG and prop wash are characterized.

8. **A2 conditioning.** Allocation stage A2 is singular when either motor thrust is at/near the
   0.1 N floor (slipstream ≈ 0). Consider a regularized/clamped inverse or a minimum effective thrust.

9. **Sim duration.** `main.py` always breaks at `t > CRASH_CHECK_TIME = 3.0 s`, so `FINAL_TIME =
   3.4 s` and the `t >= tf` exit are dead code; the run is effectively 3.0 s.

10. **Integration order.** Forward Euler at 500 Hz; rotational dynamics + quaternion kinematics
    would benefit from RK4 (already flagged in-code).

---

## 9. Parameter reference (`config.py`)

| Group | Parameter | Value |
|-------|-----------|-------|
| Physical | GRAVITY / AIR_DENSITY | 9.8 m/s² / 1.23 kg/m³ |
| Mass/inertia | MASS / INERTIA_TENSOR | 0.829 kg / diag(0.002814, 0.003882, 0.002334) |
| Geometry | WING_AREA / MEAN_AERO_CHORD | 0.0923 m² / 0.1759 m |
| Geometry | MOMENT_ARM / THRUST_MOMENT_ARM_Y_m | 0.14 m / 0.14 m |
| Allocation | MOMENT_COEFF_X / _Y | 0.144 / 0.0616 |
| Limits | MAX_THRUST_ONE_MOTOR_N | 6.62 N |
| Limits | MIN/MAX_DEFLECTION_TED_RAD | ±20° |
| Sim | SIMULATION_RATE / DT / FINAL_TIME | 500 Hz / 0.002 s / 3.4 s |
| Init | POSITION / VELOCITY | [0,0,−1] m (1 m alt, NED) / [0,0,0] |
| Init | QUATERNION / ANGULAR_VELOCITY | [0.7071,0,0.7071,0] (nose-up hover) / [0,0,0] |
| Target | FINAL_POSITION | [1, −1, −0.6] m (NED) |
| Guidance gains | Kp_BG / Kd_BG | diag(12) / diag(4) |
| Attitude gains | Kp_ATTITUDE / Kd_ATTITUDE / LAMBDA | diag(3.7) / diag(0.19) / diag(0.2) |
| Aero (flat plate) | CD_MAX_FLAT / CD_0_FLAT | 1.2 / 0.05 |
| Safety | MIN_ALTITUDE / CRASH_CHECK_TIME | 0.1 m / 3.0 s |

Gate parameters (`GATE_*`) and `Kp/Kd_POSITION` exist for the legacy polynomial/position path and
are not used by the active basic-guidance loop.
