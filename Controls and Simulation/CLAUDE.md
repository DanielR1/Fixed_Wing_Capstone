# CLAUDE.md — Fixed-Wing Tailsitter: Controls & Simulation

> Always read this file first when working in this directory. It is the orientation
> map for the project. The full guidance/control math lives in **[algorithm.md](algorithm.md)** —
> reference it whenever you touch the flight code or the simulator dynamics.
>
> **Keep the docs in sync:** whenever a change affects behavior, conventions, parameters, frames,
> or the algorithm, update **both** `CLAUDE.md` and `algorithm.md` as part of that same change.

## 1. What this project is

A custom flight stack + 6-DOF simulator for a **fixed-wing tailsitter drone**, written
in Python. The vehicle is a tailless **flying wing** (~0.53 m span) that takes off, hovers,
and transitions to forward flight using two motors and two elevons. The flight software is
a **non-linear, quaternion-based attitude controller** driven by a **pre-planned trajectory
guidance loop**.

**Actuation (4 control inputs):**
- **2 motors** with props, thrust along **body +x** (out the nose).
- **2 elevons** that deflect the motor slipstream (freestream airflow ignored for now).
  - Symmetric (together) → **pitch** moment.
  - Asymmetric (differential) → **roll** moment.
  - Elevon effectiveness scales with **thrust setting × deflection angle** (slipstream-driven).
- **Differential thrust** between the two motors → **yaw** moment.

## 2. Target hardware & environment

- **Power:** 4S LiPo. **Propulsion:** 2 motors + props (see prop-size note below).
- **Intended integration:** Prof. **Brett Lopez's VECTR lab** (UCLA) drone cage.
  - Motion-capture position tracking + the lab's custom navigation / state-estimation for
    aerial robots. The Python flight controller is designed to consume that state estimate
    (see `navigation.py`, currently a perfect-state pass-through).
  - Avionics modeled on the lab's quadrotors: power distribution board, IMU chip,
    **Teensy** flight controller, **Orange Pi** companion computer. (Details TBD.)

## 3. Repository layout

Everything currently lives under `Simulator/`. Run the code from `Simulator/scripts/`.

```
Controls and Simulation/
├── CLAUDE.md                      ← you are here
├── algorithm.md                   ← full guidance + control + dynamics math
└── Simulator/
    ├── P0 Dimesions - updated.csv ← vehicle geometry / mass / thrust spec sheet
    ├── .venv/                     ← Python 3.14 virtualenv (numpy, matplotlib, pillow)
    ├── data/                      ← sim output CSVs (data_<timestamp>.csv)
    └── scripts/
        ├── main.py                ← simulation entry point (the 6-DOF loop)
        ├── config.py              ← ALL parameters, gains, limits, initial conditions
        ├── plotter.py             ← plots the most-recent data/*.csv
        ├── quaternion_helpers.py  ← quat math, SLERP, Euler<->quat, quat<->R
        ├── helper_funcs.py        ← compute_alpha_beta() from state
        ├── rates.py               ← (empty placeholder)
        ├── flight_code/           ← THE FLIGHT STACK (would run on the real vehicle)
        │   ├── navigation.py      ← state estimation (currently perfect pass-through)
        │   ├── control.py         ← accel cmd → desired quat → SMC torque → allocation
        │   ├── aero_comp.py       ← aero feed-forward (subtract predicted lift/drag accel)
        │   └── guidance/
        │       ├── guidance.py        ← guidance dispatcher (selects a mode)
        │       ├── basic_guidance.py  ← active: timed accel profile + PD tracking
        │       └── polynomial_traj.py ← legacy 7th-order gate trajectory (unused/broken imports)
        └── truth_model/           ← THE PLANT (sim ground truth; NOT on real vehicle)
            ├── dynamics.py        ← rigid-body EOM, force/moment assembly, Euler integration
            ├── aero.py            ← XFLR5 table lookup + flat-plate blend for lift/drag/Cm
            ├── AERO_XFLR5.tsv      ← aero coefficient table vs alpha
            └── eq_motion.py       ← (empty placeholder)
```

**Key architectural split:** `flight_code/` is the controller that would be deployed on the
real drone (only sees the navigation state estimate). `truth_model/` is the simulated physics
that only `main.py` and the dynamics see. Keep that boundary clean — the controller must never
read truth-model internals.

## 4. How to run

The scripts use flat imports (`import config`, `from truth_model import dynamics`), so the
**script directory must be on the path** — run with `Simulator/` as the working directory.

```bash
cd "Simulator"
source .venv/bin/activate          # Python 3.14 venv
python scripts/main.py             # runs the sim, writes data/data_<timestamp>.csv
python scripts/plotter.py          # plots the most recent data/*.csv
```

Or use the VS Code launch configs in `Simulator/.vscode/launch.json`
("Python: Run main.py", "Python: Run plotter.py"). They set `cwd` to `Simulator/`.

- Output columns (20): `t, x,y,z, vx,vy,vz, qw,qx,qy,qz, wx,wy,wz, alpha,beta, T1,T2,delta1,delta2`
  (`t` + 15 state elements + 4 controls). Note `plotter.py` still assumes the old 13-element state,
  so its control-channel indices are off by two (see §8).
- `config.SAVE_DATA = True` controls whether a CSV is written.

## 5. State & frame conventions (read before editing dynamics/control)

**State vector** (`main.py` builds a 15-element array; dynamics integrate the first 13):

| idx | symbol | meaning | frame / units |
|-----|--------|---------|---------------|
| 0–2 | x, y, z | position | global, meters (**z is up**) |
| 3–5 | vx, vy, vz | velocity | global, m/s |
| 6–9 | qw, qx, qy, qz | attitude quaternion (body→global) | scalar-first `[w,x,y,z]` |
| 10–12 | wx, wy, wz | angular velocity | **body**, rad/s |
| 13 | alpha | angle of attack | **degrees** |
| 14 | beta | sideslip | **degrees** |

- **Global frame is z-up** (gravity acts in −z; altitude = z; crash when z < `MIN_ALTITUDE`). ENU-like, *not* NED.
- **Body frame:** x out the nose / thrust axis, y, z. `alpha = atan2(w_body, u_body)`, `beta = atan2(v_body, sqrt(u²+w²))`. Both **stored in degrees** (the XFLR5 table lookup uses degrees); converted to radians at every trig site.
- **Quaternion is scalar-first `[w,x,y,z]`**, represents body→global rotation (`quat_to_R`).
- **Control vector** `ctrl_in = [T1, T2, delta1, delta2]` — motor thrusts (N) for L/R (1=L, 2=R) and elevon deflections (radians, trailing-edge-down positive).

## 6. The guidance → control loop in one breath

Each tick (`main.py`, 500 Hz): **navigation** (perfect state) → **guidance** produces an
acceleration command `a_com` → **control** turns `a_com` into a desired attitude quaternion,
runs a sliding-mode attitude law to get a commanded torque, then **allocates** that torque +
total thrust into `[T1, T2, delta1, delta2]` → **dynamics** propagates the state.

```
nav.get_estimated_states → guid.get_a_com → controller.get_control_inputs → dyn.propagate
```

Full derivations (hover/fixed-wing desired-quaternion blend, SLERP, SMC torque law, the two
allocation matrices A1/A2, and the truth-model EOM) are in **[algorithm.md](algorithm.md)**.

## 7. Vehicle parameters (quick reference)

Geometry from `P0 Dimesions - updated.csv`; dynamics constants from `config.py`.

| Quantity | Value | Source |
|----------|-------|--------|
| Wing area S | 0.0923 m² | CSV / config |
| Wingspan | 0.53 m | CSV |
| Mean aero chord | 0.1759 m | CSV / config |
| Root / tip chord | 0.2049 / 0.1434 m | CSV |
| Taper / aspect ratio | 0.7 / 3.044 | CSV |
| Sweep | 15° | CSV |
| Airfoil | NACA 4415 (CLmax 1.5) | CSV |
| Elevon | 40% chord (rear), 60% span (centered) | CSV |
| CG | quarter-chord MAC (assumed) | CSV |
| Mass | **0.829 kg** (config) — CSV says 560 g (see §8) | config |
| Inertia (diag) | [0.002814, 0.003882, 0.002334] kg·m² | config |
| Max thrust / motor | 6.62 N (≈13.2 N total ≈ 1.35 kg) | config / CSV |
| Motor moment arm (yaw) | 0.14 m | config |
| Elevon moment coeffs | cx=0.144, cy=0.0616 (placeholder, "Gemini 4/28") | config |
| Max elevon deflection | ±20° | config |
| Sim rate / dt | 500 Hz / 0.002 s | config |
| Air density | 1.23 kg/m³ | config |

## 8. Conventions, gotchas & known discrepancies

The last commit notes the sim "runs and gets (wrong) answers." Several early issues are now
**fixed** — thrust axis (hover quaternion aligns body-x), force sign (`FM_aero + FM_control`),
alpha/beta units (radians at trig sites), and gravity feed-forward (added in `control.py`).
Remaining open items — confirm intent before "fixing." Full discussion in [algorithm.md](algorithm.md) §8.

- **`alpha`/`beta` are stored in DEGREES** (`compute_alpha_beta`). The XFLR5 table lookup and the
  AoA branch logic use degrees; every trig site converts to radians first (`np.radians`). Keep
  this split if you touch the aero code.
- **Guidance z-setpoint = 0.** `basic_guidance` only shapes the x-axis; its implied position
  setpoint is `[…, 0, 0]`, so from the `z = 1 m` start the position PD commands a descent toward
  the ground. This (plus the initial-attitude slew below) is why the sim currently descends/crashes.
- **Initial attitude vs hover.** `INITIAL_QUATERNION = [1,0,0,0]` = nose horizontal (body-x along
  +x). With gravity feed-forward, hover wants body-x pointing **up**, so a ~90° attitude slew is
  demanded at t=0. Confirm the intended start attitude for the maneuver under test.
- **`aero_comp` units.** It subtracts an aero **force** (N) from an acceleration command (m/s²)
  without dividing by mass — likely should be `… - a_aero_global / m`.
- **`plotter.py` column map is stale.** It assumes a 13-element state, but rows now carry 15
  (alpha, beta). Columns 14–19 are `[alpha, beta, T1, T2, delta1, delta2]`, so the "Motor Forces"
  plot actually shows alpha/beta/T1/T2. Fix the indices before trusting plots.
- **Sim stop time:** `main.py` unconditionally breaks at `t > CRASH_CHECK_TIME` (3.0 s), so
  `FINAL_TIME` (3.4 s) is never reached; effective run ≈ 3.0 s.
- **Allocation placeholders:** moment coeffs `cx = 0.144`, `cy = 0.0616` ("Gemini 4/28"); the A2
  matrix is singular when either motor is at the 0.1 N floor.
- **Integration:** plain forward **Euler** (`dynamics.propagate`); a `# Do RK4 later` note flags
  the intended upgrade.
- **Data-sheet mismatches (deferred):** mass (config `0.829 kg` vs CSV `560 g`) and prop size
  (6″ described vs 5″ in CSV) — to be reconciled later.
- `polynomial_traj.py` references undefined names — legacy, not on the active path.

## 9. Working agreements

- **Keep the docs in sync.** Whenever a change affects behavior, conventions, parameters, frames,
  or the algorithm, **update both `CLAUDE.md` and `algorithm.md` in the same change.** Treat the
  docs as part of "done."
- **`config.py` is the single source of truth** for parameters/gains/limits. Don't hard-code
  constants elsewhere — add them to `config.py`.
- Preserve the `flight_code/` ↔ `truth_model/` boundary (controller sees only nav state).
- Ask before changing sign/frame/unit conventions — several were recently ambiguous (see §8).
