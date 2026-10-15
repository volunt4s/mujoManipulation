# mujoManipulation 🤖

**Manipulator control you can watch — every term in the equation drawn in the simulator.**

Robotics control is taught in symbols. A mass matrix, a Jacobian, a null-space projection —
you can pass the exam without ever seeing what any of them *are*.

This repo implements the classical control ladder from scratch, one readable file each, and
then draws every quantity in the equation as geometry in the MuJoCo viewer: the gravity
torque holding the arm up, the Jacobian columns fanning out from the gripper, the
manipulability ellipsoid flattening as you approach a singularity, the MPC's predicted
rollouts trailing ahead of the robot.

Those same controllers run unchanged on a Panda, a UR5e and an xArm7 — which is how the
abstractions are kept honest.

> ⚠️ Early stage, under active development. `main` moves frequently.

---

## Why this exists

I studied robotics as an undergraduate and found the control material hard for one specific
reason: none of it was visible. The equations were on the board, the code was somewhere
else, and the connection between them was left as an exercise. You end up manipulating
symbols you have never actually seen.

This is the repo I wanted then — so that nobody has to work it out the hard way again.

Scope is deliberately capped at **control algorithms**. No RL wrappers, no task framework,
no perception. Most manipulation repos are built for reinforcement-learning throughput:
thousands of parallel environments, GPU rollouts, a training script. This one runs a single
robot, one step at a time, so the control law stays the thing you can read, watch and plot.

---

## Implementation philosophy

**MuJoCo provides the model. This repo writes the algorithm.**

| Taken from MuJoCo (physics / model) | Written out here (the algorithm) |
|---|---|
| `M(q)` mass matrix, `qfrc_bias` | The control law itself |
| Jacobians, forward kinematics | Inverse kinematics iteration |
| `mjd_transitionFD` linearisation | The Riccati recursion (LQR) |
| `mujoco.rollout` forward simulation | The optimisation loop (MPC) |

Using MuJoCo for the mass matrix is not cheating — that is the plant. Importing a library
that computes the *control law* is, because that is the part you came here to read.

Three rules follow from this:

1. **The code mirrors the equation.** Symbols match the textbook — `M`, `J`, `Lambda`,
   `tau` — and the equation being implemented sits in the docstring directly above the
   lines implementing it. Computed torque should read back as
   `τ = M(q)(q̈_d + K_dė + K_p e) + h(q, q̇)`.

2. **One controller, one self-contained file.** Controllers share an *interface*, not an
   implementation. Duplicating two lines of error computation beats a base class that hides
   where the torque actually comes from. This is deliberate: you should never have to open
   a second file to find the control law.

3. **No third-party solver for anything being taught.** An import that replaces the
   algorithm defeats the purpose of the repo.

### Everything in the equation gets drawn

MuJoCo's passive viewer exposes `user_scn`, so a controller can push geometry into the
scene on every step. This repo uses that systematically, not decoratively — if a symbol
appears in the control law, there is a way to see it:

| Concept | What appears in the viewer |
|---|---|
| Bias term `h(q, q̇)` | A torque arrow at each joint, scaled by magnitude |
| Jacobian `J(q)` | One arrow per joint at the gripper — the direction that joint moves the tip |
| Manipulability `√det(JJᵀ)` | An ellipsoid at the gripper; it flattens into a disc at a singularity |
| Operational-space error | The desired frame, the current frame, and the wrench between them |
| Null-space projection | The elbow moving while the tip stays pinned in place |
| Impedance control | The virtual spring–damper, drawn as an actual spring |
| MPC | The predicted rollouts, trailing ahead of the robot as ghost trajectories |

Built on `mjv_initGeom` / `mjv_connector` with `mjGEOM_ARROW` and `mjGEOM_ELLIPSOID`, plus
MuJoCo's own `mjVIS_CONTACTFORCE`, `mjVIS_COM` and `mjVIS_JOINT` flags.

Because this is the point of the repo rather than a finishing touch, the drawing
primitives are built in Phase 0 — **before** the controllers that use them. Retrofitting
visualisation onto five finished controllers produces five different ad-hoc versions of it.

### Every demo has a "break it" mode

Each example can disable one term and show the failure that term exists to prevent:

| Turn off | What you see |
|---|---|
| Gravity compensation | The arm sags under its own weight |
| `M(q)` in computed torque | Tracking error grows with configuration |
| Coriolis term `h(q, q̇)` | Error appears only at speed |
| Null-space term in OSC | The elbow drifts while the tip stays on target |

The contrast *is* the lesson. Seeing the term, then seeing what happens without it, is the
pair that makes it stick — these are not polish, they are the deliverable.

---

## How it renders

Two renderers, one contract. Controllers never import a viewer — they emit **draw
primitives** (arrow, frame, ellipsoid, ghost path) as plain data alongside their torque
output. Whatever is on the other end draws them.

| Renderer | Built in | For |
|---|---|---|
| Native — MuJoCo `user_scn` | Phase 0 | The development loop. ~30 lines, no server, instant iteration |
| Web — FastAPI + WebSocket + Three.js | Phase 3 | Sharing, and the sliders |

**The web viewer is the one that matters for the goal of this repo.** Somebody who is stuck
on what a Jacobian is should be able to open a link and drag a slider, not install MuJoCo
first. And live gain tuning — move `K_p`, watch the response change in the same second — is
the single most useful thing a control tutorial can offer, which a native viewer cannot give
cheaply.

It was meant to come after the whole ladder, so that the project could not stall in web
infrastructure before it had anything to show. It got pulled forward once three rungs
existed, for a reason the plan had not anticipated: **the web layer is a better instrument
than the command line, not just a prettier one.** Switching `--break-it` from the terminal
means restarting the process; in the browser it is a click, mid-stroke, with the arrows
disappearing while the arm keeps moving. The contrast *is* the lesson, and that is the only
place you can see both halves of it at once.

### How the client stays dumb

The browser never learns what a robot is. It receives the compiled model's triangles once —
not the STL files, which gzip badly and carry no promise about which frame their vertices
live in — and after that a world transform per geom per frame. There is no forward
kinematics, no notion of a body or a joint, and no per-controller code anywhere in `web/`:

| Server | Browser |
|---|---|
| `mesh_vert` / `mesh_face` from the compiled model, once | triangles, 1.72 MB gzipped |
| `geom_xpos` / `geom_xmat`, 12 floats a geom a frame | `Matrix4.set()`, row-major, no conversion |
| draw primitives — the same plain data `viz/native.py` eats | `web/primitives.js`, the twin of that file |
| `server/scenarios.py` — each rung's title, equation, term colours, break-it variants | a menu, generated |

The last row is the one that matters for keeping it honest: a term's colour in the equation
has to be its arrow's colour in the scene, so the colours live in the controller modules and
are sent to the browser rather than written into CSS. Adding a rung is an entry in
`scenarios.py` and no JavaScript at all.

A sibling project (`hand_simulator`, FastAPI + WebSocket + Three.js over MuJoCo) solved much
of this once already and was the intended source to port from; it turned out not to be on
this machine, so the web layer here is an independent implementation.

---

## Quickstart

```bash
conda env create -f environment.yml     # first time only
conda activate mujoco_sim               # python 3.12, mujoco 3.9.0

python examples/00_gravity_comp.py              # the arm holds itself up
python examples/00_gravity_comp.py --break-it   # and here is what it is holding

python examples/01_joint_pid.py                 # and now it goes where you ask
python examples/01_joint_pid.py --break-it      # without the D term, it rings

python examples/02_computed_torque.py           # with the model in the loop: 0.37°
python examples/02_computed_torque.py --pid      # the same stroke without it: 17.4°
```

Every demo also takes `--headless`, which skips the window and prints the numbers instead.
On macOS, MuJoCo's passive viewer has to be launched under `mjpython` rather than `python`
— `mjpython examples/01_joint_pid.py` — and `--headless` runs fine under either.

---

## Design

### Layers

```
env.py          MuJoCo model loading, introspection, stepping, optional viewer
robot/          Per-robot state access + torque command  (Panda, UR5e, xArm7)
controller/     The control ladder — one file each, sharing an interface only
kinematics/     FK / Jacobian / IK helpers over the MuJoCo API
planning/       Trajectory generation (joint-space, Cartesian, min-jerk)
viz/            Draw primitives — arrows, frames, ellipsoids, ghost paths
server/         Fixed-rate sim loop + WebSocket command/state protocol   (Phase 3)
web/            Three.js viewer, gain sliders, live plots                  (Phase 3)
examples/       One runnable demo per controller
```

The simulation loop lives in the caller, not in a framework:

```python
u = controller.update(desired, robot.state)
robot.set_torque(u)
env.step()
```

### The robot interface

Every controller in the ladder needs some subset of these, and nothing else:

| Quantity | Source | First needed by |
|---|---|---|
| `q`, `qd` | `qpos` / `qvel` slice | PID |
| `bias(q, q̇)` | `qfrc_bias` | Gravity compensation |
| `M(q)` | `mj_fullM` | Computed torque |
| `J(q)` | `mj_jacSite` | Operational-space control |
| `A`, `B` | `mjd_transitionFD` | LQR |
| rollout | `mujoco.rollout` | MPC |
| `set_torque(τ)` | `ctrl` slice | all |

So what actually differs between robots is only an **index mapping, an end-effector site
name, and a home pose** — a small config object, not a class hierarchy. Adding a fourth
robot should cost one config entry.

### Assets

Robot bodies and meshes come from
[`mujoco_menagerie`](https://github.com/google-deepmind/mujoco_menagerie), but the
**actuator block is defined here**: menagerie models generally ship position actuators,
while the whole control ladder above operates in torque space. The existing
`<mujocoinclude>` split (`panda_assets` / `panda_body` / `*_actuator`) is exactly the right
seam for that, and is kept.

---

## Roadmap

Little of the current code survives this plan, and that is fine — the repo is small. The
PID law (anti-windup, filtered derivative) carries over, as does the `<mujocoinclude>`
split that keeps actuators separate from robot bodies. Everything else is rewritten.

### Phase 0 — Foundation ✅
- [x] Rebuild the package skeleton: `env / robot / controller / viz`
- [x] `MuJoCoEnv(render=False)` — headless must work from day one
- [x] Split arm (7 torque channels) from gripper (2 position channels)
- [x] Draw-primitive types + the native `user_scn` renderer
- [x] **Gravity compensation** — the arm holds itself up, with the per-joint gravity torque
      drawn as arrows, and the *uncompensated remainder* drawn behind them. `g(q)` is a
      property of the pose and does not shrink when you compensate; the remainder is the
      arrow that vanishes, and `--break-it` brings it back at full length (0.000° droop
      against 167°)

### Phase 1 — The control ladder (Panda only)
- [x] **Joint-space PID** — arm-only channels, gains set by pole placement off the
      mass-matrix diagonal, and each of the three terms drawn as its own arrow at the
      joint it acts on. The payoff measurement: left to itself the integral settles at
      [-10.4, 17.3, 1.3] N·m on joints 2, 4 and 6, against a g(q) of [-10.7, 17.4, 1.3]
      there — it finds gravity to 2% without being told gravity exists. Three break-it
      modes: no D (276% overshoot, never settles), no I (a permanent 7-9° droop, which is
      g(q)/K_p to the digit), no anti-windup (the integral winds to -306 N·m against an
      ±80 N·m actuator)
- [x] **Computed torque / inverse dynamics** — τ = M(q)(q̈_d + K_dė + K_pe) + h(q, q̇), which
      cancels the plant and leaves ë + K_dė + K_pe = 0: one (ω, ζ) pair for all seven
      joints, where PID needed seven hand-tuned pairs. Tracks a 1 Hz joint-space stroke at
      2.8 rad/s to 0.37° peak. M(q) q̈_cmd, C(q, q̇)q̇ and g(q) are drawn as three separate
      arrows per joint, and each switches off on its own: no M(q), no Coriolis (invisible
      until you go fast — it scales with q̇²), no gravity
- [ ] Operational-space control (OSC)
- [ ] Impedance control
- [ ] Each ships with: the equation in its docstring, its quantities drawn via `viz/`,
      and a "break it" toggle in its demo

### Phase 2 — Robot-agnostic (Panda + UR5e)
- [ ] Migrate assets to `mujoco_menagerie`, keeping locally-defined torque actuators
- [ ] `RobotConfig` — index map, EE site, home pose
- [ ] **Demo: one OSC controller, two robots, same Cartesian trajectory**

### Phase 3 — Web viewer + command server
- [x] Fixed-rate sim loop behind a WebSocket — physics at 500 Hz, one frame in eight
      published, measured at 60.3 fps and sim/real 1.0022
- [x] Three.js viewer consuming the same draw primitives as the native renderer
- [x] **A menu generated from the server** — every rung and every break-it variant is a
      click, applied between physics steps, with the arm still moving
- [ ] **Live gain sliders** — retune `K_p` mid-run and watch the response change
- [ ] Plots alongside the 3D view (tracking error, torque)
- [ ] Python client: `robot.move_to(...)`, `robot.set_controller("osc")`
- [x] The control library must stay fully usable with the server switched off — checked by
      blocking the fastapi import and loading every controller

### Phase 4 — Optimal control
- [ ] LQR about an operating point, linearised via `mjd_transitionFD`
- [ ] MPC over `mujoco.rollout` — predicted rollouts drawn as ghost trajectories

### Phase 5 — Third robot + release
- [ ] xArm7 — prove a new robot costs one config entry
- [ ] Per-controller theory notes, demo GIFs, "break it" comparison plots
- [ ] Deploy the web viewer so the README can link to something runnable

## Non-goals

| Not building | Because |
|---|---|
| RL / Gym environments | GPU-parallel RL belongs in IsaacLab / mjlab |
| Task framework (pick & place, insertion) | Scope is capped at control |
| Perception | Same |

---

## Legacy version

The previous, stable-but-unmaintained version lives on the **`legacy`** branch.
