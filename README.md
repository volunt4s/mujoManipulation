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

It is deliberately *not* first, though. Two phases of actual control code come first, so the
project cannot stall in web infrastructure before it has anything to show.

### Ported from `hand_simulator`

The web layer is not being invented here. A sibling project of mine
(`../hand_simulator`, FastAPI + WebSocket + Three.js over MuJoCo) already solved every hard
part, and the pieces map across directly:

| There | Here |
|---|---|
| `viewer.js :: updateContacts()` — contact list → `THREE.ArrowHelper` pairs | The generic draw-primitive renderer |
| `ConvexGeometry` workspace hull | The manipulability ellipsoid |
| `{"type": ...}` WebSocket messages | The topic-like command/state protocol |
| `SimulationThread` — fixed-rate loop, kinematic ↔ physics modes | The control loop server |
| `client.py` — ZMQ + slider client | The Python client and the gain-tuning UI |

---

## Quickstart

```bash
conda activate mujoco_sim        # python 3.12, mujoco 3.9
python examples/01_pid.py
```

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

### Phase 0 — Foundation
- [ ] Rebuild the package skeleton: `env / robot / controller / viz`
- [ ] `MuJoCoEnv(render=False)` — headless must work from day one
- [ ] Split arm (7 torque channels) from gripper (2 position channels)
- [ ] Draw-primitive types + the native `user_scn` renderer
- [ ] **Gravity compensation** — the arm holds itself up, with the per-joint gravity torque
      drawn as arrows you can watch shrink as it engages

### Phase 1 — The control ladder (Panda only)
- [ ] Port the existing PID onto the new skeleton
- [ ] Computed torque / inverse dynamics
- [ ] Operational-space control (OSC)
- [ ] Impedance control
- [ ] Each ships with: the equation in its docstring, its quantities drawn via `viz/`,
      and a "break it" toggle in its demo

### Phase 2 — Robot-agnostic (Panda + UR5e)
- [ ] Migrate assets to `mujoco_menagerie`, keeping locally-defined torque actuators
- [ ] `RobotConfig` — index map, EE site, home pose
- [ ] **Demo: one OSC controller, two robots, same Cartesian trajectory**

### Phase 3 — Web viewer + command server
- [ ] Fixed-rate sim loop behind a WebSocket, state published every step
- [ ] Three.js viewer consuming the same draw primitives as the native renderer
- [ ] **Live gain sliders** — retune `K_p` mid-run and watch the response change
- [ ] Plots alongside the 3D view (tracking error, torque)
- [ ] Python client: `robot.move_to(...)`, `robot.set_controller("osc")`
- [ ] The control library must stay fully usable with the server switched off

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
