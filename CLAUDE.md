# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Environment

MuJoCo is not in the system python. Use the conda env:

```bash
conda activate mujoco_sim   # python 3.12, mujoco 3.9.0
```

`requirements.yaml` (one line, `mujoco`) and `setup.py` (empty) are placeholders — the package is not installable. Run scripts from the repo root so `mujoManipulation` resolves as a namespace package (there is no top-level `__init__.py`).

## Run

```bash
python spawn_new.py     # opens passive viewer, PID-holds the Panda at idle pose
```

There are no tests, no linter, no CI. `test_sender.py` / `test_server.py` are hand-run
socket demos, not a test suite.

**Status: the repo is being rebuilt.** The README roadmap is the plan of record and the
current source is largely pre-rewrite. Only the PID law (anti-windup + filtered derivative)
and the `<mujocoinclude>` actuator/body split are meant to carry over. Do not invest in
repairing the rest — check the roadmap phase first.

## Architecture

Four pieces, deliberately decoupled — the sim loop lives in the caller, not in a framework:

- `mujoManipulation/env.py` — `MuJoCoEnv` loads the XML and introspects the model into flat attributes (`joint_names`, `ctrl_names`, `ctrl_ranges`, plus `rev_*`/`pri_*` groups built by `_parse_specific_joints`). Owns `model`, `data`, `viewer`, and `step()`. The viewer is launched in `__init__`, so constructing an env always opens a window — there is no headless path.
- `mujoManipulation/robot/Panda.py` — `FrankaPanda(model, data)` is a thin view over MjData: `idle_joint` (9-vector), `joint_value` **aliases `data.qpos`** (a live numpy view, so it tracks the sim without re-reading), and `control(torque)` writes `data.ctrl`.
- `mujoManipulation/controller/PID.py` — vectorized PID over the whole 9-dim ctrl vector. Anti-windup: the integral only accumulates on channels where the unclipped output equals the clipped one. D-term is low-passed with `tau` (`alpha/beta` first-order filter). Output limits come from `robot.model.actuator_ctrlrange`.
- `mujoManipulation/assets/` — robosuite-derived Panda XMLs.

Caller-driven loop (see `spawn_new.py`): `controller.update(desired, curr)` → `panda.control(ut)` → `env.step()` → `env.viewer.sync()`.

`config/` and `utils/` are empty stubs.

### Control vector layout

`nq == nv == nu == njnt == 9` for `franka_panda.xml`, and the ordering lines up across qpos / ctrl / joints:

| idx | joint | actuator | type | ctrlrange |
|-----|-------|----------|------|-----------|
| 0–4 | `panda_joint1..5` | `panda_torq_j1..5` | motor (torque) | ±80 |
| 5–6 | `panda_joint6..7` | `panda_torq_j6..7` | motor (torque) | ±12 |
| 7 | `panda_finger_joint1` | position, kp=1000 | position | 0 .. 0.04 |
| 8 | `panda_finger_joint2` | position, kp=1000 | position | -0.04 .. 0 |

The single PID drives all 9 channels with the same gains, so indices 7–8 get a PID output fed into *position* actuators while 0–6 get torque. Gripper behavior is therefore a side effect of the arm gains — change gains per-channel before trusting gripper motion.

### XML composition

`assets/panda/franka_panda.xml` is the top-level model: it sets `<option>`/`<size>` then `<include>`s, in order, `assets/panda_assets.xml` (meshes) → `../common_arena/<arena>.xml` → `assets/panda_body.xml` (kinematic tree) → the two actuator files. Swap the arena or actuator include to change the scene or the control mode; the included files are `<mujocoinclude>` fragments, not standalone models. Other arenas exist under `assets/common_arena/` (table, bins, pegs, door, empty) but only `simple_plane` is wired up.

## Direction

Read the roadmap in `README.md` before proposing structural changes. The short version:
this is a **robot-agnostic control library**, scope capped at control algorithms. The
"Known breakage" below is Phase 0. Explicit non-goals — do not add these: RL/Gym wrappers
(mjlab and IsaacLab already cover that), a task framework, perception.

Robot-specific differences should stay confined to a config object (index map, EE site
name, home pose), not a class hierarchy.

### This is a teaching repo — it changes what "good code" means here

The primary goal is that a reader can see how an equation becomes code. Two consequences
that override normal library instincts:

- **Do not DRY the controllers.** Each controller is one self-contained file. Duplicated
  error computation is preferred over a base class that hides where torque is produced.
  Refactoring `controller/` toward shared helpers is a regression, not a cleanup.
- **Do not reach for a solver library** for anything the repo teaches (LQR Riccati
  recursion, IK iteration, MPC optimisation, trajectory polynomials). Using MuJoCo for the
  *model* (`mj_fullM`, `qfrc_bias`, `mj_jacSite`, `mjd_transitionFD`, `mujoco.rollout`) is
  correct and expected — that is the plant, not the algorithm.

Each controller carries its equation in the docstring above the implementing lines, its
quantities drawn through `viz/`, and a demo toggle that disables one term to show the
failure it prevents.

**Visualisation is the product, not decoration.** The repo exists because the author found
control theory hard to learn when the quantities were invisible. If a symbol appears in a
control law, there should be a way to see it in the viewer — via `viewer.user_scn` with
`mjv_initGeom` / `mjv_connector` (`mjGEOM_ARROW`, `mjGEOM_ELLIPSOID`) and the `mjVIS_*`
flags. Build `viz/` primitives before the controllers that use them; retrofitting produces
one ad-hoc version per controller.

Controllers emit draw primitives as **plain data** and never import a viewer. Two renderers
consume them: the native `user_scn` one (Phase 0, for the dev loop) and a Three.js web
viewer (Phase 3, the one that matters for sharing and for live gain sliders).

The web layer is a port, not an invention — `../hand_simulator` (FastAPI + WebSocket +
Three.js over MuJoCo, same author) already has the arrow rendering (`viewer.js ::
updateContacts`), the convex-hull mesh rendering, the typed `{"type": ...}` WS protocol, a
fixed-rate `SimulationThread`, and a slider client. Read those before writing anything new
for Phase 3.

## Known breakage

- **`spawn_new.py` does not run on Linux.** It imports `mujoManipulation.robot.panda` and `mujoManipulation.controller.pid`, but the files are `Panda.py` and `PID.py`. The repo was developed on case-insensitive macOS. Fix by matching the case (renaming the files to lowercase is the cleaner direction).
- **`franka_panda_w_objs.xml` fails to load** — its arena include points at `../../asset/common_arena/simple_plane.xml`; the correct path is `../common_arena/simple_plane.xml`.
- **`test_server.py` is stale.** It calls `FrankaPanda(xml_path=...)`, `.idle_pose`, `.mj_model`, `.mj_data`, and `PIDController(robot_model=...)` — none of which exist after commit `a25a77a` ("Update robot spawn method"), which moved the robot to `FrankaPanda(model, data)`. It also hardcodes an absolute macOS path. Port it to `MuJoCoEnv` before using it.
- PID is constructed with `dt=0.001` while the model timestep is `0.002`. Intentional or not, the I and D terms are scaled against the wrong step.

## Branches

`main` is active development. `legacy` (remote-only) holds the older stable version and is unmaintained — don't port fixes there.
