# Roadmap

The plan of record. Moved out of the README to keep that short.

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
