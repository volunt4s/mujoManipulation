# mujoManipulation 🤖

**Manipulator control you can watch — every term in the equation drawn in the simulator.**

Robotics control is taught in symbols. A mass matrix, a Jacobian, a null-space projection —
you can pass the exam without ever having seen what any of them *are*. This repo implements
the classical control ladder from scratch, one readable file per law, and draws every
quantity in the equation as geometry you can look at while the arm moves.

> ⚠️ Early stage, under active development.

## Run

```bash
conda env create -f environment.yml     # first time only
conda activate mujoco_sim

python -m mujoManipulation.server       # then open http://127.0.0.1:8000
```

Pick a start pose, an end pose and a profile, then **switch the control law underneath while
the arm is still moving**. Every term of the running equation is an arrow in the scene, in
the colour the equation gives it, and any one of them can be switched off mid-motion to show
the failure it prevents.

Each rung also runs on its own from the command line and prints its numbers:

```bash
python examples/00_gravity_comp.py --headless            # the arm holds itself up
python examples/00_gravity_comp.py --headless -b         # and here is what it was holding
python examples/01_joint_pid.py --headless -b i          # no integral: a permanent droop
python examples/02_computed_torque.py --headless --pid   # the same stroke, no model
python examples/03_osc.py --headless --settle -b         # the tip arrives, the elbow wanders
```

On macOS a run that opens a MuJoCo window needs `mjpython`; `--headless` and the server
do not.

## What it shows

One task — home to a reach pose on a minimum-jerk profile — under each law in turn:

| | end effector | peak joint error | settled |
|---|---|---|---|
| Gravity compensation | 202 mm off target | 57.30° | 57.296° |
| Joint-space PID | 0.37 mm | 7.06° | 0.049° |
| Computed torque | 0.00 mm | 0.04° | 0.000° |
| Operational-space control | 0.00 mm | 21.96° | 0.049° |

Feedforward alone never leaves home. Feedback arrives seven degrees behind. Putting the
model in the loop tracks to four hundredths of a degree — and the same two gains then work
at every pose, where the PID needed seven hand-tuned pairs.

The last row is being measured by the wrong yardstick on purpose. Operational-space control
is given a *pose*, not seven angles, and what the joints do on the way there is a free
parameter rather than an error: seven joints chasing six numbers have one degree of freedom
left over. Switch its null-space term off and the tip still lands within 0.08 mm while the
arm settles 98° away from the posture it was handed — the clearest picture of redundancy
there is, and `examples/03_osc.py --settle -b` prints it.

## The one rule

**MuJoCo provides the model. This repo writes the algorithm.** The mass matrix, the
Jacobians, `mjd_transitionFD` — that is the plant, and asking for it is not cheating.
Importing something that computes the *control law* is, because that is the part you came
here to read. It follows that there is no solver library for anything being taught, that
each controller is one self-contained file, and that the symbols match the textbook.

Controllers never import a viewer. They emit draw primitives as plain data, and two
renderers consume the same primitives: MuJoCo's native `user_scn` for the development loop,
and a Three.js client for the browser.

## Layout

```
env.py          model loading, introspection, stepping, optional viewer
robot/          per-robot state access and torque command
controller/     the control ladder — one file each, sharing an interface only
viz/            draw primitives, and the native renderer
server/         fixed-rate loop, the scenario registry, the WebSocket protocol
web/            Three.js viewer and the generated controls — no build step
examples/       one runnable demo per controller
```

Not building: RL / Gym wrappers, a task framework, perception. Scope is capped at control
algorithms.

The previous, stable-but-unmaintained version lives on the **`legacy`** branch.
