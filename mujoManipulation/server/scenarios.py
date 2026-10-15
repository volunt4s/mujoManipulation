"""What the server can be asked to run, and how each run describes itself.

The menu in the browser is generated, not written. Nothing in `web/` knows that
a PID has an integral or that computed torque has a mass matrix: each scenario
hands over its own title, its own equation with the colour of every term, its
own break-it variants and its own choice of reference, and the client renders
whatever arrives. Adding the OSC rung should mean adding an entry here and
touching no JavaScript at all.

The colours come from the controller modules rather than from CSS, because a
term's colour in the equation has to be the colour of its arrow in the scene,
and the controller is the only place that decides it.

Each scenario is an independent class with no base class, in the same spirit as
the controllers: they share the four names the loop calls — `step`, `reset`,
`set_reference`, `set_scale` — and share no implementation. The body of `step`
is deliberately the body of the matching example's loop, duplicated rather than
abstracted, because the point of both is to be read start to finish.

Gains are not adjustable from here yet; the sliders and the per-term toggles
arrive with the equation panel. What a client can do today is choose a rung,
choose which term to break, choose the reference, and set the arrow scale.
"""

import numpy as np

from ..controller import computed_torque as ct
from ..controller import gravity_comp as gc
from ..controller import pid as pid_mod
from ..controller.computed_torque import ComputedTorque
from ..controller.gravity_comp import GravityCompensation
from ..controller.pid import JointSpacePID


def _term(text, rgba, term_id=None):
    piece = {"text": text, "rgba": list(rgba)}
    if term_id:
        piece["term"] = term_id
    return piece


def _plain(text):
    return {"text": text}


# --- rung 0 ---------------------------------------------------------------


class GravityRun:
    """τ = g(q). Asked to hold the home pose, and nothing else."""

    id = "gravity"
    title = "Gravity compensation"
    subtitle = "feedforward only — it never looks at where the arm is"
    equation = [_plain("τ = "), _term("g(q)", gc.LOAD_RGBA, "gravity")]
    variants = [
        {"id": None, "label": "on", "note": "worst joint droop 0.000°"},
        {"id": "off", "label": "compensation off", "note": "the arm falls — 167°"},
    ]
    references = [{"id": "hold", "label": "hold the home pose"}]

    reference = "hold"            # there is only one, but state() should say so

    def __init__(self, robot, dt, variant=None):
        self.robot = robot
        self.home = robot.HOME.copy()
        self.controller = GravityCompensation(robot, enabled=variant != "off")

    def step(self, t):
        tau, primitives = self.controller.update()
        return tau, primitives, self.home

    def reset(self):
        self.robot.go_home()

    def set_reference(self, _ref_id):
        pass                      # there is only one

    def set_scale(self, nm_per_metre):
        self.controller.nm_per_metre = nm_per_metre


# --- rung 1 ---------------------------------------------------------------


class PIDRun:
    """τ = K_p e + K_i ∫e dt + K_d ė, against examples/01's square wave."""

    id = "pid"
    title = "Joint-space PID"
    subtitle = "feedback only — it is told nothing about the arm"
    equation = [
        _plain("τ = "),
        _term("K_p e", pid_mod.P_RGBA, "p"),
        _plain(" + "),
        _term("K_i ∫e dt", pid_mod.I_RGBA, "i"),
        _plain(" + "),
        _term("K_d ė", pid_mod.D_RGBA, "d"),
    ]
    variants = [
        {"id": None, "label": "full PID", "note": "47.4% overshoot, settles 2.31 s"},
        {"id": "d", "label": "no D term", "note": "276% overshoot, never settles"},
        {"id": "i", "label": "no I term", "note": "a permanent 7–9° droop"},
        {"id": "windup", "label": "no anti-windup", "note": "needs an unreachable target"},
    ]
    references = [
        {"id": "square", "label": "square wave, every 3.5 s"},
        {"id": "hold", "label": "hold the home pose"},
        {"id": "limit", "label": "a target past joint 4's limit"},
    ]

    # examples/01's numbers, copied rather than imported so that each stays
    # readable on its own — and so the seven pairs sit where you can see them.
    K_P = np.array([120.0, 120.0, 120.0, 120.0, 50.0, 20.0, 10.0])
    K_D = np.array([26.0, 30.0, 24.0, 23.0, 9.0, 4.8, 2.5])
    K_I = np.array([185.0, 160.0, 195.0, 205.0, 95.0, 28.0, 14.0])
    STEP = np.array([0.6, 0.35, 0.0, 0.5, 0.0, -0.5, 1.0])
    HOLD_SECONDS = 3.5
    PAST_LIMIT = 0.4

    def __init__(self, robot, dt, variant=None):
        self.robot = robot
        self.home = robot.HOME.copy()
        self.controller = JointSpacePID(
            robot, K_p=self.K_P, K_i=self.K_I, K_d=self.K_D, dt=dt,
            d_enabled=variant != "d",
            i_enabled=variant != "i",
            anti_windup=variant != "windup",
        )
        # Windup only happens against a demand that cannot be met, so that
        # variant brings its own reference with it.
        self.set_reference("limit" if variant == "windup" else "square")

    def _target(self):
        target = self.home + self.STEP
        if self.reference == "limit":
            j4 = self.robot.arm_joint_ids[3]
            target[3] = self.robot.model.jnt_range[j4, 0] - self.PAST_LIMIT
        return target

    def step(self, t):
        if self.reference == "hold":
            q_ref = self.home
        else:
            outbound = int(t / self.HOLD_SECONDS) % 2 == 0
            q_ref = self._target() if outbound else self.home
        tau, primitives = self.controller.update(q_ref)
        return tau, primitives, q_ref

    def reset(self):
        self.robot.go_home()
        self.controller.reset()

    def set_reference(self, ref_id):
        self.reference = ref_id if ref_id in {r["id"] for r in self.references} else "square"

    def set_scale(self, nm_per_metre):
        self.controller.nm_per_metre = nm_per_metre


# --- rung 2 ---------------------------------------------------------------


class ComputedTorqueRun:
    """τ = M(q) q̈_cmd + C(q, q̇) q̇ + g(q), against examples/02's stroke.

    The equation is written with h(q, q̇) split into its two halves, because
    that is how the code computes it and how the demo switches them off.
    """

    id = "computed_torque"
    title = "Computed torque"
    subtitle = "the model goes into the loop — one gain pair for all seven joints"
    equation = [
        _plain("τ = "),
        _term("M(q)", ct.INERTIAL_RGBA, "m"),
        _plain("(q̈_d + K_d ė + K_p e) + "),
        _term("C(q, q̇) q̇", ct.CORIOLIS_RGBA, "c"),
        _plain(" + "),
        _term("g(q)", ct.GRAVITY_RGBA, "g"),
    ]
    variants = [
        {"id": None, "label": "full law", "note": "0.372° peak at 2.8 rad/s"},
        {"id": "m", "label": "no M(q)", "note": "4.223°, 11x worse"},
        {"id": "c", "label": "no Coriolis", "note": "costs 0.72° — and 0.06° at quarter speed"},
        {"id": "g", "label": "no g(q)", "note": "joint 4 sits 3.3° low all stroke"},
    ]
    references = [
        {"id": "stroke", "label": "1 Hz joint-space stroke"},
        {"id": "hold", "label": "hold the home pose"},
    ]

    # examples/02's numbers. K_p = ω², K_d = 2ζω: a specification, not a tuning.
    OMEGA, ZETA = 20.0, 1.0
    AMPLITUDE = np.array([0.9, 0.5, 0.0, 0.7, 0.6, 0.5, 0.0])
    FREQUENCY = 1.0

    def __init__(self, robot, dt, variant=None):
        self.robot = robot
        self.home = robot.HOME.copy()
        self.controller = ComputedTorque(
            robot,
            K_p=self.OMEGA**2,
            K_d=2.0 * self.ZETA * self.OMEGA,
            use_inertia=variant != "m",
            use_coriolis=variant != "c",
            use_gravity=variant != "g",
        )
        self.set_reference("stroke")

    def step(self, t):
        if self.reference == "hold":
            q_d = self.home
            qd_d = qdd_d = np.zeros(self.robot.n_arm)
        else:
            # q_d = home + A(1 − cos ωt)/2, differentiated twice by hand. The
            # (1 − cos) shape so position and velocity both match the arm at
            # t = 0 and nothing is a startup transient.
            w = 2.0 * np.pi * self.FREQUENCY
            half = self.AMPLITUDE / 2.0
            q_d = self.home + half * (1.0 - np.cos(w * t))
            qd_d = half * w * np.sin(w * t)
            qdd_d = half * w**2 * np.cos(w * t)
        tau, primitives = self.controller.update(q_d, qd_d, qdd_d)
        return tau, primitives, q_d

    def reset(self):
        self.robot.go_home()
        self.controller.reset()

    def set_reference(self, ref_id):
        self.reference = ref_id if ref_id in {r["id"] for r in self.references} else "stroke"

    def set_scale(self, nm_per_metre):
        self.controller.nm_per_metre = nm_per_metre


# --- the catalogue --------------------------------------------------------

SCENARIOS = {run.id: run for run in (GravityRun, PIDRun, ComputedTorqueRun)}


def valid_variant(run_id: str, variant):
    """`variant` if that rung has one by that name, otherwise None.

    Rungs do not share break-it modes — "no Coriolis" means nothing to a PID —
    and silently running the unbroken law while reporting a broken one is the
    one outcome worth ruling out.
    """
    run = SCENARIOS.get(run_id)
    if run is None:
        return None
    return variant if variant in {v["id"] for v in run.variants} else None
DEFAULT = ComputedTorqueRun.id          # the one that moves, so the pipe shows


def catalogue() -> list:
    """Everything the menu needs, in ladder order. Rungs that do not exist yet
    are listed too, greyed out, because the menu *is* the roadmap."""
    listed = [
        {
            "id": run.id,
            "title": run.title,
            "subtitle": run.subtitle,
            "equation": run.equation,
            "variants": run.variants,
            "references": run.references,
            "available": True,
        }
        for run in SCENARIOS.values()
    ]
    listed += [
        {"id": "osc", "title": "Operational-space control",
         "subtitle": "Phase 1 — not built yet", "equation": [],
         "variants": [], "references": [], "available": False},
        {"id": "impedance", "title": "Impedance control",
         "subtitle": "Phase 1 — not built yet", "equation": [],
         "variants": [], "references": [], "available": False},
    ]
    return listed
