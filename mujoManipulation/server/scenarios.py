"""What the server can be asked to run, and how each run describes itself.

The menu in the browser is generated, not written. Nothing in `web/` knows that
a PID has an integral or that computed torque has a mass matrix: each scenario
hands over its own title, its own equation with the colour of every term, its
own break-it variants, and the client renders whatever arrives. Adding the OSC
rung should mean adding an entry here and touching no JavaScript at all.

What a scenario is *not* responsible for is where the arm is going. The goal
pose and the profile that reaches it belong to the simulation (`reference.py`),
so that every rung answers the same question and the difference you see between
them is the law rather than the task.

The colours come from the controller modules rather than from CSS, because a
term's colour in the equation has to be the colour of its arrow in the scene,
and the controller is the only place that decides it.

Each scenario is an independent class with no base class, in the same spirit as
the controllers: they share the three names the loop calls — `step`, `reset`,
`set_scale` — and share no implementation. The body of `step`
is deliberately the body of the matching example's loop, duplicated rather than
abstracted, because the point of both is to be read start to finish.

Gains are not adjustable from here yet; the sliders and the per-term toggles
arrive with the equation panel.
"""

import numpy as np

from ..controller import computed_torque as ct
from ..controller import gravity_comp as gc
from ..controller import osc as osc_mod
from ..controller import pid as pid_mod
from ..controller.computed_torque import ComputedTorque
from ..controller.gravity_comp import GravityCompensation
from ..controller.osc import OperationalSpaceControl
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
    equation = [_plain("τ = "), _term("g(q)", gc.LOAD_RGBA, "gravity")]
    variants = [
        {"id": None, "label": "on"},
        {"id": "off", "label": "compensation off"},
    ]

    def __init__(self, robot, dt, variant=None):
        self.robot = robot
        self.controller = GravityCompensation(robot, enabled=variant != "off")

    def step(self, q_d, qd_d, qdd_d):
        # Every argument ignored, which is the whole character of this rung: it
        # is told where the arm should be and has no way to act on it.
        return self.controller.update()

    def reset(self):
        pass

    def set_scale(self, nm_per_metre):
        self.controller.nm_per_metre = nm_per_metre


# --- rung 1 ---------------------------------------------------------------


class PIDRun:
    """τ = K_p e + K_i ∫e dt + K_d ė, against examples/01's square wave."""

    id = "pid"
    title = "Joint-space PID"
    equation = [
        _plain("τ = "),
        _term("K_p e", pid_mod.P_RGBA, "p"),
        _plain(" + "),
        _term("K_i ∫e dt", pid_mod.I_RGBA, "i"),
        _plain(" + "),
        _term("K_d ė", pid_mod.D_RGBA, "d"),
    ]
    variants = [
        {"id": None, "label": "full PID"},
        {"id": "d", "label": "no D term"},
        {"id": "i", "label": "no I term"},
        {"id": "windup", "label": "no anti-windup"},
    ]

    # examples/01's numbers, copied rather than imported so that each stays
    # readable on its own — and so the seven pairs sit where you can see them.
    K_P = np.array([120.0, 120.0, 120.0, 120.0, 50.0, 20.0, 10.0])
    K_D = np.array([26.0, 30.0, 24.0, 23.0, 9.0, 4.8, 2.5])
    K_I = np.array([185.0, 160.0, 195.0, 205.0, 95.0, 28.0, 14.0])

    def __init__(self, robot, dt, variant=None):
        self.robot = robot
        self.controller = JointSpacePID(
            robot, K_p=self.K_P, K_i=self.K_I, K_d=self.K_D, dt=dt,
            d_enabled=variant != "d",
            i_enabled=variant != "i",
            anti_windup=variant != "windup",
        )

    def step(self, q_d, qd_d, qdd_d):
        # q̇_d goes in; q̈_d has nowhere to go, because this law has no term that
        # could carry it. That missing term is the next rung.
        return self.controller.update(q_d, qd_d)

    def reset(self):
        self.controller.reset()

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
    equation = [
        _plain("τ = "),
        _term("M(q)", ct.INERTIAL_RGBA, "m"),
        _plain("(q̈_d + K_d ė + K_p e) + "),
        _term("C(q, q̇) q̇", ct.CORIOLIS_RGBA, "c"),
        _plain(" + "),
        _term("g(q)", ct.GRAVITY_RGBA, "g"),
    ]
    variants = [
        {"id": None, "label": "full law"},
        {"id": "m", "label": "no M(q)"},
        {"id": "c", "label": "no Coriolis"},
        {"id": "g", "label": "no g(q)"},
    ]

    # examples/02's numbers. K_p = ω², K_d = 2ζω: a specification, not a tuning.
    OMEGA, ZETA = 20.0, 1.0

    def __init__(self, robot, dt, variant=None):
        self.robot = robot
        self.controller = ComputedTorque(
            robot,
            K_p=self.OMEGA**2,
            K_d=2.0 * self.ZETA * self.OMEGA,
            use_inertia=variant != "m",
            use_coriolis=variant != "c",
            use_gravity=variant != "g",
        )

    def step(self, q_d, qd_d, qdd_d):
        return self.controller.update(q_d, qd_d, qdd_d)

    def reset(self):
        self.controller.reset()

    def set_scale(self, nm_per_metre):
        self.controller.nm_per_metre = nm_per_metre


# --- rung 3 ---------------------------------------------------------------


class OSCRun:
    """τ = Jᵀ Λ(q)(K_p e + K_d ė) + h(q, q̇) + N τ₀, servoing the gripper frame.

    The task arrives as a joint pose like everyone else's, and the controller
    turns it into the end-effector frame it actually chases — which is what
    keeps the comparison honest while every goal on offer is still joint-space.
    """

    id = "osc"
    title = "Operational-space control"
    equation = [
        _plain("τ = Jᵀ "),
        _term("Λ(q)", osc_mod.WRENCH_RGBA, "l"),
        _plain("(K_p e + K_d ė) + "),
        _term("h(q, q̇)", osc_mod.BIAS_RGBA, "h"),
        _plain(" + "),
        _term("N τ₀", osc_mod.NULLSPACE_RGBA, "n"),
    ]
    variants = [
        {"id": None, "label": "full OSC"},
        {"id": "n", "label": "no null-space"},
        {"id": "l", "label": "no Λ(q)"},
        {"id": "h", "label": "no h(q, q̇)"},
    ]

    # examples/03's numbers. K_p is a stiffness in N/m, and it means the same
    # thing at every pose, which is the whole of what Λ buys.
    K_P, K_D = 600.0, 50.0
    K_P_NULL, K_D_NULL = 12.0, 7.0

    def __init__(self, robot, dt, variant=None):
        self.robot = robot
        self.controller = OperationalSpaceControl(
            robot, K_p=self.K_P, K_d=self.K_D,
            K_p_null=self.K_P_NULL, K_d_null=self.K_D_NULL,
            use_nullspace=variant != "n",
            use_lambda=variant != "l",
            use_bias=variant != "h",
        )

    def step(self, q_d, qd_d, qdd_d):
        # q̈_d is dropped: turning it into a task acceleration needs the J̇ q̇
        # term, which this rung does not carry.
        return self.controller.update(q_d, qd_d)

    def reset(self):
        self.controller.reset()

    def set_scale(self, nm_per_metre):
        self.controller.nm_per_metre = nm_per_metre


# --- the catalogue --------------------------------------------------------

SCENARIOS = {run.id: run
             for run in (GravityRun, PIDRun, ComputedTorqueRun, OSCRun)}


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
            "equation": run.equation,
            "variants": run.variants,
            "available": True,
        }
        for run in SCENARIOS.values()
    ]
    listed += [
        {"id": "impedance", "title": "Impedance control", "equation": [],
         "variants": [], "available": False},
    ]
    return listed
