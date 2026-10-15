"""What the server can be asked to run, and how each run describes itself.

The menu in the browser is generated, not written. Nothing in `web/` knows that
a PID has an integral or that computed torque has a mass matrix: each scenario
hands over its own title, its own equation with the colour of every term, its
own switchable terms and its own gains, and the client renders whatever arrives.
Adding a rung should mean adding an entry here and touching no JavaScript at all
— which is what adding OSC and impedance actually cost.

Every term is its own switch, not one choice from a list. The controllers were
always built that way — `use_inertia`, `use_coriolis` and `use_gravity` are three
booleans, not an enum — and an equation whose terms are the buttons has to be
able to turn two of them off at once. `switches` covers things that are not
equation terms too, like the PID's anti-windup, and the client renders those as
their own chips.

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

Gains are declared the same way: a range, a step and a unit, so the client can
build a slider without knowing what it controls. Where the gains mean something
physical they are exposed as themselves — ω and ζ for computed torque, newtons
per metre for the two task-space rungs — and where they are seven hand-picked
numbers, as in the PID, the slider is a multiplier over all of them, because one
slider cannot honestly claim to be seven.
"""

import numpy as np

from ..controller import computed_torque as ct
from ..controller import gravity_comp as gc
from ..controller import impedance as imp_mod
from ..controller import osc as osc_mod
from ..controller import pid as pid_mod
from ..controller.computed_torque import ComputedTorque
from ..controller.gravity_comp import GravityCompensation
from ..controller.impedance import CartesianImpedance
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
    """τ = g(q). It has no gains at all, which is the point of it."""

    id = "gravity"
    title = "Gravity compensation"
    equation = [_plain("τ = "), _term("g(q)", gc.LOAD_RGBA, "g")]
    switches = [{"id": "g", "label": "g(q)"}]
    gains = []

    # Which attribute each switch owns. Flipping it live is why a term can
    # be turned off mid-motion without rebuilding the controller and losing
    # whatever state it had — a PID's integral, for one.
    SWITCH_ATTR = {"g": "enabled"}

    def __init__(self, robot, dt, off=()):
        self.robot = robot
        self.controller = GravityCompensation(robot, enabled="g" not in off)

    def step(self, q_d, qd_d, qdd_d):
        # Every argument ignored, which is the whole character of this rung: it
        # is told where the arm should be and has no way to act on it.
        return self.controller.update()

    def reset(self):
        pass

    def set_gain(self, gain_id, value):
        pass

    def gain_values(self):
        return {}

    def set_switch(self, switch_id, on):
        attr = self.SWITCH_ATTR.get(switch_id)
        if attr is not None:
            setattr(self.controller, attr, bool(on))

    def set_scale(self, nm_per_metre):
        self.controller.nm_per_metre = nm_per_metre


# --- rung 1 ---------------------------------------------------------------


class PIDRun:
    """τ = K_p e + K_i ∫e dt + K_d ė, on seven hand-picked pairs of gains."""

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
    # No switch for P: a PID without its proportional term is not a degradation
    # worth showing, it is a different and worse controller.
    switches = [
        {"id": "i", "label": "K_i ∫e dt"},
        {"id": "d", "label": "K_d ė"},
        {"id": "windup", "label": "anti-windup"},
    ]
    gains = [
        {"id": "kp", "label": "K_p", "min": 0.2, "max": 3.0, "step": 0.05, "unit": "x"},
        {"id": "ki", "label": "K_i", "min": 0.0, "max": 3.0, "step": 0.05, "unit": "x"},
        {"id": "kd", "label": "K_d", "min": 0.2, "max": 3.0, "step": 0.05, "unit": "x"},
    ]

    # examples/01's numbers, copied rather than imported so that each stays
    # readable on its own — and so the seven pairs sit where you can see them.
    K_P = np.array([120.0, 120.0, 120.0, 120.0, 50.0, 20.0, 10.0])
    K_D = np.array([26.0, 30.0, 24.0, 23.0, 9.0, 4.8, 2.5])
    K_I = np.array([185.0, 160.0, 195.0, 205.0, 95.0, 28.0, 14.0])

    # Which attribute each switch owns. Flipping it live is why a term can
    # be turned off mid-motion without rebuilding the controller and losing
    # whatever state it had — a PID's integral, for one.
    SWITCH_ATTR = {"i": "i_enabled", "d": "d_enabled", "windup": "anti_windup"}

    def __init__(self, robot, dt, off=()):
        self.robot = robot
        self.scale = {"kp": 1.0, "ki": 1.0, "kd": 1.0}
        self.controller = JointSpacePID(
            robot, K_p=self.K_P, K_i=self.K_I, K_d=self.K_D, dt=dt,
            d_enabled="d" not in off,
            i_enabled="i" not in off,
            anti_windup="windup" not in off,
        )

    def step(self, q_d, qd_d, qdd_d):
        # q̇_d goes in; q̈_d has nowhere to go, because this law has no term that
        # could carry it. That missing term is the next rung.
        return self.controller.update(q_d, qd_d)

    def reset(self):
        self.controller.reset()

    def set_gain(self, gain_id, value):
        """A multiplier over all seven, because one slider is not seven numbers
        and pretending otherwise would be the dishonest kind of simple."""
        if gain_id not in self.scale:
            return
        self.scale[gain_id] = float(value)
        self.controller.K_p = self.K_P * self.scale["kp"]
        self.controller.K_i = self.K_I * self.scale["ki"]
        self.controller.K_d = self.K_D * self.scale["kd"]

    def gain_values(self):
        return dict(self.scale)

    def set_switch(self, switch_id, on):
        attr = self.SWITCH_ATTR.get(switch_id)
        if attr is not None:
            setattr(self.controller, attr, bool(on))

    def set_scale(self, nm_per_metre):
        self.controller.nm_per_metre = nm_per_metre


# --- rung 2 ---------------------------------------------------------------


class ComputedTorqueRun:
    """τ = M(q) q̈_cmd + C(q, q̇) q̇ + g(q), with ω and ζ as the real gains.

    The equation is written with h(q, q̇) split into its two halves, because
    that is how the code computes it and how each half is switched off.
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
    switches = [
        {"id": "m", "label": "M(q)"},
        {"id": "c", "label": "C(q, q̇) q̇"},
        {"id": "g", "label": "g(q)"},
    ]
    # The one rung whose gains are a specification rather than a tuning: after
    # the cancellation every joint obeys ë + K_dė + K_pe = 0, so ω is how fast
    # an error dies and ζ is how it dies.
    gains = [
        {"id": "omega", "label": "ω", "min": 4.0, "max": 40.0, "step": 0.5,
         "unit": "rad/s"},
        {"id": "zeta", "label": "ζ", "min": 0.2, "max": 2.0, "step": 0.05, "unit": ""},
    ]

    OMEGA, ZETA = 20.0, 1.0

    # Which attribute each switch owns. Flipping it live is why a term can
    # be turned off mid-motion without rebuilding the controller and losing
    # whatever state it had — a PID's integral, for one.
    SWITCH_ATTR = {"m": "use_inertia", "c": "use_coriolis", "g": "use_gravity"}

    def __init__(self, robot, dt, off=()):
        self.robot = robot
        self.omega, self.zeta = self.OMEGA, self.ZETA
        self.controller = ComputedTorque(
            robot,
            K_p=self.omega**2,
            K_d=2.0 * self.zeta * self.omega,
            use_inertia="m" not in off,
            use_coriolis="c" not in off,
            use_gravity="g" not in off,
        )

    def step(self, q_d, qd_d, qdd_d):
        return self.controller.update(q_d, qd_d, qdd_d)

    def reset(self):
        self.controller.reset()

    def set_gain(self, gain_id, value):
        if gain_id == "omega":
            self.omega = float(value)
        elif gain_id == "zeta":
            self.zeta = float(value)
        else:
            return
        n = self.robot.n_arm
        self.controller.K_p = np.full(n, self.omega**2)
        self.controller.K_d = np.full(n, 2.0 * self.zeta * self.omega)

    def gain_values(self):
        return {"omega": self.omega, "zeta": self.zeta}

    def set_switch(self, switch_id, on):
        attr = self.SWITCH_ATTR.get(switch_id)
        if attr is not None:
            setattr(self.controller, attr, bool(on))

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
    switches = [
        {"id": "l", "label": "Λ(q)"},
        {"id": "h", "label": "h(q, q̇)"},
        {"id": "n", "label": "N τ₀"},
    ]
    # Task-space gains: K_p is the gripper's stiffness and means the same thing
    # at every pose, which is the whole of what Λ buys.
    gains = [
        {"id": "kp", "label": "K_p", "min": 50.0, "max": 3000.0, "step": 50.0,
         "unit": "N/m"},
        {"id": "kd", "label": "K_d", "min": 5.0, "max": 200.0, "step": 5.0,
         "unit": "N·s/m"},
        {"id": "kp_null", "label": "K_p posture", "min": 0.0, "max": 60.0,
         "step": 1.0, "unit": ""},
    ]

    K_P, K_D = 600.0, 50.0
    K_P_NULL, K_D_NULL = 12.0, 7.0

    # Which attribute each switch owns. Flipping it live is why a term can
    # be turned off mid-motion without rebuilding the controller and losing
    # whatever state it had — a PID's integral, for one.
    SWITCH_ATTR = {"l": "use_lambda", "h": "use_bias", "n": "use_nullspace"}

    def __init__(self, robot, dt, off=()):
        self.robot = robot
        self.values = {"kp": self.K_P, "kd": self.K_D, "kp_null": self.K_P_NULL}
        self.controller = OperationalSpaceControl(
            robot, K_p=self.K_P, K_d=self.K_D,
            K_p_null=self.K_P_NULL, K_d_null=self.K_D_NULL,
            use_nullspace="n" not in off,
            use_lambda="l" not in off,
            use_bias="h" not in off,
        )

    def step(self, q_d, qd_d, qdd_d):
        # q̈_d is dropped: turning it into a task acceleration needs the J̇ q̇
        # term, which this rung does not carry.
        return self.controller.update(q_d, qd_d)

    def reset(self):
        self.controller.reset()

    def set_gain(self, gain_id, value):
        if gain_id not in self.values:
            return
        self.values[gain_id] = float(value)
        self.controller.K_p = np.full(6, self.values["kp"])
        self.controller.K_d = np.full(6, self.values["kd"])
        self.controller.K_p_null = np.full(self.robot.n_arm, self.values["kp_null"])

    def gain_values(self):
        return dict(self.values)

    def set_switch(self, switch_id, on):
        attr = self.SWITCH_ATTR.get(switch_id)
        if attr is not None:
            setattr(self.controller, attr, bool(on))

    def set_scale(self, nm_per_metre):
        self.controller.nm_per_metre = nm_per_metre


# --- rung 4 ---------------------------------------------------------------


class ImpedanceRun:
    """τ = Jᵀ[K e + D ė] + h(q, q̇) + N τ₀ — a spring at the gripper.

    The only rung whose point is not accuracy, which makes it the only one that
    needs something to push against before it shows anything. `op: push` is that
    something.
    """

    id = "impedance"
    title = "Cartesian impedance"
    equation = [
        _plain("τ = Jᵀ["),
        _term("K e", imp_mod.SPRING_RGBA, "spring"),
        _plain(" + "),
        _term("D ė", imp_mod.DAMPER_RGBA, "d"),
        _plain("] + "),
        _term("h(q, q̇)", imp_mod.BIAS_RGBA, "h"),
        _plain(" + "),
        _term("N τ₀", imp_mod.NULLSPACE_RGBA, "n"),
    ]
    # No switch for the spring: without it there is no impedance, only a
    # damper, and that is not a lesson about impedance control.
    switches = [
        {"id": "d", "label": "D ė"},
        {"id": "h", "label": "h(q, q̇)"},
        {"id": "n", "label": "N τ₀"},
    ]
    # K is a compliance specification: push with F and the tip moves F/K.
    gains = [
        {"id": "k", "label": "K", "min": 50.0, "max": 3000.0, "step": 50.0,
         "unit": "N/m"},
        {"id": "d", "label": "D", "min": 5.0, "max": 200.0, "step": 5.0,
         "unit": "N·s/m"},
        {"id": "k_null", "label": "K posture", "min": 0.0, "max": 60.0,
         "step": 1.0, "unit": ""},
    ]

    K_P, K_D = 300.0, 35.0
    K_P_NULL, K_D_NULL = 12.0, 7.0

    # Which attribute each switch owns. Flipping it live is why a term can
    # be turned off mid-motion without rebuilding the controller and losing
    # whatever state it had — a PID's integral, for one.
    SWITCH_ATTR = {"d": "use_damping", "h": "use_bias", "n": "use_nullspace"}

    def __init__(self, robot, dt, off=()):
        self.robot = robot
        self.values = {"k": self.K_P, "d": self.K_D, "k_null": self.K_P_NULL}
        self.controller = CartesianImpedance(
            robot, K_p=self.K_P, K_d=self.K_D,
            K_p_null=self.K_P_NULL, K_d_null=self.K_D_NULL,
            use_damping="d" not in off,
            use_bias="h" not in off,
            use_nullspace="n" not in off,
        )

    def step(self, q_d, qd_d, qdd_d):
        return self.controller.update(q_d, qd_d)

    def reset(self):
        self.controller.reset()

    def set_gain(self, gain_id, value):
        if gain_id not in self.values:
            return
        self.values[gain_id] = float(value)
        self.controller.K_p = np.full(6, self.values["k"])
        self.controller.K_d = np.full(6, self.values["d"])
        self.controller.K_p_null = np.full(self.robot.n_arm, self.values["k_null"])

    def gain_values(self):
        return dict(self.values)

    def set_switch(self, switch_id, on):
        attr = self.SWITCH_ATTR.get(switch_id)
        if attr is not None:
            setattr(self.controller, attr, bool(on))

    def set_scale(self, nm_per_metre):
        self.controller.nm_per_metre = nm_per_metre


# --- the catalogue --------------------------------------------------------

SCENARIOS = {run.id: run
             for run in (GravityRun, PIDRun, ComputedTorqueRun, OSCRun,
                         ImpedanceRun)}
DEFAULT = ComputedTorqueRun.id          # the one that moves, so the pipe shows


def valid_off(run_id: str, off) -> frozenset:
    """The subset of `off` that this rung actually has a switch for.

    Rungs do not share terms — "no Coriolis" means nothing to a PID — and
    silently running the whole law while reporting a broken one is the single
    outcome worth ruling out.
    """
    run = SCENARIOS.get(run_id)
    if run is None:
        return frozenset()
    known = {s["id"] for s in run.switches}
    return frozenset(s for s in (off or ()) if s in known)


def catalogue() -> list:
    """Everything the controls need, in ladder order."""
    return [
        {
            "id": run.id,
            "title": run.title,
            "equation": run.equation,
            "switches": run.switches,
            "gains": run.gains,
            "available": True,
        }
        for run in SCENARIOS.values()
    ]
