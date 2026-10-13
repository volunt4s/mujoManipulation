"""Draw primitives — plain data. No MuJoCo, no viewer, no rendering.

A controller returns a list of these alongside its torque, and whatever is on
the other end draws them: the native `user_scn` renderer in `viz/native.py`
today, the Three.js viewer in Phase 3 later. Nothing in this file imports
mujoco and nothing in it knows that a renderer exists — that separation is the
whole contract, and it is what lets one controller feed two renderers.

Coordinates are world frame, metres and radians.
"""

from dataclasses import dataclass
from typing import Union

import numpy as np

RGBA = tuple[float, float, float, float]


@dataclass
class Arrow:
    """Arrow starting at `origin`, pointing along `vec`, with length `|vec|`.

    Scale the vector yourself before constructing: a torque of 3.8 N·m has no
    natural length in metres, so the choice of "how long is one newton-metre"
    belongs to the controller that knows what it is drawing, not to the
    renderer.
    """

    origin: np.ndarray
    vec: np.ndarray
    width: float = 0.008
    rgba: RGBA = (0.95, 0.25, 0.2, 0.9)
    label: str = ""


@dataclass
class Frame:
    """Coordinate frame drawn as three arrows: x red, y green, z blue.

    `mat` is a 3x3 rotation whose *columns* are the frame axes in world
    coordinates — the same layout MuJoCo uses for `data.xmat` once reshaped.
    """

    pos: np.ndarray
    mat: np.ndarray
    length: float = 0.1
    width: float = 0.006
    label: str = ""


@dataclass
class Ellipsoid:
    """Ellipsoid with semi-axes `radii` along the columns of `mat`.

    Used for the manipulability ellipsoid in Phase 1: the shape the end
    effector can move in, drawn where the end effector is.
    """

    center: np.ndarray
    mat: np.ndarray
    radii: np.ndarray
    rgba: RGBA = (0.3, 0.55, 0.95, 0.35)
    label: str = ""


@dataclass
class GhostPath:
    """Polyline through `points`, drawn as connected segments.

    Phase 4 uses this for MPC rollouts — what the controller predicted, drawn
    next to what actually happened.
    """

    points: np.ndarray
    width: float = 0.004
    rgba: RGBA = (0.6, 0.6, 0.65, 0.5)
    label: str = ""


Primitive = Union[Arrow, Frame, Ellipsoid, GhostPath]
