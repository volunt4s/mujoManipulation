"""Every message that crosses the socket, in one file.

JSON, deliberately, for everything except the geometry blob. The per-frame
payload is around 400 numbers — 144 for the geom transforms, the rest draw
primitives — which is some 40 KB/s at 60 Hz and nowhere near worth packing into
a binary frame format. What it buys instead is that you can open the browser's
network tab, click a frame, and read the protocol. In a repo whose product is
being able to see what the controller is doing, that is the better trade; the
binary upgrade is a later optimisation and not an interesting one.

Direction matters for naming:

    server -> browser    "scene" (once, see scene.py), "state" (per frame)
    browser -> server    "command"

The draw primitives are the same plain dataclasses the native renderer eats
(`viz/primitives.py`), and `primitive_dict` below is the exact twin of
`viz/native.py::_add` — one isinstance dispatch, one entry per primitive type.
When a primitive is added to `viz/`, both renderers have to learn it, and that
is on purpose: a primitive only one of them can draw does not belong there.
"""

import numpy as np

from ..viz.primitives import Arrow, Ellipsoid, Frame, GhostPath


# Four decimals is a tenth of a millimetre on a position and six thousandths
# of a degree on a rotation entry, which is finer than anything a screen can
# show. The default repr sends eight significant figures of float noise and
# costs a third of the frame.
PLACES = 4


def _floats(a) -> list:
    return np.round(np.asarray(a, dtype=float).ravel(), PLACES).tolist()


def _label(p) -> dict:
    """Labels are usually empty, and an empty one is 12 bytes a primitive."""
    return {"label": p.label} if p.label else {}


def primitive_dict(p) -> dict:
    """One draw primitive as plain JSON-able data."""
    if isinstance(p, Arrow):
        return {
            "kind": "arrow",
            "origin": _floats(p.origin),
            "vec": _floats(p.vec),
            "width": float(p.width),
            "rgba": list(p.rgba),
            **_label(p),
        }
    if isinstance(p, Frame):
        return {
            "kind": "frame",
            "pos": _floats(p.pos),
            "mat": _floats(p.mat),
            "length": float(p.length),
            "width": float(p.width),
            **_label(p),
        }
    if isinstance(p, Ellipsoid):
        return {
            "kind": "ellipsoid",
            "center": _floats(p.center),
            "mat": _floats(p.mat),
            "radii": _floats(p.radii),
            "rgba": list(p.rgba),
            **_label(p),
        }
    if isinstance(p, GhostPath):
        return {
            "kind": "path",
            "points": _floats(p.points),
            "width": float(p.width),
            "rgba": list(p.rgba),
            **_label(p),
        }
    raise TypeError(f"unknown draw primitive: {type(p).__name__}")


def state_message(sim_time: float, transforms, primitives, channels=None) -> dict:
    """One frame: where the robot is, what the controller wants drawn, and the
    scalars worth plotting."""
    return {
        "type": "state",
        "t": round(float(sim_time), 6),
        "xform": [round(v, PLACES) for v in transforms],
        "draw": [primitive_dict(p) for p in primitives],
        "channels": channels or {},
    }
