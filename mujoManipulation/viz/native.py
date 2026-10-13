"""Native renderer — turns draw primitives into geometry in MuJoCo's viewer.

This is the only file in `viz/` that imports mujoco. It owns `viewer.user_scn`,
a scratch scene the passive viewer draws on top of the model every frame, and
it rewrites that scene from scratch on every call to `draw()`.

Phase 3 adds a second renderer that consumes the *same* primitives and emits
Three.js geometry over a WebSocket. If a primitive can only be drawn here, it
does not belong in `primitives.py`.
"""

import mujoco
import numpy as np

from .primitives import Arrow, Ellipsoid, Frame, GhostPath

_ZERO3 = np.zeros(3)
_EYE9 = np.eye(3).flatten()
_EPS = 1e-9

_AXIS_RGBA = (
    (0.95, 0.25, 0.2, 0.9),   # x
    (0.3, 0.8, 0.35, 0.9),    # y
    (0.3, 0.55, 0.95, 0.9),   # z
)


class NativeRenderer:
    """Draws primitives into a `mjvScene` — in practice `viewer.user_scn`.

    Silently stops adding geometry once the scene is full rather than raising:
    a visualisation that quietly loses its last arrow is better than a control
    loop that dies mid-run.
    """

    def __init__(self, scene: mujoco.MjvScene):
        self.scene = scene
        self.dropped = 0

    def draw(self, primitives) -> None:
        self.scene.ngeom = 0
        self.dropped = 0
        for p in primitives:
            self._add(p)

    # --- dispatch ---------------------------------------------------------

    def _add(self, p) -> None:
        if isinstance(p, Arrow):
            self._arrow(p.origin, p.vec, p.width, p.rgba)
        elif isinstance(p, Frame):
            mat = np.asarray(p.mat, dtype=float).reshape(3, 3)
            for axis in range(3):
                self._arrow(p.pos, mat[:, axis] * p.length, p.width, _AXIS_RGBA[axis])
        elif isinstance(p, Ellipsoid):
            self._ellipsoid(p)
        elif isinstance(p, GhostPath):
            pts = np.asarray(p.points, dtype=float).reshape(-1, 3)
            for a, b in zip(pts[:-1], pts[1:]):
                self._connector(mujoco.mjtGeom.mjGEOM_CAPSULE, p.width, a, b, p.rgba)
        else:
            raise TypeError(f"unknown draw primitive: {type(p).__name__}")

    # --- geometry ---------------------------------------------------------

    def _next(self):
        if self.scene.ngeom >= self.scene.maxgeom:
            self.dropped += 1
            return None
        geom = self.scene.geoms[self.scene.ngeom]
        self.scene.ngeom += 1
        return geom

    def _arrow(self, origin, vec, width, rgba) -> None:
        origin = np.asarray(origin, dtype=float)
        vec = np.asarray(vec, dtype=float)
        if np.linalg.norm(vec) < _EPS:
            return  # a zero-length arrow renders as a degenerate blob
        self._connector(mujoco.mjtGeom.mjGEOM_ARROW, width, origin, origin + vec, rgba)

    def _connector(self, gtype, width, start, end, rgba) -> None:
        geom = self._next()
        if geom is None:
            return
        mujoco.mjv_initGeom(
            geom, gtype, _ZERO3, _ZERO3, _EYE9, np.asarray(rgba, dtype=np.float32)
        )
        mujoco.mjv_connector(
            geom,
            gtype,
            width,
            np.asarray(start, dtype=float),
            np.asarray(end, dtype=float),
        )

    def _ellipsoid(self, p: Ellipsoid) -> None:
        geom = self._next()
        if geom is None:
            return
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_ELLIPSOID,
            np.asarray(p.radii, dtype=float),
            np.asarray(p.center, dtype=float),
            np.asarray(p.mat, dtype=float).flatten(),
            np.asarray(p.rgba, dtype=np.float32),
        )
