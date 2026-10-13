"""Visualisation: plain-data primitives plus the renderers that consume them.

Import primitives from here. Import `NativeRenderer` only from renderer-side
code — a controller that imports a renderer has broken the contract.
"""

from .primitives import Arrow, Ellipsoid, Frame, GhostPath, Primitive

__all__ = ["Arrow", "Ellipsoid", "Frame", "GhostPath", "Primitive"]
