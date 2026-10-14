"""The static half of the protocol — what the browser needs once, not per frame.

A browser cannot load a MuJoCo model, so the server has to hand it the geometry.
The question is where that geometry comes from, and the answer is *not* the STL
files in `assets/`: they are 8.6 MB of Materialise-flavoured binary STL that
gzips only 2.1x, and nothing guarantees that a vertex in a `.stl` sits in the
frame MuJoCo's transforms apply to.

So the geometry is taken from the **compiled model** instead — `mesh_vert` and
`mesh_face`, after mjcf compilation. Those arrays are already deduplicated,
already recentred, and already expressed in exactly the frame that
`data.geom_xpos` / `data.geom_xmat` map to world. The browser can then be dumb:
it receives triangles once and a transform per geom per frame, and never has to
know what a body or a joint is.

    92,355 verts + 167,252 faces  ->  3.12 MB raw, 1.72 MB over gzip

Which geoms
    Group 1, which in this model is exactly the 12 visual meshes — the same set
    the native viewer shows with a group-1 mask. Group 0 holds the collision
    meshes (identical shapes, opaque grey, drawn on top of the visuals if you
    let them through) *and* the arena's ground plane, camera stand and mount.
    Those last ones are robosuite photography props, so the web viewer draws its
    own ground instead of shipping a procedural checker texture across the wire.

    Note `MjvGeom.dataid` is **not** a mesh id — mjv keeps two entries per mesh,
    so it comes back doubled. `model.geom_dataid` is the one you want. This file
    reads the model directly and never builds an MjvScene, which sidesteps that
    and the three site markers mjv would otherwise hand over as geometry.
"""

import os

import mujoco
import numpy as np

VISUAL_GROUP = 1

# MuJoCo geom types this protocol knows how to name. Anything else is sent with
# kind "unsupported" and skipped by the renderer rather than guessed at.
_KINDS = {
    mujoco.mjtGeom.mjGEOM_PLANE: "plane",
    mujoco.mjtGeom.mjGEOM_SPHERE: "sphere",
    mujoco.mjtGeom.mjGEOM_CAPSULE: "capsule",
    mujoco.mjtGeom.mjGEOM_ELLIPSOID: "ellipsoid",
    mujoco.mjtGeom.mjGEOM_CYLINDER: "cylinder",
    mujoco.mjtGeom.mjGEOM_BOX: "box",
    mujoco.mjtGeom.mjGEOM_MESH: "mesh",
}


class Scene:
    """The drawable geometry of a model, plus where it lives in one binary blob.

    Built once per model. `description()` is small enough to be JSON and is what
    the page fetches first; `blob()` is the geometry, fetched once after it.
    """

    def __init__(self, model: mujoco.MjModel, xml_path: str = ""):
        self.model = model
        self.xml_path = xml_path

        self.geom_ids = np.flatnonzero(model.geom_group == VISUAL_GROUP).astype(int)

        # Distinct meshes, in first-use order. Two geoms can share one mesh —
        # the two fingers do — so this is deduplicated and the geom entries
        # below index into it.
        self.mesh_ids: list[int] = []
        for geom_id in self.geom_ids:
            if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mesh_id = int(model.geom_dataid[geom_id])
            if mesh_id not in self.mesh_ids:
                self.mesh_ids.append(mesh_id)

        self._layout()
        self._blob: bytes | None = None

    # --- layout -----------------------------------------------------------

    def _layout(self) -> None:
        """Byte offsets of every mesh inside the blob.

        All vertices first, then all indices, so that both halves stay 4-byte
        aligned and the browser can take a typed-array view straight onto the
        ArrayBuffer with no copying.
        """
        m = self.model
        self.mesh_table = []
        vert_cursor = face_cursor = 0

        for mesh_id in self.mesh_ids:
            n_vert = int(m.mesh_vertnum[mesh_id])
            n_face = int(m.mesh_facenum[mesh_id])
            self.mesh_table.append(
                {
                    "verts": n_vert,
                    "faces": n_face,
                    "vert_offset": vert_cursor,
                    "face_offset": face_cursor,
                }
            )
            vert_cursor += n_vert * 3 * 4   # float32 xyz
            face_cursor += n_face * 3 * 4   # uint32 triangle

        self.vert_bytes = vert_cursor
        self.face_bytes = face_cursor

    # --- the two payloads --------------------------------------------------

    def description(self) -> dict:
        m = self.model
        geoms = []

        for geom_id in self.geom_ids:
            kind = _KINDS.get(mujoco.mjtGeom(m.geom_type[geom_id]), "unsupported")
            entry = {
                "name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)),
                "kind": kind,
                "size": m.geom_size[geom_id].tolist(),
                "rgba": m.geom_rgba[geom_id].tolist(),
            }
            if kind == "mesh":
                entry["mesh"] = self.mesh_ids.index(int(m.geom_dataid[geom_id]))
            geoms.append(entry)

        return {
            "type": "scene",
            "model": os.path.basename(self.xml_path),
            "timestep": float(m.opt.timestep),
            "gravity": m.opt.gravity.tolist(),
            "geoms": geoms,
            "meshes": self.mesh_table,
            "blob": {
                "url": "/scene.bin",
                "vert_bytes": self.vert_bytes,
                "face_bytes": self.face_bytes,
            },
        }

    def blob(self) -> bytes:
        """Vertices then indices, little-endian, built once and cached.

        Indices are local to their own mesh (MuJoCo stores them that way), so
        each mesh is independent and the browser needs no global rebasing.
        Normals are deliberately *not* shipped: MuJoCo indexes normals with a
        second face array that does not have to agree with the vertex one, so
        sending them means splitting vertices. Three.js computing them from the
        triangles costs one call and one frame, once.
        """
        if self._blob is not None:
            return self._blob

        m = self.model
        verts, faces = [], []
        for mesh_id in self.mesh_ids:
            va, vn = int(m.mesh_vertadr[mesh_id]), int(m.mesh_vertnum[mesh_id])
            fa, fn = int(m.mesh_faceadr[mesh_id]), int(m.mesh_facenum[mesh_id])
            verts.append(m.mesh_vert[va:va + vn].astype("<f4").ravel())
            faces.append(m.mesh_face[fa:fa + fn].astype("<u4").ravel())

        self._blob = b"".join(
            [np.concatenate(verts).tobytes(), np.concatenate(faces).tobytes()]
        )
        return self._blob

    # --- per-frame ---------------------------------------------------------

    def transforms(self, data: mujoco.MjData) -> list:
        """World pose of every drawable geom: 3 position + 9 rotation, flat.

        Row-major, which is what `Matrix4.set()` takes on the other end, so the
        browser does no conversion and no quaternion arithmetic. 12 geoms is
        144 floats a frame.
        """
        out = []
        for geom_id in self.geom_ids:
            out.extend(data.geom_xpos[geom_id].tolist())
            out.extend(data.geom_xmat[geom_id].tolist())
        return out
