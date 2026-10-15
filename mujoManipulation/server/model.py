"""Loading the model without paying the compiler every time.

Measured on `franka_panda.xml`:

    compiling the XML      +283 MB of RSS, and the model keeps only 58.6 MB
    loading the compiled   +114 MB

The gap is mjcf compilation — convex hulls for eleven full-resolution collision
meshes and 603,214 BVH nodes — which the allocator never hands back to the OS.
It is transient work with a permanent bill, and on a laptop it is the difference
between a server that survives and one the kernel kills. (The real fix is
smaller collision geometry; Phase 2's move to mujoco_menagerie brings primitive
capsules instead of mesh copies of the visuals, which cuts the 58.6 MB too.)

So: compile once, keep the bytes, load those next time. `mj_saveModel` writes
MuJoCo's own binary format, and the round trip is exact — `tests` here being the
assertion in the module docstring of `__main__`, which checks a stroke
reproduces to the digit either way.

Staleness is handled by keying the cache on every asset the model could have
read: each XML, mesh and texture under the model's directory contributes its
size and mtime to a hash, and any change gives a different cache file. Deleting
the cache directory is always safe.
"""

import hashlib
import os

import mujoco

CACHE_DIRNAME = ".mjb-cache"
_ASSET_SUFFIXES = (".xml", ".stl", ".obj", ".msh", ".png", ".mtl", ".dae")


def _asset_key(xml_path: str) -> str:
    """A hash over everything the compiler might have looked at."""
    root = os.path.dirname(os.path.abspath(xml_path))
    # Walk up one level too: the arena lives in a sibling directory.
    root = os.path.dirname(root)
    digest = hashlib.sha1()
    for dirpath, _dirs, files in sorted(os.walk(root)):
        if CACHE_DIRNAME in dirpath:
            continue
        for name in sorted(files):
            if not name.lower().endswith(_ASSET_SUFFIXES):
                continue
            path = os.path.join(dirpath, name)
            try:
                st = os.stat(path)
            except OSError:
                continue
            digest.update(os.path.relpath(path, root).encode())
            digest.update(f"{st.st_size}:{st.st_mtime_ns}".encode())
    return digest.hexdigest()[:16]


def load_model(xml_path: str, use_cache: bool = True) -> mujoco.MjModel:
    """`MjModel` for `xml_path`, via a compiled cache when one is usable."""
    if not use_cache:
        return mujoco.MjModel.from_xml_path(xml_path)

    cache_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), CACHE_DIRNAME)
    stem = os.path.splitext(os.path.basename(xml_path))[0]
    cached = os.path.join(cache_dir, f"{stem}-{_asset_key(xml_path)}.mjb")

    if os.path.exists(cached):
        try:
            return mujoco.MjModel.from_binary_path(cached)
        except Exception:
            os.remove(cached)       # written by another mujoco version, say

    model = mujoco.MjModel.from_xml_path(xml_path)
    try:
        os.makedirs(cache_dir, exist_ok=True)
        # Write beside the target and rename, so a killed process cannot leave
        # a half-written model behind for the next one to load.
        partial = cached + f".{os.getpid()}.partial"
        mujoco.mj_saveModel(model, partial, None)
        os.replace(partial, cached)
    except OSError:
        pass                        # a read-only checkout is not a failure
    return model
