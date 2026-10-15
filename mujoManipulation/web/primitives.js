// The web twin of viz/native.py — draw primitives into a Three.js scene.
//
// Same contract, same dispatch, same failure mode. A controller emits plain
// data (`viz/primitives.py`), two renderers consume it, and neither one is
// allowed to understand a primitive the other cannot draw. Reading this file
// next to native.py should feel like reading the same function twice.
//
// Three deliberate echoes of the Python side:
//
//   a zero-length arrow is skipped, because it renders as a degenerate blob
//   geometry past the cap is dropped and counted, never thrown, because a
//     visualisation quietly losing its last arrow beats a loop that dies
//   everything is positioned from world-frame metres, straight off the wire
//
// The one structural difference is pooling. MuJoCo hands us a pre-allocated
// `user_scn.geoms[]` and we fill it; here the objects are ours to keep, and
// allocating fifty meshes sixty times a second would hand the garbage
// collector a job it does not need. So each shape has a pool: taken in order
// every frame, grown on demand, and whatever is left over gets hidden.

import * as THREE from 'three';

// Three's cylinders, cones and capsules are all built along +Y, so +Y is the
// reference axis here and no geometry needs rotating at construction.
const Y_AXIS = new THREE.Vector3(0, 1, 0);

// Unit shapes, scaled per use. One geometry serves every width and length.
const UNIT_SHAFT = new THREE.CylinderGeometry(1, 1, 1, 16);
const UNIT_HEAD = new THREE.ConeGeometry(1, 1, 18);
const UNIT_BALL = new THREE.SphereGeometry(1, 24, 16);

// Arrow proportions. MuJoCo gives a quarter of the length to the head; the
// second term keeps a long thin arrow from growing a comical one.
const HEAD_FRACTION = 0.25;
const HEAD_MAX_WIDTHS = 10;
const HEAD_FLARE = 2.5;

// x red, y green, z blue — the same triple native.py uses for a Frame.
const AXIS_RGBA = [
  [0.95, 0.25, 0.2, 0.9],
  [0.3, 0.8, 0.35, 0.9],
  [0.3, 0.55, 0.95, 0.9],
];

const _a = new THREE.Vector3();
const _b = new THREE.Vector3();
const _dir = new THREE.Vector3();
const _span = new THREE.Vector3();   // #segment's own, so #arrow can keep _dir
const _mat = new THREE.Matrix4();

class Pool {
  constructor(scene, geometry, cap) {
    this.scene = scene;
    this.geometry = geometry;
    this.cap = cap;
    this.items = [];
    this.cursor = 0;
  }

  take() {
    if (this.cursor >= this.cap) return null;
    if (this.cursor === this.items.length) {
      const mesh = new THREE.Mesh(
        this.geometry,
        // Transparent from birth and forever: flipping `transparent` on a
        // live material recompiles its shader, and these alphas change every
        // frame. `depthWrite` is a render state rather than a shader define,
        // so that one is free to toggle.
        new THREE.MeshBasicMaterial({ transparent: true, depthWrite: false }),
      );
      mesh.frustumCulled = false;   // moved by hand every frame; let it draw
      this.scene.add(mesh);
      this.items.push(mesh);
    }
    return this.items[this.cursor++];
  }

  begin() {
    this.cursor = 0;
  }

  end() {
    for (let i = this.cursor; i < this.items.length; i++) {
      this.items[i].visible = false;
    }
  }
}

export class PrimitiveRenderer {
  constructor(scene, { cap = 512 } = {}) {
    this.scene = scene;
    this.dropped = 0;
    this.count = 0;
    this.shafts = new Pool(scene, UNIT_SHAFT, cap);
    this.heads = new Pool(scene, UNIT_HEAD, cap);
    this.balls = new Pool(scene, UNIT_BALL, 64);
  }

  draw(primitives) {
    this.dropped = 0;
    this.count = 0;
    this.shafts.begin();
    this.heads.begin();
    this.balls.begin();

    for (const p of primitives) this.#add(p);

    this.shafts.end();
    this.heads.end();
    this.balls.end();
  }

  // --- dispatch, in the same order as native.py -------------------------

  #add(p) {
    switch (p.kind) {
      case 'arrow':
        this.#arrow(p.origin, p.vec, p.width, p.rgba);
        break;
      case 'frame':
        // mat is row-major; its *columns* are the frame's axes, which is the
        // layout MuJoCo's xmat has once reshaped.
        for (let axis = 0; axis < 3; axis++) {
          const col = [p.mat[axis], p.mat[axis + 3], p.mat[axis + 6]];
          this.#arrow(
            p.pos,
            col.map((v) => v * p.length),
            p.width,
            AXIS_RGBA[axis],
          );
        }
        break;
      case 'ellipsoid':
        this.#ellipsoid(p);
        break;
      case 'path':
        for (let i = 0; i + 5 < p.points.length; i += 3) {
          _a.fromArray(p.points, i);
          _b.fromArray(p.points, i + 3);
          this.#segment(this.shafts, _a, _b, p.width, p.rgba);
        }
        break;
      default:
        console.warn(`primitives: unknown kind "${p.kind}"`);
    }
  }

  // --- geometry ---------------------------------------------------------

  #arrow(origin, vec, width, rgba) {
    const length = Math.hypot(vec[0], vec[1], vec[2]);
    if (length < 1e-9) return;          // a zero arrow is a blob, not an arrow

    const head = Math.min(HEAD_FRACTION * length, HEAD_MAX_WIDTHS * width);
    const shaft = length - head;

    _dir.fromArray(vec).divideScalar(length);
    _a.fromArray(origin);
    _b.copy(_a).addScaledVector(_dir, shaft);
    this.#segment(this.shafts, _a, _b, width, rgba);

    // The cone sits on the end of the shaft: centred half a head further on,
    // so its apex lands exactly at |vec|.
    const tip = this.heads.take();
    if (tip === null) {
      this.dropped++;
      return;
    }
    tip.position.copy(_b).addScaledVector(_dir, head / 2);
    tip.quaternion.setFromUnitVectors(Y_AXIS, _dir);
    tip.scale.set(HEAD_FLARE * width, head, HEAD_FLARE * width);
    this.#paint(tip, rgba);
  }

  // A unit +Y shape stretched to span `from` -> `to`. Arrow shafts and the
  // segments of a ghost path are the same problem, which is why native.py has
  // one `_connector` for both.
  #segment(pool, from, to, radius, rgba) {
    // Measure before taking: an object pulled from the pool and then left
    // unpainted keeps last frame's pose, because `end()` only hides the ones
    // past the cursor.
    _span.subVectors(to, from);
    const length = _span.length();
    if (length < 1e-9) return;

    const mesh = pool.take();
    if (mesh === null) {
      this.dropped++;
      return;
    }
    mesh.position.addVectors(from, to).multiplyScalar(0.5);
    mesh.quaternion.setFromUnitVectors(Y_AXIS, _span.divideScalar(length));
    mesh.scale.set(radius, length, radius);
    this.#paint(mesh, rgba);
  }

  #ellipsoid(p) {
    const mesh = this.balls.take();
    if (mesh === null) {
      this.dropped++;
      return;
    }
    const m = p.mat;
    _mat.set(m[0], m[1], m[2], 0, m[3], m[4], m[5], 0, m[6], m[7], m[8], 0, 0, 0, 0, 1);
    mesh.position.fromArray(p.center);
    mesh.quaternion.setFromRotationMatrix(_mat);
    mesh.scale.fromArray(p.radii);
    this.#paint(mesh, p.rgba);
  }

  #paint(mesh, rgba) {
    const [r, g, b, a = 1] = rgba;
    mesh.material.color.setRGB(r, g, b);
    mesh.material.opacity = a;
    // Nearly-opaque geometry writes depth so it occludes properly; the faint
    // wide arrows do not, so they blend instead of fighting over pixels.
    mesh.material.depthWrite = a > 0.98;
    mesh.visible = true;
    this.count++;
  }
}
