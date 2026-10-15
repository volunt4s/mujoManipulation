// The Three.js side of the native renderer's contract.
//
// This file does the part the native viewer gets for free: drawing the robot
// itself, which nobody does for us in a browser. The draw primitives on top of
// it are `primitives.js`, which is the actual twin of `viz/native.py`.
//
// The world is Z-up, because MuJoCo's is. Rather than rotating the scene into
// Three's Y-up convention — which quietly inverts every primitive we are about
// to draw — the camera is told that up is +Z and everything else stays in
// MuJoCo's frame. Vectors from the server can then be used verbatim.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { PrimitiveRenderer } from '/web/primitives.js';

const UP = new THREE.Vector3(0, 0, 1);

// The centre of what is actually drawn, measured rather than guessed: the
// visual geoms at the home pose span z = -0.008 .. 1.699, so their midpoint is
// 0.85 and not the world origin. Aiming at (0, 0, 0) would put the whole robot
// in the top half of the frame, because the arm stands on a pedestal.
const LOOK_AT = new THREE.Vector3(0, 0, 0.85);
// 2.9 m back fills about 70% of a 45-degree vertical fov with the 1.71 m of
// robot, which leaves room for the arrows without them running off screen.
const EYE = new THREE.Vector3(2.1, -2.1, 1.7);

export class Viewer {
  constructor(canvas) {
    this.canvas = canvas;
    this.geoms = [];          // one THREE.Mesh per drawable geom, server order

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x14161a);

    this.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
    this.camera.up.copy(UP);
    this.camera.position.copy(EYE);

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.copy(LOOK_AT);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.maxDistance = 12;
    this.controls.minDistance = 0.4;
    // Orbiting under the floor is never what anyone wanted.
    this.controls.maxPolarAngle = Math.PI * 0.49;
    this.controls.saveState();        // so resetView() has somewhere to go

    this.#lights();
    this.#ground();
    this.primitives = new PrimitiveRenderer(this.scene);

    // Measured from the stage, never from the canvas itself: the canvas's own
    // size is an output of this function, and reading it back would be a loop.
    // A ResizeObserver covers the sidebar changing width or the window
    // changing shape without either one having to know about the other.
    this.stage = canvas.parentElement;
    new ResizeObserver(() => this.resize()).observe(this.stage);
    this.resize();
  }

  #lights() {
    this.scene.add(new THREE.HemisphereLight(0xbfd4ff, 0x20242b, 1.5));
    const key = new THREE.DirectionalLight(0xffffff, 2.0);
    key.position.set(2.5, -2.0, 4.0);
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0xffffff, 0.5);
    fill.position.set(-2.5, 1.5, 1.0);
    this.scene.add(fill);
  }

  #ground() {
    // The model's own arena is a checkerboard plane plus a camera stand and
    // mount — robosuite photography props. Drawing our own floor is cheaper
    // than shipping a procedural texture, and less in the way.
    const grid = new THREE.GridHelper(8, 32, 0x2b2f37, 0x21242a);
    grid.rotation.x = Math.PI / 2;             // XZ plane -> XY, because Z is up
    this.scene.add(grid);

    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(8, 8),
      new THREE.MeshStandardMaterial({ color: 0x1a1d22, roughness: 0.95 }),
    );
    floor.position.z = -0.002;                 // under the grid, not through it
    this.scene.add(floor);
  }

  // --- geometry, once ---------------------------------------------------

  // `desc` is GET /scene, `buffer` is GET /scene.bin. Vertices come first in
  // the blob and indices after, both little-endian and 4-byte aligned, so
  // these are views onto the ArrayBuffer rather than copies of it.
  setScene(desc, buffer) {
    const vertBytes = desc.blob.vert_bytes;
    const geometries = desc.meshes.map((m) => {
      const position = new Float32Array(buffer, m.vert_offset, m.verts * 3);
      const index = new Uint32Array(buffer, vertBytes + m.face_offset, m.faces * 3);
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(position, 3));
      g.setIndex(new THREE.BufferAttribute(index, 1));
      // MuJoCo indexes normals with a second face array that need not agree
      // with the vertex one, so they are not shipped; one pass here is enough.
      g.computeVertexNormals();
      return g;
    });

    for (const spec of desc.geoms) {
      const mesh = this.#build(spec, geometries);
      if (!mesh) continue;
      mesh.matrixAutoUpdate = false;           // every pose comes from the wire
      this.scene.add(mesh);
      this.geoms.push(mesh);
    }
    return this.geoms.length;
  }

  #build(spec, geometries) {
    const [r, g, b, a] = spec.rgba;
    const material = new THREE.MeshStandardMaterial({
      color: new THREE.Color(r, g, b),
      transparent: a < 1,
      opacity: a,
      roughness: 0.5,
      metalness: 0.05,
    });
    const s = spec.size;

    switch (spec.kind) {
      case 'mesh':
        return new THREE.Mesh(geometries[spec.mesh], material);
      // MuJoCo's size conventions: half-extents for a box, radius for a
      // sphere, (radius, half-length) for a cylinder or capsule.
      case 'box':
        return new THREE.Mesh(
          new THREE.BoxGeometry(s[0] * 2, s[1] * 2, s[2] * 2), material);
      case 'sphere':
        return new THREE.Mesh(new THREE.SphereGeometry(s[0], 24, 16), material);
      case 'cylinder': {
        const cyl = new THREE.CylinderGeometry(s[0], s[0], s[1] * 2, 24);
        cyl.rotateX(Math.PI / 2);              // Three's cylinder is Y-axis
        return new THREE.Mesh(cyl, material);
      }
      case 'capsule': {
        const cap = new THREE.CapsuleGeometry(s[0], s[1] * 2, 8, 16);
        cap.rotateX(Math.PI / 2);
        return new THREE.Mesh(cap, material);
      }
      default:
        console.warn(`viewer: skipping geom "${spec.name}" of kind ${spec.kind}`);
        return null;
    }
  }

  // --- per frame --------------------------------------------------------

  // `xform` is 12 floats per geom: 3 position, then a row-major 3x3 rotation.
  // Matrix4.set() takes its arguments row-major too, so there is no conversion
  // here and no quaternion arithmetic anywhere in the client.
  applyFrame(xform) {
    for (let i = 0; i < this.geoms.length; i++) {
      const o = i * 12;
      if (o + 12 > xform.length) break;
      this.geoms[i].matrix.set(
        xform[o + 3], xform[o + 4], xform[o + 5], xform[o + 0],
        xform[o + 6], xform[o + 7], xform[o + 8], xform[o + 1],
        xform[o + 9], xform[o + 10], xform[o + 11], xform[o + 2],
        0, 0, 0, 1,
      );
      this.geoms[i].matrixWorldNeedsUpdate = true;
    }
  }

  // Orbit controls wander. One key to put the camera back where it started.
  resetView() {
    this.controls.reset();
    this.camera.position.copy(EYE);
    this.controls.target.copy(LOOK_AT);
    this.controls.update();
  }

  resize() {
    const box = this.stage.getBoundingClientRect();
    const w = Math.max(1, Math.round(box.width));
    const h = Math.max(1, Math.round(box.height));
    // updateStyle off, because the stylesheet already gives the canvas an
    // explicit 100% x 100% inside the stage. Letting three write inline pixels
    // too would pin the element to the size we just measured and turn the next
    // measurement into an echo of the last one.
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  render() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
