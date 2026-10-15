// Boot: fetch the geometry once, fetch the menu, then follow the socket.
//
// The stream arrives at a fixed 60 Hz and the browser redraws whenever it feels
// like it, so the two are deliberately not coupled: the socket only ever stores
// the newest frame and requestAnimationFrame renders whatever is there. A
// dropped frame costs nothing and a slow tab cannot back up the server.

import { Viewer } from '/web/viewer.js';
import { Menu } from '/web/menu.js';

const el = (id) => document.getElementById(id);

const viewer = new Viewer(el('view'));
let socket = null;
let latest = null;          // newest state message, or null before the first
let applied = -1;           // sim time of the frame currently on screen
let framesSeen = 0;

// Commands are fire-and-forget. The server answers by changing what it streams,
// which is the only acknowledgement worth having: every frame carries the
// switch positions, so the menu reflects the simulation and not a local guess
// about what a click did.
function send(command) {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(command));
  }
}

const menu = new Menu(el('menu'), send);

async function loadScene() {
  const desc = await (await fetch('/scene')).json();
  const buffer = await (await fetch(desc.blob.url)).arrayBuffer();
  const n = viewer.setScene(desc, buffer);
  el('st-geoms').textContent = `${n} / ${desc.geoms.length}`;
  console.log(
    `scene: ${desc.model}, ${desc.geoms.length} geoms, ${desc.meshes.length} meshes, ` +
    `${(buffer.byteLength / 1e6).toFixed(2)} MB`,
  );
}

async function loadMenu() {
  menu.setCatalogue(await (await fetch('/menu')).json());
  menu.open();              // land on the menu, with the scene already behind it
}

function connect() {
  socket = new WebSocket(`ws://${location.host}/ws`);

  socket.onopen = () => {
    el('st-socket').textContent = 'open';
    el('st-socket').className = 'up';
  };
  socket.onmessage = (ev) => {
    latest = JSON.parse(ev.data);
    framesSeen++;
  };
  socket.onclose = () => {
    el('st-socket').textContent = 'closed — retrying';
    el('st-socket').className = 'down';
    setTimeout(connect, 1000);        // uvicorn restarts during development
  };
  socket.onerror = () => socket.close();
}

window.addEventListener('keydown', (e) => {
  if (e.key === 'Escape') menu.toggle();
  else if (e.key === 'r' || e.key === 'R') viewer.resetView();
});

// Frames per second as measured, not as hoped for.
setInterval(() => {
  el('st-fps').textContent = framesSeen.toString();
  framesSeen = 0;
}, 1000);

function describeRun(active) {
  if (!active || !active.scenario) return '—';
  const variant = active.variant ? ` · ${active.variant}` : '';
  const paused = active.paused ? ' · paused' : '';
  const speed = active.speed !== 1 ? ` · ${active.speed}x` : '';
  return `${active.scenario}${variant}${speed}${paused}`;
}

function tick() {
  if (latest && latest.t !== applied) {
    viewer.applyFrame(latest.xform);
    viewer.primitives.draw(latest.draw);
    el('st-time').textContent = `${latest.t.toFixed(2)} s`;
    el('st-run').textContent = describeRun(latest.active);
    // One primitive is often several meshes — an arrow is a shaft plus a cone —
    // so both numbers are worth seeing.
    const { count, dropped } = viewer.primitives;
    el('st-draw').textContent =
      `${latest.draw.length} prim / ${count} mesh` + (dropped ? ` (+${dropped} dropped)` : '');
    menu.setActive(latest.active);
    applied = latest.t;
  }
  viewer.render();
  requestAnimationFrame(tick);
}

Promise.all([loadScene(), loadMenu()]).then(connect).catch((e) => {
  el('st-socket').textContent = 'load failed';
  console.error(e);
});
requestAnimationFrame(tick);
