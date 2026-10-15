// Boot: fetch the geometry once, fetch the menu, then follow the socket.
//
// The stream arrives at a fixed 60 Hz and the browser redraws whenever it feels
// like it, so the two are deliberately not coupled: the socket only ever stores
// the newest frame and requestAnimationFrame renders whatever is there. A
// dropped frame costs nothing and a slow tab cannot back up the server.

import { Viewer } from '/web/viewer.js';
import { Controls } from '/web/controls.js';
import { Plots } from '/web/plots.js';

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
  // Mark the instant on the plots. The client is the only thing that knows when
  // a command was sent, and "here is where the damper went off, and here is what
  // the trace did about it" is most of why the plots are worth having.
  const kind = { term: 'term', gain: 'gain', push: 'push' }[command.op];
  if (kind) plots.mark(latest && latest.t, kind);
  else if (command.op === 'reset') plots.clear();
  else if (command.op) plots.mark(latest && latest.t, 'select');
}

const controls = new Controls(el('controls'), send);
const plots = new Plots(el('plots'));

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

async function loadControls() {
  controls.setCatalogue(await (await fetch('/catalogue')).json());
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

el('plot-toggle').onclick = () => {
  const open = el('plots').classList.toggle('open');
  el('plot-toggle').classList.toggle('folded', !open);
  el('plot-toggle').querySelector('.caret').textContent = open ? '\u2304' : '\u203a';
  plots.setVisible(open);
  viewer.resize();              // the stage just changed height
};
el('plot-clear').onclick = () => plots.clear();

document.addEventListener('keydown', (e) => {
  // `code` rather than `key`, because a non-Latin keyboard layout reports a
  // different `key` for the letter but the same physical key.
  if (e.code === 'KeyR') viewer.resetView();
});

// Frames per second as measured, not as hoped for.
setInterval(() => {
  el('st-fps').textContent = framesSeen.toString();
  framesSeen = 0;
}, 1000);

function describeRun(active) {
  if (!active || !active.scenario) return '—';
  const off = (active.off || []).length ? ` · no ${active.off.join(', ')}` : '';
  const paused = active.paused ? ' · paused' : '';
  const speed = active.speed !== 1 ? ` · ${active.speed}x` : '';
  return `${active.scenario}${off}${speed}${paused}`;
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
    controls.setActive(latest.active);
    plots.push(latest.t, latest.channels);
    applied = latest.t;
  }
  viewer.render();
  plots.render();
  requestAnimationFrame(tick);
}

Promise.all([loadScene(), loadControls()]).then(connect).catch((e) => {
  el('st-socket').textContent = 'load failed';
  console.error(e);
});
requestAnimationFrame(tick);
