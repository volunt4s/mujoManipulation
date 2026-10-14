// Boot: fetch the geometry once, then follow the socket.
//
// The stream arrives at a fixed 60 Hz and the browser redraws whenever it
// feels like it, so the two are deliberately not coupled: the socket only ever
// stores the newest frame and requestAnimationFrame renders whatever is there.
// A dropped frame costs nothing and a slow tab cannot back up the server.

import { Viewer } from '/web/viewer.js';

const el = (id) => document.getElementById(id);

const viewer = new Viewer(el('view'));
let latest = null;          // newest state message, or null before the first
let applied = -1;           // sim time of the frame currently on screen
let framesSeen = 0;

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

function connect() {
  const ws = new WebSocket(`ws://${location.host}/ws`);

  ws.onopen = () => {
    el('st-socket').textContent = 'open';
    el('st-socket').className = 'up';
  };
  ws.onmessage = (ev) => {
    latest = JSON.parse(ev.data);
    framesSeen++;
  };
  ws.onclose = () => {
    el('st-socket').textContent = 'closed — retrying';
    el('st-socket').className = 'down';
    setTimeout(connect, 1000);        // uvicorn restarts during development
  };
  ws.onerror = () => ws.close();
}

// Frames per second as measured, not as hoped for.
setInterval(() => {
  el('st-fps').textContent = framesSeen.toString();
  framesSeen = 0;
}, 1000);

function tick() {
  if (latest && latest.t !== applied) {
    viewer.applyFrame(latest.xform);
    el('st-time').textContent = `${latest.t.toFixed(2)} s`;
    el('st-draw').textContent = latest.draw.length.toString();  // drawn in W2
    applied = latest.t;
  }
  viewer.render();
  requestAnimationFrame(tick);
}

loadScene().then(connect).catch((e) => {
  el('st-socket').textContent = 'scene failed';
  console.error(e);
});
requestAnimationFrame(tick);
