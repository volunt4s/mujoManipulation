// Strip charts: what the arrows were doing, as numbers over time.
//
// Hand-drawn on a canvas rather than taken from a charting library, for two
// reasons that are the whole design:
//
// The axis is locked to a decade. Autoscaling is what a dashboard does, and it
// is exactly wrong here: turn the gain up, the error halves, the axis halves
// with it and the trace looks identical. The scale here steps through 1, 2, 5
// times a power of ten, rises the moment the data needs it and falls only after
// the data has stayed small for a while — so when the error drops, the trace
// visibly sinks to the floor and stays there.
//
// And every command gets a tick. The client knows when it sent one, so it can
// mark the instant: here is where the damper was switched off, and here is what
// the trace did about it. That mark is the difference between a plot you read
// and a plot you learn something from.
//
// Magnitudes, not signed values. The question in all three panels is "how big",
// and seven signed traces crossing zero is mush; a saturated actuator still
// reads, as a flat top against the limit.

const WINDOW_SECONDS = 10;
const RATE_HZ = 60;
const CAPACITY = WINDOW_SECONDS * RATE_HZ;

// 1, 2, 5 × a power of ten. Hysteresis keeps the axis from stepping back down
// the instant a transient passes.
const STEPS = [1, 2, 5];
const SHRINK_BELOW = 0.35;
const SHRINK_AFTER_SECONDS = 1.5;

const PANELS = [
  { key: 'error_deg', label: 'joint error', unit: '°', traces: 7, floor: 1 },
  { key: 'tip_mm', label: 'tip error', unit: 'mm', traces: 1, floor: 1 },
  { key: 'torque_nm', label: 'joint torque', unit: 'N·m', traces: 7, floor: 10 },
];

const MARK_RGBA = {
  term: 'rgba(240, 200, 90, 0.85)',
  gain: 'rgba(122, 162, 247, 0.85)',
  push: 'rgba(233, 112, 90, 0.9)',
  select: 'rgba(200, 205, 215, 0.6)',
};

const TRACE = 'rgba(150, 200, 255, 0.55)';
const TRACE_WORST = 'rgba(235, 245, 255, 0.95)';
const GRID = 'rgba(255, 255, 255, 0.07)';
const TEXT = 'rgba(139, 145, 156, 1)';

function niceAbove(value) {
  if (!(value > 0)) return null;
  const decade = Math.pow(10, Math.floor(Math.log10(value)));
  for (const step of STEPS) {
    if (step * decade >= value) return step * decade;
  }
  return 10 * decade;
}

function niceBelow(value) {
  // One stop down the 1-2-5 ladder from `value`.
  const decade = Math.pow(10, Math.floor(Math.log10(value * 0.999)));
  const mantissa = value / decade;
  if (mantissa > 5.001) return 5 * decade;
  if (mantissa > 2.001) return 2 * decade;
  if (mantissa > 1.001) return decade;
  return 5 * decade / 10;
}

class Panel {
  constructor(spec, root) {
    this.spec = spec;
    this.values = new Float32Array(CAPACITY * spec.traces);
    this.axisMax = spec.floor;
    this.smallSince = null;

    this.box = document.createElement('div');
    this.box.className = 'plot';
    this.head = document.createElement('div');
    this.head.className = 'plot-head';
    this.title = document.createElement('span');
    this.title.textContent = spec.label;
    this.readout = document.createElement('span');
    this.readout.className = 'plot-value';
    this.head.append(this.title, this.readout);

    this.canvas = document.createElement('canvas');
    this.ctx = this.canvas.getContext('2d');
    this.box.append(this.head, this.canvas);
    root.appendChild(this.box);
  }

  write(index, value) {
    const base = index * this.spec.traces;
    if (Array.isArray(value)) {
      for (let k = 0; k < this.spec.traces; k++) {
        this.values[base + k] = Math.abs(value[k] ?? 0);
      }
    } else {
      this.values[base] = Math.abs(value ?? 0);
    }
  }

  resize(dpr) {
    const w = Math.max(1, Math.round(this.canvas.clientWidth));
    const h = Math.max(1, Math.round(this.canvas.clientHeight));
    if (this.canvas.width !== w * dpr || this.canvas.height !== h * dpr) {
      this.canvas.width = w * dpr;
      this.canvas.height = h * dpr;
    }
    return [w, h];
  }
}

export class Plots {
  constructor(root) {
    this.root = root;
    this.times = new Float32Array(CAPACITY);
    this.cursor = 0;
    this.filled = 0;
    this.marks = [];
    this.panels = PANELS.map((spec) => new Panel(spec, root));
    this.visible = false;
  }

  // --- data in ----------------------------------------------------------

  push(t, channels) {
    this.times[this.cursor] = t;
    for (const panel of this.panels) {
      panel.write(this.cursor, channels[panel.spec.key]);
    }
    this.cursor = (this.cursor + 1) % CAPACITY;
    this.filled = Math.min(this.filled + 1, CAPACITY);
  }

  // `kind` is one of the keys in MARK_RGBA. `t` is simulation time, so the mark
  // sits where the command landed rather than where the wall clock was.
  mark(t, kind) {
    if (t === null || t === undefined) return;
    this.marks.push({ t, kind });
    if (this.marks.length > 200) this.marks.splice(0, this.marks.length - 200);
  }

  clear() {
    this.cursor = 0;
    this.filled = 0;
    this.marks.length = 0;
    for (const panel of this.panels) {
      panel.values.fill(0);
      panel.axisMax = panel.spec.floor;
      panel.smallSince = null;
    }
  }

  setVisible(visible) {
    this.visible = visible;
  }

  // --- drawing ----------------------------------------------------------

  render() {
    if (!this.visible || this.filled < 2) return;
    const dpr = Math.min(window.devicePixelRatio, 2);
    const newest = this.times[(this.cursor - 1 + CAPACITY) % CAPACITY];
    const oldest = newest - WINDOW_SECONDS;

    // Marks outside the window are gone for good.
    this.marks = this.marks.filter((m) => m.t >= oldest);

    for (const panel of this.panels) this.#drawPanel(panel, dpr, oldest, newest);
  }

  #peak(panel, oldest) {
    let peak = 0;
    for (let i = 0; i < this.filled; i++) {
      const index = (this.cursor - 1 - i + CAPACITY * 2) % CAPACITY;
      if (this.times[index] < oldest) break;
      const base = index * panel.spec.traces;
      for (let k = 0; k < panel.spec.traces; k++) {
        if (panel.values[base + k] > peak) peak = panel.values[base + k];
      }
    }
    return peak;
  }

  #updateAxis(panel, peak, now) {
    if (peak > panel.axisMax) {
      panel.axisMax = Math.max(niceAbove(peak), panel.spec.floor);
      panel.smallSince = null;
      return;
    }
    // Step down only after the data has stayed well inside the axis for a
    // while, so a single spike does not leave the scale stuck high and a dip
    // does not make it flicker.
    if (peak < SHRINK_BELOW * panel.axisMax && panel.axisMax > panel.spec.floor) {
      if (panel.smallSince === null) panel.smallSince = now;
      else if (now - panel.smallSince > SHRINK_AFTER_SECONDS) {
        // Straight to the scale the data wants, not one stop at a time: three
        // steps of 1.5 s each is fifteen seconds of a stale axis after a gain
        // has already visibly changed the trace.
        panel.axisMax = Math.max(niceAbove(peak), panel.spec.floor);
        panel.smallSince = null;
      }
    } else {
      panel.smallSince = null;
    }
  }

  #drawPanel(panel, dpr, oldest, newest) {
    const [w, h] = panel.resize(dpr);
    const ctx = panel.ctx;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const peak = this.#peak(panel, oldest);
    this.#updateAxis(panel, peak, newest);
    const top = panel.axisMax;

    panel.readout.textContent =
      `${format(peak)} / ${format(top)} ${panel.spec.unit}`;

    // Half and full scale, so the eye has something to measure against.
    ctx.strokeStyle = GRID;
    ctx.lineWidth = 1;
    for (const frac of [0, 0.5, 1]) {
      const y = Math.round(h - frac * (h - 1)) - 0.5;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    const x = (t) => ((t - oldest) / WINDOW_SECONDS) * w;
    const y = (v) => h - Math.min(v / top, 1) * (h - 1);

    for (const mark of this.marks) {
      ctx.strokeStyle = MARK_RGBA[mark.kind] || MARK_RGBA.select;
      ctx.beginPath();
      ctx.moveTo(Math.round(x(mark.t)) + 0.5, 0);
      ctx.lineTo(Math.round(x(mark.t)) + 0.5, h);
      ctx.stroke();
    }

    // Which trace owns the peak gets drawn last and brightest; the rest are a
    // band, which is what shows the joints pulling on each other.
    const traces = panel.spec.traces;
    let worst = 0;
    for (let k = 1; k < traces; k++) {
      const a = panel.values[((this.cursor - 1 + CAPACITY) % CAPACITY) * traces + k];
      const b = panel.values[((this.cursor - 1 + CAPACITY) % CAPACITY) * traces + worst];
      if (a > b) worst = k;
    }

    for (const k of [...Array(traces).keys()].filter((i) => i !== worst).concat(worst)) {
      ctx.strokeStyle = k === worst && traces > 1 ? TRACE_WORST : TRACE;
      ctx.lineWidth = k === worst && traces > 1 ? 1.3 : 0.9;
      ctx.beginPath();
      let started = false;
      for (let i = this.filled - 1; i >= 0; i--) {
        const index = (this.cursor - 1 - i + CAPACITY * 2) % CAPACITY;
        const t = this.times[index];
        if (t < oldest) continue;
        const px = x(t);
        const py = y(panel.values[index * traces + k]);
        if (started) ctx.lineTo(px, py);
        else { ctx.moveTo(px, py); started = true; }
      }
      ctx.stroke();
    }
  }
}

function format(v) {
  if (v >= 100) return v.toFixed(0);
  if (v >= 10) return v.toFixed(1);
  return v.toFixed(2);
}
