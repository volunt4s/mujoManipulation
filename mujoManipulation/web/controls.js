// The controls, generated from what the server says it can run.
//
// Nothing in this file knows that a PID has an integral or that computed torque
// has a mass matrix. It reads /catalogue, renders whatever rungs are listed with
// whatever switchable terms and gains they declare, and sends back commands.
// Adding a rung is an entry in server/scenarios.py and no JavaScript.
//
// It is a section of the page that is always there, not a panel that opens.
// Nothing is ever hidden, so nothing can get stuck shown: the first two attempts
// at this were an overlay, and both bugs were in the open/close state rather
// than in anything the controls do.
//
// The scene stays alive across every choice, which is the point — switching from
// PID to computed torque *while the arm is moving* is the comparison the whole
// repo is about, and the one thing a command line cannot show.

const el = (tag, className, text) => {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
};

export class Controls {
  constructor(root, send) {
    this.root = root;
    this.send = send;
    this.catalogue = [];
    this.active = {};
    // Which sections are folded away. Local to the client — it is a view
    // preference, not a fact about the simulation — and kept on the instance so
    // that it survives the re-render every state change triggers.
    this.collapsed = { goal: false, method: false, push: false, view: false };
  }

  setCatalogue(data) {
    this.catalogue = data.scenarios || [];
    this.goals = data.goals || [];
    this.motions = data.motions || [];
    this.durationRange = data.duration_range || [0.5, 6];
    this.active = data.active || {};
    this.render();
  }

  // The server is the authority on what is running; every frame carries it, so
  // the controls follow the simulation rather than their own memory of a click.
  // Re-rendering on every frame would fight the slider the user is dragging, so
  // it only happens when something actually changed.
  setActive(active) {
    if (!active) return;
    if (JSON.stringify(active) === JSON.stringify(this.active)) return;
    this.active = active;
    this.render();
  }

  // --- rendering --------------------------------------------------------

  render() {
    const cards = this.catalogue.map((run) => this.#card(run));
    this.root.replaceChildren(
      ...this.#section('goal', 'goal', [this.#goalSection()]),
      ...this.#section('method', 'control algorithm', cards),
      ...this.#section('push', 'disturbance', [this.#pushSection()]),
      ...this.#section('view', 'view', [this.#viewSection()]),
    );
  }

  #section(key, title, children) {
    const head = el('button', `section-head${this.collapsed[key] ? ' folded' : ''}`);
    head.appendChild(el('span', 'caret', this.collapsed[key] ? '\u203a' : '\u2304'));
    head.appendChild(el('span', null, title));
    head.onclick = () => {
      this.collapsed[key] = !this.collapsed[key];
      this.render();
    };
    if (this.collapsed[key]) return [head];
    const body = el('div', 'section-body');
    for (const child of children) body.appendChild(child);
    return [head, body];
  }

  // One goal, one profile, shared by every rung. The whole point of putting
  // them here and not on the cards is that the task stops changing when the
  // controller does.
  #goalSection() {
    const box = el('div', 'view-section');

    // Two named poses, not one: naming the start as well makes the reference a
    // function of the task alone, so the same task gives the same demand every
    // time and two controllers can be held against each other.
    box.appendChild(this.#chips('from', this.goals, this.active.start,
      (id) => this.send({ op: 'start', id })));
    box.appendChild(this.#chips('to', this.goals, this.active.goal,
      (id) => this.send({ op: 'goal', id })));
    box.appendChild(this.#chips('profile', this.motions, this.active.motion,
      (id) => this.send({ op: 'motion', id })));

    const [lo, hi] = this.durationRange;
    box.appendChild(this.#slider('duration', this.active.duration ?? 1, lo, hi, 0.1, 's',
      (value) => this.send({ op: 'duration', value })));
    return box;
  }

  // Something to push against. Impedance control is the one rung whose subject
  // is what happens when the world disagrees with the arm, and without a shove
  // it looks like a soft version of the rung before it.
  #pushSection() {
    const box = el('div', 'view-section');
    box.appendChild(this.#slider('force', this.active.push_newtons ?? 20, 0, 80, 5, 'N',
      (value) => this.send({ op: 'push', value })));

    const row = el('div', 'refs');
    row.appendChild(el('span', 'refs-label', 'shove the gripper for a second'));
    const chips = el('div', 'chip-row');
    for (const axis of ['x', 'y', 'z']) {
      for (const [sign, label] of [[false, `+${axis}`], [true, `\u2212${axis}`]]) {
        const button = el('button', 'chip', label);
        button.onclick = () => this.send({ op: 'push', axis, negative: sign });
        chips.appendChild(button);
      }
    }
    row.appendChild(chips);
    box.appendChild(row);
    return box;
  }

  #chips(label, items, selected, onPick) {
    const row = el('div', 'refs');
    row.appendChild(el('span', 'refs-label', label));
    const chips = el('div', 'chip-row');
    for (const item of items) {
      const button = el('button', `chip${item.id === selected ? ' on' : ''}`, item.label);
      button.onclick = () => onPick(item.id);
      chips.appendChild(button);
    }
    row.appendChild(chips);
    return row;
  }

  // Collapsed to a title and its equation until it is the one running. The
  // equation *is* the control panel: a term the rung can switch is a button, in
  // its own colour, struck through when it is off. Everything a rung can switch
  // that is not a term in the equation -- the PID's anti-windup -- becomes a
  // chip underneath, and the gains become sliders. None of it is written here;
  // all of it comes from server/scenarios.py.
  #card(run) {
    const isActive = run.id === this.active.scenario;
    const off = new Set(isActive ? this.active.off || [] : []);
    const switchable = new Set((run.switches || []).map((s) => s.id));
    const card = el('div', `card${isActive ? ' active' : ''}${run.available ? '' : ' soon'}`);

    const head = el('button', 'card-head');
    head.appendChild(el('span', 'dot'));
    head.appendChild(el('span', 'card-title', run.title));
    if (run.available) {
      head.onclick = () => this.send({ op: 'select', scenario: run.id, off: [] });
    } else {
      head.disabled = true;
    }
    card.appendChild(head);

    if (run.equation.length) {
      const eq = el('div', 'equation');
      for (const piece of run.equation) {
        const live = isActive && switchable.has(piece.term);
        const node = el(live ? 'button' : 'span',
          piece.term ? `term${off.has(piece.term) ? ' off' : ''}${live ? ' live' : ''}` : null,
          piece.text);
        if (piece.rgba) node.style.color = rgbaCss(piece.rgba);
        if (live) {
          node.title = off.has(piece.term) ? 'switch it back on' : 'switch it off';
          node.onclick = () =>
            this.send({ op: 'term', id: piece.term, on: off.has(piece.term) });
        }
        eq.appendChild(node);
      }
      card.appendChild(eq);
    }

    if (!isActive) return card;

    // Switches with no term of their own in the equation.
    const extra = (run.switches || []).filter(
      (s) => !run.equation.some((piece) => piece.term === s.id));
    if (extra.length) {
      const row = el('div', 'toggles');
      for (const s of extra) {
        const button = el('button', `chip${off.has(s.id) ? '' : ' on'}`, s.label);
        button.onclick = () => this.send({ op: 'term', id: s.id, on: off.has(s.id) });
        row.appendChild(button);
      }
      card.appendChild(row);
    }

    const values = this.active.gains || {};
    for (const gain of run.gains || []) {
      card.appendChild(this.#slider(
        gain.label, values[gain.id] ?? gain.min, gain.min, gain.max, gain.step,
        gain.unit, (value) => this.send({ op: 'gain', id: gain.id, value })));
    }
    if (!run.gains.length) {
      card.appendChild(el('p', 'no-gains', 'no gains — there is nothing to tune'));
    }
    return card;
  }

  #viewSection() {
    const box = el('div', 'view-section');

    box.appendChild(this.#slider(
      'arrow scale', this.active.scale ?? 40, 10, 200, 5, 'N·m per metre',
      (value) => this.send({ op: 'scale', value }),
    ));
    box.appendChild(this.#slider(
      'speed', this.active.speed ?? 1, 0.05, 4, 0.05, 'x real time',
      (value) => this.send({ op: 'speed', value }),
    ));

    const show = this.active.show || {};
    box.appendChild(this.#toggles([
      { id: 'ee', label: 'end-effector frame' },
      { id: 'goal', label: 'goal frame' },
    ], show));

    const row = el('div', 'buttons');
    const pause = el('button', 'chip', this.active.paused ? 'resume' : 'pause');
    pause.onclick = () => this.send({ op: 'pause', value: !this.active.paused });
    row.appendChild(pause);

    const stepOnce = el('button', 'chip', 'step 1');
    stepOnce.onclick = () => this.send({ op: 'step', n: 1 });
    row.appendChild(stepOnce);

    const reset = el('button', 'chip', 'reset the run');
    reset.onclick = () => this.send({ op: 'reset' });
    row.appendChild(reset);
    box.appendChild(row);

    return box;
  }

  #toggles(items, state) {
    const row = el('div', 'toggles');
    for (const item of items) {
      const button = el('button', `chip${state[item.id] ? ' on' : ''}`, item.label);
      button.onclick = () =>
        this.send({ op: 'show', id: item.id, value: !state[item.id] });
      row.appendChild(button);
    }
    return row;
  }

  #slider(label, value, min, max, step, unit, onInput) {
    const box = el('div', 'slider');
    const head = el('div', 'slider-head');
    head.appendChild(el('span', 'slider-label', label));
    const readout = el('span', 'slider-value', `${fmt(value)} ${unit}`);
    head.appendChild(readout);
    box.appendChild(head);

    const input = el('input');
    Object.assign(input, { type: 'range', min, max, step, value });
    input.oninput = () => {
      readout.textContent = `${fmt(+input.value)} ${unit}`;
      onInput(+input.value);
    };
    box.appendChild(input);
    return box;
  }
}

const fmt = (v) => (Number.isInteger(v) ? v.toString() : v.toFixed(2));
const rgbaCss = ([r, g, b, a = 1]) =>
  // The alpha a term's arrow is drawn with is often low enough to be unreadable
  // as text, so the colour carries over and the transparency does not.
  `rgba(${Math.round(r * 255)}, ${Math.round(g * 255)}, ${Math.round(b * 255)}, ` +
  `${Math.max(a, 0.85)})`;
