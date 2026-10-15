// The menu, generated from what the server says it can run.
//
// Nothing in this file knows that a PID has an integral or that computed torque
// has a mass matrix. It reads /menu, renders whatever rungs are listed with
// whatever terms and break-it variants they declare, and sends back commands.
// Adding a rung is an entry in server/scenarios.py and no JavaScript.
//
// It is an overlay rather than a page because the alternative is re-fetching
// 1.72 MB of geometry and rebuilding the WebGL context on every choice — and
// because leaving the scene alive is what lets you switch from PID to computed
// torque *while the arm is moving*, which is the comparison the whole repo is
// about and the one thing a command line cannot do.

const el = (tag, className, text) => {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
};

export class Menu {
  constructor(root, send) {
    this.root = root;
    this.send = send;
    this.catalogue = [];
    this.active = {};
    this.visible = false;

    root.addEventListener('click', (e) => {
      if (e.target === root) this.close();     // click the backdrop to dismiss
    });
  }

  setCatalogue(data) {
    this.catalogue = data.scenarios || [];
    this.active = data.active || {};
    this.render();
  }

  // The server is the authority on what is running; every frame carries it, so
  // the menu follows the simulation rather than its own memory of a click.
  setActive(active) {
    if (!active) return;
    const changed = JSON.stringify(active) !== JSON.stringify(this.active);
    this.active = active;
    if (changed && this.visible) this.render();
  }

  open() { this.visible = true; this.root.hidden = false; this.render(); }
  close() { this.visible = false; this.root.hidden = true; }
  toggle() { this.visible ? this.close() : this.open(); }

  // --- rendering --------------------------------------------------------

  render() {
    if (!this.visible) return;
    this.root.replaceChildren(this.#panel());
  }

  #panel() {
    const panel = el('div', 'menu-panel');
    panel.appendChild(el('h2', null, 'control ladder'));
    for (const run of this.catalogue) panel.appendChild(this.#card(run));
    panel.appendChild(el('h2', null, 'view'));
    panel.appendChild(this.#viewSection());
    panel.appendChild(el('p', 'menu-hint',
      'Esc closes · R resets the camera · the simulation is shared by every tab'));
    return panel;
  }

  #card(run) {
    const isActive = run.id === this.active.scenario;
    const card = el('div', `card${isActive ? ' active' : ''}${run.available ? '' : ' soon'}`);

    const head = el('div', 'card-head');
    head.appendChild(el('span', 'dot'));
    head.appendChild(el('span', 'card-title', run.title));
    head.appendChild(el('span', 'card-sub', run.subtitle));
    card.appendChild(head);

    if (run.equation.length) {
      const eq = el('div', 'equation');
      for (const piece of run.equation) {
        const span = el('span', piece.term ? 'term' : null, piece.text);
        if (piece.rgba) span.style.color = rgbaCss(piece.rgba);
        eq.appendChild(span);
      }
      card.appendChild(eq);
    }

    if (run.variants.length) {
      const row = el('div', 'variants');
      for (const variant of run.variants) {
        const on = isActive && (this.active.variant ?? null) === (variant.id ?? null);
        const button = el('button', `variant${on ? ' on' : ''}`);
        button.appendChild(el('span', 'variant-label', variant.label));
        button.appendChild(el('span', 'variant-note', variant.note));
        button.onclick = () =>
          this.send({ op: 'select', scenario: run.id, variant: variant.id ?? null });
        row.appendChild(button);
      }
      card.appendChild(row);
    }

    // The reference only belongs to the rung that is running: offering to
    // change the reference of a controller that is not loaded would be a lie
    // about what the next click does.
    if (isActive && run.references.length > 1) {
      const row = el('div', 'refs');
      row.appendChild(el('span', 'refs-label', 'reference'));
      for (const ref of run.references) {
        const button = el('button',
          `chip${this.active.reference === ref.id ? ' on' : ''}`, ref.label);
        button.onclick = () => this.send({ op: 'reference', id: ref.id });
        row.appendChild(button);
      }
      card.appendChild(row);
    }
    return card;
  }

  #viewSection() {
    const box = el('div', 'view-section');

    box.appendChild(this.#slider(
      'arrow scale', this.active.scale ?? 40, 10, 200, 5, 'N·m per metre',
      (value) => this.send({ op: 'scale', value }),
      'how many newton-metres one metre of arrow is worth. The default 40 is ' +
      'shared with every rung so an arrow here compares with an arrow there.',
    ));
    box.appendChild(this.#slider(
      'speed', this.active.speed ?? 1, 0.05, 4, 0.05, 'x real time',
      (value) => this.send({ op: 'speed', value }),
      'slow it down to watch a term that only exists while the error moves.',
    ));

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

  #slider(label, value, min, max, step, unit, onInput, help) {
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
    if (help) box.appendChild(el('p', 'slider-help', help));
    return box;
  }
}

const fmt = (v) => (Number.isInteger(v) ? v.toString() : v.toFixed(2));
const rgbaCss = ([r, g, b, a = 1]) =>
  // The alpha a term's arrow is drawn with is often low enough to be unreadable
  // as text, so the colour carries over and the transparency does not.
  `rgba(${Math.round(r * 255)}, ${Math.round(g * 255)}, ${Math.round(b * 255)}, ` +
  `${Math.max(a, 0.85)})`;
