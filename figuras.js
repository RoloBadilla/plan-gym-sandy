/* Figuras animadas de cada ejercicio.
   Funcionan sin internet: son dibujos SVG que se mueven con JavaScript.

   Cómo está armado:
   - Cada pose dice el ángulo de cada parte del cuerpo (0 = derecha, 90 = abajo, -90 = arriba).
   - "anchor" fija un punto del cuerpo (por ejemplo, el tobillo) en un lugar del dibujo.
   - "ik" hace que una mano o un pie se quede pegado a un punto (por ejemplo, la mano en la silla).
   - Cada ejercicio tiene dos poses (A y B) y la figura va y viene entre ellas.
*/
(function (root) {
  const FLOOR = 192;
  const L = { torso: 50, head: 14, headR: 11, ua: 28, fa: 26, th: 40, sh: 38, ft: 13 };
  const COLOR = { near: '#4a4a4f', far: '#b3b3b8', hl: '#b5513f', hlFar: '#dca596', floor: '#d2d2d7', mat: '#ece4df', prop: '#c7c7cc', bottle: '#5b8fd1' };

  const rad = a => a * Math.PI / 180;
  const dir = a => [Math.cos(rad(a)), Math.sin(rad(a))];
  const add = (p, a, len) => { const d = dir(a); return [p[0] + d[0] * len, p[1] + d[1] * len]; };
  const rot = (v, a) => { const c = Math.cos(rad(a)), s = Math.sin(rad(a)); return [v[0] * c - v[1] * s, v[0] * s + v[1] * c]; };
  const deg = (x, y) => Math.atan2(y, x) * 180 / Math.PI;

  // Two-link IK: from start, reach target with segments a and b. bend = 1 or -1 picks the elbow/knee side.
  function ik(start, target, a, b, bend) {
    const dx = target[0] - start[0], dy = target[1] - start[1];
    let d = Math.hypot(dx, dy);
    const base = deg(dx, dy);
    d = Math.min(d, a + b - 0.01);
    const cosA = Math.max(-1, Math.min(1, (a * a + d * d - b * b) / (2 * a * d)));
    const A = Math.acos(cosA) * 180 / Math.PI;
    const a1 = base + bend * A;
    const mid = add(start, a1, a);
    const a2 = deg(target[0] - mid[0], target[1] - mid[1]);
    return [a1, a2];
  }

  // Compute every joint for a pose.
  function solve(p, view) {
    const hip0 = [0, 0];
    const td = dir(p.torso);
    const neck = add(hip0, p.torso, L.torso);
    const front = view === 'front';
    const perpL = rot(td, -90), perpR = rot(td, 90);
    const off = (pt, v, k) => [pt[0] + v[0] * k, pt[1] + v[1] * k];
    const sh1 = front ? off(neck, perpL, 13) : neck, sh2 = front ? off(neck, perpR, 13) : neck;
    const hp1 = front ? off(hip0, perpL, 8) : hip0, hp2 = front ? off(hip0, perpR, 8) : hip0;
    const build = (q) => {
      const J = { hip: hip0, neck, sh1, sh2, hp1, hp2 };
      J.head = add(neck, q.headA ?? q.torso, L.head);
      J.elbow1 = add(sh1, q.ua1, L.ua); J.hand1 = add(J.elbow1, q.fa1, L.fa);
      J.elbow2 = add(sh2, q.ua2, L.ua); J.hand2 = add(J.elbow2, q.fa2, L.fa);
      J.knee1 = add(hp1, q.th1, L.th); J.ankle1 = add(J.knee1, q.sh1, L.sh); J.toe1 = add(J.ankle1, q.ft1 ?? 0, L.ft);
      J.knee2 = add(hp2, q.th2, L.th); J.ankle2 = add(J.knee2, q.sh2, L.sh); J.toe2 = add(J.ankle2, q.ft2 ?? 0, L.ft);
      return J;
    };
    let J = build(p);
    const [an, ax, ay] = p.anchor;
    const shift = [ax - J[an][0], ay - J[an][1]];
    const mv = pt => [pt[0] + shift[0], pt[1] + shift[1]];
    for (const k in J) J[k] = mv(J[k]);
    if (p.ik) {
      const q = Object.assign({}, p);
      const S = { arm1: ['sh1', 'ua1', 'fa1', L.ua, L.fa], arm2: ['sh2', 'ua2', 'fa2', L.ua, L.fa], leg1: ['hp1', 'th1', 'sh1', L.th, L.sh], leg2: ['hp2', 'th2', 'sh2', L.th, L.sh] };
      for (const limb in p.ik) {
        const [from, k1, k2, a, b] = S[limb];
        let [tx, ty, bend] = p.ik[limb];
        if (typeof tx === 'string') { bend = ty; [tx, ty] = J[tx]; }
        const [a1, a2] = ik(J[from], [tx, ty], a, b, bend);
        q[k1] = a1; q[k2] = a2;
      }
      delete q.ik;
      // rebuild with solved angles, keep the same anchor point
      let J2 = build(q);
      const s2 = [J[an][0] - J2[an][0], J[an][1] - J2[an][1]];
      for (const k in J2) J2[k] = [J2[k][0] + s2[0], J2[k][1] + s2[1]];
      J = J2;
    }
    return J;
  }

  const ANG = ['torso', 'headA', 'ua1', 'fa1', 'ua2', 'fa2', 'th1', 'sh1', 'ft1', 'th2', 'sh2', 'ft2'];
  function lerpPose(A, B, t) {
    const o = { anchor: [A.anchor[0], A.anchor[1] + (B.anchor[1] - A.anchor[1]) * t, A.anchor[2] + (B.anchor[2] - A.anchor[2]) * t] };
    for (const k of ANG) if (A[k] !== undefined) o[k] = A[k] + ((B[k] ?? A[k]) - A[k]) * t;
    if (A.ik) {
      o.ik = {};
      for (const l in A.ik) { const a = A.ik[l], b = (B.ik && B.ik[l]) || a; o.ik[l] = typeof a[0] === 'string' ? a : [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2]]; }
    }
    return o;
  }

  // ---------- Poses ----------
  const STAND = { torso: -90, ua1: 95, fa1: 88, ua2: 86, fa2: 92, th1: 90, sh1: 90, ft1: 0, th2: 91, sh2: 89, ft2: 0 };
  const P = (base, extra) => Object.assign({}, base, extra);
  const SQUAT = { torso: -55, ua1: 0, fa1: 0, ua2: 4, fa2: 2, th1: 8, sh1: 112, ft1: 0, th2: 12, sh2: 110, ft2: 0 };
  const FSTAND = { torso: -90, ua1: 100, fa1: 95, ua2: 80, fa2: 85, th1: 96, sh1: 92, ft1: 180, th2: 84, sh2: 88, ft2: 0 };

  const EXFIG = {
    squat: { hl: ['th', 'glute'], k: [P(STAND, { anchor: ['ankle1', 150, 186] }), P(SQUAT, { anchor: ['ankle1', 150, 186] })] },
    sumo: { view: 'front', hl: ['th', 'glute'], k: [
      P(FSTAND, { th1: 108, sh1: 98, th2: 72, sh2: 82, ua1: 70, fa1: -40, ua2: 110, fa2: 220, anchor: ['ankle1', 128, 186] }),
      P(FSTAND, { th1: 152, sh1: 100, th2: 28, sh2: 80, ua1: 70, fa1: -40, ua2: 110, fa2: 220, anchor: ['ankle1', 128, 186] })] },
    bridge: { props: ['mat'], hl: ['glute', 'th'], k: [
      { torso: 180, headA: 180, ua1: 2, fa1: 0, ua2: 0, fa2: 0, th1: -50, sh1: 65, ft1: 0, th2: -46, sh2: 68, ft2: 0, anchor: ['ankle1', 192, 184] },
      { torso: 154, headA: 178, ua1: 6, fa1: 0, ua2: 4, fa2: 0, th1: -15, sh1: 92, ft1: 0, th2: -12, sh2: 94, ft2: 0, anchor: ['ankle1', 192, 184] }] },
    bridge1: { props: ['mat'], hl: ['glute', 'th'], k: [
      { torso: 180, headA: 180, ua1: 2, fa1: 0, ua2: 0, fa2: 0, th1: -32, sh1: -32, ft1: -30, th2: -50, sh2: 65, ft2: 0, anchor: ['ankle2', 192, 184] },
      { torso: 154, headA: 178, ua1: 6, fa1: 0, ua2: 4, fa2: 0, th1: -18, sh1: -18, ft1: -18, th2: -15, sh2: 92, ft2: 0, anchor: ['ankle2', 192, 184] }] },
    rlunge: { hl: ['th', 'glute'], k: [
      P(STAND, { anchor: ['ankle1', 165, 186] }),
      P(STAND, { torso: -88, th1: 5, sh1: 95, th2: 115, sh2: 176, ft2: 40, anchor: ['ankle1', 165, 186] })] },
    kick: { props: ['mat'], hl: ['glute', 'th1'], k: [
      { torso: -14, ua1: 90, fa1: 90, ua2: 91, fa2: 90, th1: 92, sh1: 180, ft1: 180, th2: 90, sh2: 180, ft2: 180, anchor: ['knee2', 120, 187] },
      { torso: -14, ua1: 90, fa1: 90, ua2: 91, fa2: 90, th1: 200, sh1: -90, ft1: 180, th2: 90, sh2: 180, ft2: 180, anchor: ['knee2', 120, 187] }] },
    plank: { props: ['mat'], hl: ['torso'], k: [
      { torso: -10, ua1: 90, fa1: 0, ua2: 88, fa2: 0, th1: 170, sh1: 170, ft1: 95, th2: 170, sh2: 170, ft2: 95, anchor: ['elbow1', 185, 185] },
      { torso: -11, ua1: 90, fa1: 0, ua2: 88, fa2: 0, th1: 169, sh1: 169, ft1: 95, th2: 169, sh2: 169, ft2: 95, anchor: ['elbow1', 185, 185] }] },
    pushup: { props: ['mat'], hl: ['torso', 'ua'], k: [
      { torso: -36, th1: 144, sh1: 205, ft1: 180, th2: 144, sh2: 205, ft2: 180, ua1: 90, fa1: 90, ua2: 90, fa2: 90, ik: { arm1: [194, 188, 1], arm2: [198, 188, 1] }, anchor: ['knee1', 117, 187] },
      { torso: -10, th1: 170, sh1: 205, ft1: 180, th2: 170, sh2: 205, ft2: 180, ua1: 90, fa1: 90, ua2: 90, fa2: 90, ik: { arm1: [194, 188, 1], arm2: [198, 188, 1] }, anchor: ['knee1', 117, 187] }] },
    dips: { props: ['chair'], hl: ['ua'], k: [
      { torso: -95, ua1: 90, fa1: 90, ua2: 90, fa2: 90, th1: 0, sh1: 90, th2: 0, sh2: 90, ft1: 0, ft2: 0, ik: { arm1: [118, 140, 1], arm2: [114, 140, 1], leg1: [172, 186, -1], leg2: [176, 186, -1] }, anchor: ['hip', 128, 138] },
      { torso: -95, ua1: 90, fa1: 90, ua2: 90, fa2: 90, th1: 0, sh1: 90, th2: 0, sh2: 90, ft1: 0, ft2: 0, ik: { arm1: [118, 140, 1], arm2: [114, 140, 1], leg1: [172, 186, -1], leg2: [176, 186, -1] }, anchor: ['hip', 128, 166] }] },
    row: { props: ['bottles'], hl: ['torso', 'ua'], k: [
      { torso: -25, ua1: 90, fa1: 90, ua2: 92, fa2: 90, th1: 80, sh1: 100, ft1: 0, th2: 82, sh2: 98, ft2: 0, anchor: ['ankle1', 150, 186] },
      { torso: -25, ua1: 205, fa1: 92, ua2: 200, fa2: 90, th1: 80, sh1: 100, ft1: 0, th2: 82, sh2: 98, ft2: 0, anchor: ['ankle1', 150, 186] }] },
    lateral: { view: 'front', props: ['bottles'], hl: ['ua'], k: [
      P(FSTAND, { anchor: ['ankle1', 140, 186] }),
      P(FSTAND, { ua1: 182, fa1: 178, ua2: -2, fa2: 2, anchor: ['ankle1', 140, 186] })] },
    superman: { props: ['mat'], hl: ['torso', 'glute'], k: [
      { torso: 0, headA: 0, ua1: 0, fa1: 0, ua2: 2, fa2: 0, th1: 180, sh1: 180, ft1: 180, th2: 178, sh2: 180, ft2: 180, anchor: ['hip', 130, 183] },
      { torso: -12, headA: -14, ua1: -16, fa1: -16, ua2: -14, fa2: -14, th1: 194, sh1: 194, ft1: 194, th2: 192, sh2: 192, ft2: 192, anchor: ['hip', 130, 183] }] },
    deadbug: { props: ['mat'], hl: ['torso'], k: [
      { torso: 180, ua1: -88, fa1: -88, ua2: -92, fa2: -92, th1: -90, sh1: 0, ft1: -90, th2: -92, sh2: 2, ft2: -90, anchor: ['hip', 175, 183] },
      { torso: 180, ua1: -88, fa1: -88, ua2: 198, fa2: 194, th1: -12, sh1: -8, ft1: -80, th2: -92, sh2: 2, ft2: -90, anchor: ['hip', 175, 183] }] },
    sldl: { hl: ['th', 'glute'], k: [
      P(STAND, { anchor: ['ankle1', 165, 186] }),
      P(STAND, { torso: -8, ua1: 90, fa1: 90, ua2: 92, fa2: 90, th1: 95, sh1: 85, th2: 172, sh2: 172, ft2: 90, anchor: ['ankle1', 165, 186] })] },
    jump: { hl: ['th', 'glute'], k: [
      P(SQUAT, { ua1: 140, fa1: 140, ua2: 136, fa2: 136, anchor: ['ankle1', 150, 186] }),
      P(STAND, { ua1: -78, fa1: -82, ua2: -84, fa2: -86, ft1: 60, ft2: 62, anchor: ['ankle1', 150, 170] })] },
    climb: { props: ['mat'], hl: ['torso'], k: [
      { torso: -20, ua1: 90, fa1: 90, ua2: 92, fa2: 90, th1: 160, sh1: 160, ft1: 95, th2: 160, sh2: 160, ft2: 95, anchor: ['hand1', 205, 188] },
      { torso: -20, ua1: 90, fa1: 90, ua2: 92, fa2: 90, th1: 22, sh1: 158, ft1: 70, th2: 160, sh2: 160, ft2: 95, anchor: ['hand1', 205, 188] }] },
    bike: { props: ['mat'], hl: ['torso'], k: [
      { torso: 196, headA: 205, ua1: -70, fa1: 175, ua2: -66, fa2: 178, th1: -110, sh1: 0, ft1: -60, th2: -16, sh2: -16, ft2: -60, anchor: ['hip', 175, 183] },
      { torso: 198, headA: 207, ua1: -70, fa1: 175, ua2: -66, fa2: 178, th1: -16, sh1: -16, ft1: -60, th2: -110, sh2: 0, ft2: -60, anchor: ['hip', 175, 183] }] },
    sideplank: { props: ['mat'], hl: ['torso'], k: [
      { torso: -10, ua1: -90, fa1: -90, ua2: 92, fa2: 0, th1: 170, sh1: 170, ft1: 80, th2: 171, sh2: 171, ft2: 80, anchor: ['elbow2', 195, 186] },
      { torso: -11, ua1: -90, fa1: -90, ua2: 92, fa2: 0, th1: 169, sh1: 169, ft1: 80, th2: 170, sh2: 170, ft2: 80, anchor: ['elbow2', 195, 186] }] },
    // ----- Calentamiento -----
    march: { hl: ['th'], down: .55, up: .55, hold: .05, k: [
      P(STAND, { ua1: 130, fa1: 70, ua2: 50, fa2: 20, th1: 5, sh1: 95, th2: 90, sh2: 90, anchor: ['hip', 150, 108] }),
      P(STAND, { ua1: 50, fa1: 20, ua2: 130, fa2: 70, th1: 90, sh1: 90, th2: 5, sh2: 95, anchor: ['hip', 150, 108] })] },
    armcircle: { view: 'front', hl: ['ua'], loop: 2.2, k: [
      P(FSTAND, { ua1: 90, fa1: 90, ua2: 90, fa2: 90, anchor: ['ankle1', 140, 186] }),
      P(FSTAND, { ua1: 450, fa1: 450, ua2: -270, fa2: -270, anchor: ['ankle1', 140, 186] })] },
    halfsquat: { hl: ['th', 'glute'], down: 1.4, up: 1.2, k: [
      P(STAND, { ua1: 60, fa1: 40, ua2: 64, fa2: 44, anchor: ['ankle1', 150, 186] }),
      P(STAND, { torso: -70, ua1: 0, fa1: 0, ua2: 4, fa2: 2, th1: 45, sh1: 108, th2: 48, sh2: 106, anchor: ['ankle1', 150, 186] })] },
    // ----- Estiramiento -----
    quad: { hl: ['th1'], down: 2.2, up: 2.2, hold: .6, k: [
      P(STAND, { ua2: 0, fa2: -6, th1: 100, sh1: -118, ft1: 150, ik: { arm1: ['ankle1', -1] }, anchor: ['ankle2', 150, 186] }),
      P(STAND, { ua2: 0, fa2: -6, th1: 104, sh1: -122, ft1: 150, ik: { arm1: ['ankle1', -1] }, anchor: ['ankle2', 150, 186] })] },
    ham: { props: ['mat'], hl: ['th', 'sh'], down: 2.4, up: 2.4, hold: .6, k: [
      { torso: -62, ua1: 25, fa1: 20, ua2: 28, fa2: 22, th1: 0, sh1: 0, ft1: -80, th2: -1, sh2: 0, ft2: -80, anchor: ['hip', 105, 183] },
      { torso: -32, headA: -24, ua1: 8, fa1: 4, ua2: 10, fa2: 6, th1: 0, sh1: 0, ft1: -80, th2: -1, sh2: 0, ft2: -80, anchor: ['hip', 105, 183] }] },
    fig4: { props: ['mat'], hl: ['glute', 'th1'], down: 2.4, up: 2.4, hold: .6, k: [
      { torso: 180, headA: 180, th2: -100, sh2: -8, ft2: -80, th1: -30, sh1: 172, ft1: 170, ua1: -60, fa1: -40, ua2: -60, fa2: -40, ik: { arm1: ['knee2', 1], arm2: ['knee2', 1] }, anchor: ['hip', 175, 183] },
      { torso: 180, headA: 180, th2: -118, sh2: -12, ft2: -80, th1: -46, sh1: 168, ft1: 170, ua1: -60, fa1: -40, ua2: -60, fa2: -40, ik: { arm1: ['knee2', 1], arm2: ['knee2', 1] }, anchor: ['hip', 175, 183] }] },
    child: { props: ['mat'], hl: ['torso'], down: 2.6, up: 2.6, hold: .4, k: [
      { torso: 14, headA: 22, ua1: 4, fa1: 0, ua2: 6, fa2: 0, th1: 36, sh1: 180, ft1: 180, th2: 36, sh2: 180, ft2: 180, anchor: ['knee1', 140, 187] },
      { torso: 18, headA: 24, ua1: 6, fa1: 2, ua2: 8, fa2: 2, th1: 36, sh1: 180, ft1: 180, th2: 36, sh2: 180, ft2: 180, anchor: ['knee1', 140, 187] }] }
  };

  // ---------- Drawing ----------
  const NS = 'http://www.w3.org/2000/svg';
  function el(tag, attrs) { const e = (root.document || null) ? root.document.createElementNS(NS, tag) : { tag, attrs: {}, setAttribute(k, v) { this.attrs[k] = v; } }; for (const k in attrs) e.setAttribute(k, attrs[k]); return e; }

  function segColor(def, name, far) {
    const hl = def.hl || [];
    const on = hl.includes(name) || hl.includes(name + (far ? '2' : '1'));
    return on ? (far ? COLOR.hlFar : COLOR.hl) : (far ? COLOR.far : COLOR.near);
  }

  // Returns a list of draw items for a set of joints (used by both browser and preview script)
  function items(def, J) {
    const front = def.view === 'front';
    const W = front ? { torso: 22, ua: 9, fa: 8, th: 12, sh: 10, ft: 6 } : { torso: 17, ua: 9, fa: 8, th: 12, sh: 10, ft: 6 };
    const out = [];
    const line = (a, b, w, c) => out.push({ t: 'line', a, b, w, c });
    // far side first
    const far = !front;
    line(J.sh2, J.elbow2, W.ua, segColor(def, 'ua', far)); line(J.elbow2, J.hand2, W.fa, segColor(def, 'fa', far));
    line(J.hp2, J.knee2, W.th, segColor(def, 'th', far)); line(J.knee2, J.ankle2, W.sh, segColor(def, 'sh', far)); line(J.ankle2, J.toe2, W.ft, segColor(def, 'ft', far));
    // torso + glute
    line(J.hip, J.neck, W.torso, segColor(def, 'torso', false));
    if ((def.hl || []).includes('glute')) {
      const td = dir(def._torso);
      const back = front ? [0, 0] : rot(td, -90);
      const g = [J.hip[0] + back[0] * 4 + td[0] * 3, J.hip[1] + back[1] * 4 + td[1] * 3];
      out.push({ t: 'circle', p: g, r: front ? 9 : 9.5, c: COLOR.hl });
    }
    // near side
    line(J.hp1, J.knee1, W.th, segColor(def, 'th', false)); line(J.knee1, J.ankle1, W.sh, segColor(def, 'sh', false)); line(J.ankle1, J.toe1, W.ft, segColor(def, 'ft', false));
    line(J.sh1, J.elbow1, W.ua, segColor(def, 'ua', false)); line(J.elbow1, J.hand1, W.fa, segColor(def, 'fa', false));
    out.push({ t: 'circle', p: J.head, r: L.headR, c: COLOR.near });
    if ((def.props || []).includes('bottles')) {
      for (const h of [J.hand2, J.hand1]) out.push({ t: 'bottle', p: h });
    }
    return out;
  }

  function staticSvg(def) {
    const p = [];
    p.push(`<line x1="-200" y1="${FLOOR + 1}" x2="500" y2="${FLOOR + 1}" stroke="${COLOR.floor}" stroke-width="2" stroke-linecap="round"/>`);
    if ((def.props || []).includes('mat')) p.push(`<rect x="30" y="${FLOOR - 3}" width="240" height="4" rx="2" fill="${COLOR.mat}"/>`);
    if ((def.props || []).includes('chair')) p.push(`<g fill="none" stroke="${COLOR.prop}" stroke-width="5" stroke-linecap="round"><line x1="62" y1="143" x2="120" y2="143"/><line x1="66" y1="143" x2="66" y2="${FLOOR}"/><line x1="116" y1="143" x2="116" y2="${FLOOR}"/><line x1="66" y1="143" x2="66" y2="88"/></g>`);
    return p.join('');
  }

  function frameSvg(def, J) {
    return items(def, J).map(it => {
      if (it.t === 'line') return `<line x1="${it.a[0].toFixed(1)}" y1="${it.a[1].toFixed(1)}" x2="${it.b[0].toFixed(1)}" y2="${it.b[1].toFixed(1)}" stroke="${it.c}" stroke-width="${it.w}" stroke-linecap="round"/>`;
      if (it.t === 'circle') return `<circle cx="${it.p[0].toFixed(1)}" cy="${it.p[1].toFixed(1)}" r="${it.r}" fill="${it.c}"/>`;
      if (it.t === 'bottle') return `<rect x="${(it.p[0] - 4).toFixed(1)}" y="${(it.p[1] - 7).toFixed(1)}" width="8" height="15" rx="3" fill="${COLOR.bottle}"/>`;
      return '';
    }).join('');
  }

  const ease = t => t < .5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
  // Timeline: A -> B in 1.6 s, hold 0.4 s, B -> A in 1.0 s, hold 0.4 s
  function progressAt(ms, def) {
    if (def.loop) return (ms % (def.loop * 1000)) / (def.loop * 1000);
    const down = (def.down || 1.6) * 1000, hold = (def.hold ?? 0.4) * 1000, up = (def.up || 1.0) * 1000;
    const cyc = down + hold + up + hold, t = ms % cyc;
    if (t < down) return ease(t / down);
    if (t < down + hold) return 1;
    if (t < down + hold + up) return 1 - ease((t - down - hold) / up);
    return 0;
  }

  function poseAt(def, t) {
    const p = lerpPose(def.k[0], def.k[1], t);
    def._torso = p.torso;
    return solve(p, def.view);
  }

  // ---------- Browser player ----------
  const live = new Set();
  let raf = null, last = 0;
  function tick(now) {
    const dt = last ? Math.min(100, now - last) : 0; last = now;
    for (const f of live) {
      if (f.paused) continue;
      f.t += dt;
      f.g.innerHTML = frameSvg(f.def, poseAt(f.def, progressAt(f.t, f.def)));
    }
    if (live.size) raf = requestAnimationFrame(tick); else { raf = null; last = 0; }
  }
  const reduce = root.matchMedia ? root.matchMedia('(prefers-reduced-motion: reduce)').matches : false;
  const io = root.IntersectionObserver ? new IntersectionObserver(es => {
    for (const e of es) {
      const f = e.target._fig; if (!f) continue;
      if (e.isIntersecting && !reduce) live.add(f); else live.delete(f);
    }
    if (live.size && !raf) raf = requestAnimationFrame(tick);
  }, { rootMargin: '80px' }) : null;

  // Frame each exercise so the figure fills the box (same 300x210 shape, floor at the bottom)
  function fitBox(def) {
    let x0 = 1e9, x1 = -1e9, y0 = 1e9;
    for (let i = 0; i <= 10; i++) {
      const J = poseAt(def, i / 10);
      for (const k in J) { x0 = Math.min(x0, J[k][0]); x1 = Math.max(x1, J[k][0]); y0 = Math.min(y0, J[k][1]); }
    }
    if ((def.props || []).includes('chair')) { x0 = Math.min(x0, 60); y0 = Math.min(y0, 86); }
    const pad = 18, bottom = FLOOR + 10;
    let w = x1 - x0 + pad * 2, h = bottom - (y0 - pad);
    if (w / h < 300 / 210) w = h * 300 / 210; else h = w * 210 / 300;
    w = Math.max(w, 170); h = w * 210 / 300;
    const cx = (x0 + x1) / 2;
    return [cx - w / 2, bottom - h, w, h].map(v => v.toFixed(1)).join(' ');
  }

  const PAUSE = '<svg viewBox="0 0 24 24"><rect x="6" y="5" width="4" height="14" rx="1.2"/><rect x="14" y="5" width="4" height="14" rx="1.2"/></svg>';
  const PLAY = '<svg viewBox="0 0 24 24"><path d="M8 5.5v13a1 1 0 0 0 1.5.86l10.5-6.5a1 1 0 0 0 0-1.72L9.5 4.64A1 1 0 0 0 8 5.5z"/></svg>';
  function setPaused(f, v) {
    f.paused = v;
    f.btn.innerHTML = (v ? PLAY : PAUSE) + `<span>${v ? 'Reproducir' : 'Pausar'}</span>`;
    f.btn.setAttribute('aria-label', v ? 'Reproducir animación' : 'Pausar animación');
  }
  function mount(container, id) {
    const def = EXFIG[id];
    if (!def) { container.innerHTML = '<p class="fig-miss">Cierra y vuelve a abrir la app para ver el dibujo.</p>'; return; }
    container.innerHTML = `<svg viewBox="${fitBox(def)}" role="img" aria-label="Animación del ejercicio"><g>${staticSvg(def)}</g><g class="fig"></g></svg><button class="fig-btn" type="button"></button>`;
    const g = container.querySelector('.fig');
    g.innerHTML = frameSvg(def, poseAt(def, reduce ? 1 : 0));
    const f = { def, g, t: Math.random() * 300, paused: false, btn: container.querySelector('.fig-btn') };
    setPaused(f, reduce);
    f.btn.addEventListener('click', e => { e.stopPropagation(); setPaused(f, !f.paused); });
    container._fig = f;
    if (io) io.observe(container);
  }
  function unmountAll() { live.clear(); }

  const api = { mount, unmountAll, EXFIG, poseAt, frameSvg, staticSvg, fitBox, FLOOR };
  if (typeof module !== 'undefined' && module.exports) module.exports = api; else root.Figuras = api;
})(typeof window !== 'undefined' ? window : globalThis);
