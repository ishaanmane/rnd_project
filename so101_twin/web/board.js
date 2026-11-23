// Sorting demo in the digital twin: the printed grid sheet, the blocks, and the plan.
//
// Where is the sheet? The taught grab poses say it: for every taught cell, forward kinematics of
// the twin (an invisible copy of the arm) gives the fingertip point (URDF gripper_frame_link).
// A best-fit rotation + shift maps the known 60 mm cell layout onto those points. The fit error
// (mm) is shown in the panel: it tells how well the teaching and the twin calibration agree.
//
// The blocks come from the server's board state, which the sequencer updates on every pick and
// place. While the robot holds a block, the block rides on the twin's fingertip.
//
// In AR (Sort screen only) the real sheet and blocks are visible through passthrough, so the
// paper and the virtual cubes are hidden. Drawn instead: a coloured outline around every detected
// block, the pick/place rings and arrows, and a banner beyond the far edge of the grid with the
// current move and the reason for it.

const CELL_MM = 60;                       // grid_sheet_60mm.pdf
const BLOCK_M = 0.025;
const PAGE_W = 210, PAGE_H = 297;          // mm
const GRID_X0 = 15, GRID_Y0 = 58.5;        // mm, top-left of the 3x3 grid on the page
const MARKER = 35, MARKER_MARGIN = 12;     // mm
// ArUco DICT_4X4_50 IDs 0..3 as 6x6 bits (1 = black), from OpenCV generateImageMarker
const MARKER_BITS = ['111111101001110101111001111011111111', '111111111111100001101101101011111111',
                     '111111111001111001111011100101111111', '111111101101101101110111110011111111'];
const COLORS = { blue: 0x2f74e0, black: 0x26272b, red: 0xd8393b, yellow: 0xf2c230, green: 0x3cae4f,
                 orange: 0xf08a24, purple: 0x9450c2, block: 0x9a9caa };
const CYCLE = [null, 'green', 'yellow', 'blue', 'red'];   // demo set: 3 green, 2 yellow, 2 blue, 2 red
const KIND_COLOR = { home: 0x3ecf8e, home_frees: 0x4fd1c5, park: 0xf0b54a };

// cell n -> (x, y) in mm from the grid centre; x to the right, y towards the robot
function cellMM(n) {
  const r = Math.floor((n - 1) / 3), c = (n - 1) % 3;
  return [(c - 1) * CELL_MM, (r - 1) * CELL_MM];
}

function sheetTexture(THREE) {
  const s = 4;                               // px per mm
  const cv = document.createElement('canvas');
  cv.width = PAGE_W * s; cv.height = PAGE_H * s;
  const g = cv.getContext('2d');
  g.fillStyle = '#f4f4f1'; g.fillRect(0, 0, cv.width, cv.height);
  const bit = MARKER / 6;
  const origins = [[MARKER_MARGIN, MARKER_MARGIN], [PAGE_W - MARKER_MARGIN - MARKER, MARKER_MARGIN],
                   [PAGE_W - MARKER_MARGIN - MARKER, PAGE_H - MARKER_MARGIN - MARKER],
                   [MARKER_MARGIN, PAGE_H - MARKER_MARGIN - MARKER]];
  g.fillStyle = '#000';
  origins.forEach(([ox, oy], id) => {
    for (let r = 0; r < 6; r++) for (let c = 0; c < 6; c++)
      if (MARKER_BITS[id][r * 6 + c] === '1') g.fillRect((ox + c * bit) * s, (oy + r * bit) * s, bit * s + 1, bit * s + 1);
  });
  g.strokeStyle = '#333'; g.lineWidth = 0.8 * s;
  for (let i = 0; i <= 3; i++) {
    g.beginPath(); g.moveTo((GRID_X0 + i * CELL_MM) * s, GRID_Y0 * s); g.lineTo((GRID_X0 + i * CELL_MM) * s, (GRID_Y0 + 180) * s); g.stroke();
    g.beginPath(); g.moveTo(GRID_X0 * s, (GRID_Y0 + i * CELL_MM) * s); g.lineTo((GRID_X0 + 180) * s, (GRID_Y0 + i * CELL_MM) * s); g.stroke();
  }
  g.fillStyle = '#888'; g.font = `bold ${11 * s / 2.8}px sans-serif`;
  for (let n = 1; n <= 9; n++) {
    const r = Math.floor((n - 1) / 3), c = (n - 1) % 3;
    g.fillText(String(n), (GRID_X0 + c * CELL_MM + 3) * s, (GRID_Y0 + r * CELL_MM + 7) * s);
  }
  g.fillStyle = '#666'; g.font = `bold ${10 * s / 2.8}px sans-serif`; g.textAlign = 'center';
  g.fillText('ROBOT SIDE', PAGE_W / 2 * s, (PAGE_H - 28) * s);
  const tex = new THREE.CanvasTexture(cv);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  return tex;
}

function textSprite(THREE, text, color = '#ffffff') {
  const cv = document.createElement('canvas'); cv.width = 128; cv.height = 128;
  const g = cv.getContext('2d');
  g.fillStyle = 'rgba(15,16,20,0.85)'; g.beginPath(); g.arc(64, 64, 56, 0, Math.PI * 2); g.fill();
  g.fillStyle = color; g.font = 'bold 64px sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText(text, 64, 68);
  const tex = new THREE.CanvasTexture(cv); tex.colorSpace = THREE.SRGBColorSpace;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  sp.scale.set(0.018, 0.018, 1); sp.renderOrder = 5;
  return sp;
}

export function initSorting(ctx) {
  const { THREE, twinRoot, actualBot, toRad, api, say, renderer } = ctx;
  const $ = id => document.getElementById(id);

  // ---- 3D objects ---------------------------------------------------------------
  const group = new THREE.Group();          // sheet frame: x right, z towards the robot, origin = grid centre
  group.visible = false;
  twinRoot.add(group);
  const paper = new THREE.Mesh(new THREE.PlaneGeometry(PAGE_W / 1000, PAGE_H / 1000),
    new THREE.MeshStandardMaterial({ map: sheetTexture(THREE), roughness: 0.95 }));
  paper.rotation.x = -Math.PI / 2;
  paper.position.y = 0.0008;
  paper.receiveShadow = true;
  group.add(paper);

  const blockGeo = new THREE.BoxGeometry(BLOCK_M, BLOCK_M, BLOCK_M);
  const mats = {};
  const matOf = c => mats[c] || (mats[c] = new THREE.MeshStandardMaterial({ color: COLORS[c] ?? COLORS.block, roughness: 0.55 }));
  const blocks = [];
  for (let i = 0; i < 9; i++) {
    const m = new THREE.Mesh(blockGeo, matOf('block'));
    m.castShadow = true; m.receiveShadow = true; m.visible = false;
    group.add(m); blocks.push(m);
  }
  const held = new THREE.Mesh(blockGeo, matOf('block'));
  held.castShadow = true; held.visible = false;
  const tip = actualBot.links?.gripper_frame_link;
  if (tip) tip.add(held);

  // rings: where the current move picks (orange) and places (green)
  const ringGeo = new THREE.RingGeometry(0.022, 0.026, 40);
  const ringSrc = new THREE.Mesh(ringGeo, new THREE.MeshBasicMaterial({ color: 0xf0b54a, transparent: true, opacity: 0.95, depthWrite: false }));
  const ringDst = new THREE.Mesh(ringGeo, new THREE.MeshBasicMaterial({ color: 0x3ecf8e, transparent: true, opacity: 0.95, depthWrite: false }));
  for (const r of [ringSrc, ringDst]) { r.rotation.x = -Math.PI / 2; r.position.y = 0.0015; r.visible = false; r.renderOrder = 3; group.add(r); }

  const arrows = new THREE.Group(); group.add(arrows);

  // AR: a coloured square outline around each detected block (instead of a virtual cube)
  const frameShape = new THREE.Shape();
  frameShape.moveTo(-0.015, -0.015); frameShape.lineTo(0.015, -0.015); frameShape.lineTo(0.015, 0.015); frameShape.lineTo(-0.015, 0.015);
  const hole = new THREE.Path();
  hole.moveTo(-0.0125, -0.0125); hole.lineTo(-0.0125, 0.0125); hole.lineTo(0.0125, 0.0125); hole.lineTo(0.0125, -0.0125);
  frameShape.holes.push(hole);
  const frameGeo = new THREE.ShapeGeometry(frameShape);
  const frameMats = {};
  const frameMat = c => frameMats[c] || (frameMats[c] = new THREE.MeshBasicMaterial({
    color: COLORS[c] ?? COLORS.block, side: THREE.DoubleSide, depthWrite: false }));
  const outlines = [];
  for (let n = 1; n <= 9; n++) {
    const m = new THREE.Mesh(frameGeo, frameMat('block'));
    m.rotation.x = -Math.PI / 2; m.position.copy(cellPos(n, 0.002)); m.visible = false; m.renderOrder = 2;
    group.add(m); outlines.push(m);
  }

  // AR: banner with the current move and why, floating beyond the far edge of the grid
  const BW = 0.34, BH = 0.095, BPX = 2400;
  const bCanvas = document.createElement('canvas');
  bCanvas.width = BW * BPX; bCanvas.height = BH * BPX;
  const bTex = new THREE.CanvasTexture(bCanvas); bTex.colorSpace = THREE.SRGBColorSpace;
  const banner = new THREE.Mesh(new THREE.PlaneGeometry(BW, BH),
    new THREE.MeshBasicMaterial({ map: bTex, transparent: true, depthTest: false, depthWrite: false }));
  banner.renderOrder = 23; banner.visible = false;
  banner.position.set(0, 0.11, -0.16);          // grid frame: up, and past the far row
  group.add(banner);
  let bannerKey = '';
  function drawBanner(title, titleColor, body, foot) {
    const key = JSON.stringify([title, titleColor, body, foot]);
    if (key === bannerKey) return;
    bannerKey = key;
    const g = bCanvas.getContext('2d'), W = bCanvas.width, H = bCanvas.height, pad = 0.01 * BPX;
    g.clearRect(0, 0, W, H);
    g.fillStyle = 'rgba(22,23,28,0.88)'; g.beginPath(); g.roundRect(0, 0, W, H, 0.01 * BPX); g.fill();
    g.textBaseline = 'top'; g.textAlign = 'left';
    g.fillStyle = titleColor; g.font = `700 ${0.016 * BPX}px system-ui, sans-serif`;
    g.fillText(title, pad, pad, W - 2 * pad);
    g.fillStyle = '#e8e8e8'; g.font = `500 ${0.0115 * BPX}px system-ui, sans-serif`;
    const words = (body || '').split(' '), lines = [];
    let line = '';
    for (const w of words) {                          // simple word wrap
      const t = line ? line + ' ' + w : w;
      if (g.measureText(t).width > W - 2 * pad && line) { lines.push(line); line = w; } else line = t;
    }
    if (line) lines.push(line);
    lines.slice(0, 3).forEach((ln, i) => g.fillText(ln, pad, pad + 0.024 * BPX + i * 0.015 * BPX, W - 2 * pad));
    if (foot) {
      g.fillStyle = '#4aa3ff'; g.font = `600 ${0.0105 * BPX}px system-ui, sans-serif`;
      g.fillText(foot, pad, H - pad - 0.0115 * BPX, W - 2 * pad);
    }
    bTex.needsUpdate = true;
  }
  const KIND_HEX = { home: '#3ecf8e', home_frees: '#4fd1c5', park: '#f0b54a' };
  function updateBanner(st) {
    const sort = st.sort, p = st.program, sc = st.scan;
    if (sc?.busy) return drawBanner('Finding the blocks...', '#4aa3ff', 'The detector is reading the photo.', '');
    if (sort && sort.moves.length) {
      const N = sort.moves.length;
      // while running: the move the robot is doing (it stays current until the next move starts)
      const cur = (p?.active && sort.current) ? sort.moves[sort.current - 1] : sort.moves.find(m => m.n > sort.done);
      if (p?.active && cur) {
        return drawBanner(`Move ${cur.n} of ${N}: ${cur.color} from cell ${cur.src} to cell ${cur.dst}`,
                          KIND_HEX[cur.kind] || '#fff', cur.why, p.label.replace(/^Move \d+: /, ''));
      }
      if (sort.done >= N) return drawBanner('Sorted', '#3ecf8e', p?.msg || 'All blocks are in their cells.', '');
      if (cur) return drawBanner(`Plan: ${N} move${N > 1 ? 's' : ''}. Press Run.`, '#e8e8e8',
                                 `First: ${cur.color} from cell ${cur.src} to cell ${cur.dst}. ${cur.why}`,
                                 p?.msg && !p.active ? p.msg : '');
    }
    if (sc && !sc.ok) return drawBanner('Scan problem', '#ff9f1a', sc.msg, 'Fix it and press Take picture again.');
    if (sc?.ok) return drawBanner('Already sorted', '#3ecf8e', sc.msg, '');
    drawBanner('Ready for a picture', '#e8e8e8', 'Put the arm at Home, look down at the whole sheet, then press Take picture.', '');
  }

  function cellPos(n, y = 0) { const [x, z] = cellMM(n); return new THREE.Vector3(x / 1000, y, z / 1000); }

  function makeArrow(src, dst, color, opacity, label) {
    const a = cellPos(src, 0.03), b = cellPos(dst, 0.03);
    const mid = a.clone().add(b).multiplyScalar(0.5); mid.y = 0.03 + 0.25 * a.distanceTo(b) + 0.02;
    const curve = new THREE.QuadraticBezierCurve3(a, mid, b);
    const mat = new THREE.MeshBasicMaterial({ color, transparent: true, opacity, depthWrite: false });
    const g = new THREE.Group();
    g.add(new THREE.Mesh(new THREE.TubeGeometry(curve, 32, 0.0025, 8, false), mat));
    const end = curve.getPoint(1), dir = curve.getTangent(1);
    const cone = new THREE.Mesh(new THREE.ConeGeometry(0.007, 0.016, 16), mat);
    cone.position.copy(end); cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    g.add(cone);
    if (label) { const sp = textSprite(THREE, label); sp.position.copy(curve.getPoint(0.5)).add(new THREE.Vector3(0, 0.012, 0)); g.add(sp); }
    return g;
  }

  // ---- grid placement from taught grab poses -----------------------------------
  const fkBot = actualBot.clone(true);
  fkBot.visible = false;
  for (const j of Object.values(fkBot.joints || {})) j.ignoreLimits = true;   // clone() resets this
  twinRoot.add(fkBot);
  const fkTip = fkBot.links?.gripper_frame_link;
  let fit = null;                                       // {n, rmsMM}

  function fitGrid(poses) {
    fit = null; group.visible = false;
    if (!fkTip || !poses?.cells) return;
    const P = [], Q = [], v = new THREE.Vector3();
    for (let n = 1; n <= 9; n++) {
      const g = poses.cells[String(n)]?.grab;
      if (!g) continue;
      for (const [j, val] of Object.entries(g)) fkBot.joints[j]?.setJointValue(toRad(j, val));
      fkBot.updateMatrixWorld(true);
      fkTip.getWorldPosition(v); twinRoot.worldToLocal(v);
      const [x, y] = cellMM(n);
      P.push([x / 1000, y / 1000]); Q.push([v.x, v.z]);
    }
    if (P.length < 2) return;
    // 2D rigid fit (Kabsch): q = R(theta) p + t
    const mean = A => A.reduce((s, a) => [s[0] + a[0] / A.length, s[1] + a[1] / A.length], [0, 0]);
    const pm = mean(P), qm = mean(Q);
    let sxx = 0, sxy = 0;
    P.forEach((p, i) => {
      const px = p[0] - pm[0], py = p[1] - pm[1], qx = Q[i][0] - qm[0], qy = Q[i][1] - qm[1];
      sxx += px * qx + py * qy; sxy += px * qy - py * qx;
    });
    const th = Math.atan2(sxy, sxx), c = Math.cos(th), s = Math.sin(th);
    const tx = qm[0] - (c * pm[0] - s * pm[1]), tz = qm[1] - (s * pm[0] + c * pm[1]);
    let err = 0;
    P.forEach((p, i) => { const ex = c * p[0] - s * p[1] + tx - Q[i][0], ez = s * p[0] + c * p[1] + tz - Q[i][1]; err += ex * ex + ez * ez; });
    fit = { n: P.length, rmsMM: Math.sqrt(err / P.length) * 1000 };
    group.position.set(tx, 0, tz);
    group.rotation.set(0, -th, 0);                       // three.js yaw is the opposite sense of theta in x-z
    refreshVisibility();
  }

  // ---- panel ----------------------------------------------------------------------
  const cellBtns = [];
  for (let n = 1; n <= 9; n++) {
    const b = document.createElement('button'); b.className = 'scell'; b.dataset.n = n;
    b.innerHTML = `<span class="sn">${n}</span><span class="sc"></span>`;
    b.onclick = () => cycleCell(n);
    $('sortGrid').append(b); cellBtns.push(b);
  }
  let state = null, posesVersion = -1, planKey = '';

  async function cycleCell(n) {
    if (!state || state.program?.active) return;
    const cells = { ...state.board };
    const k = CYCLE.indexOf(cells[n] ?? null);
    cells[n] = CYCLE[(k + 1) % CYCLE.length];
    const r = await api('/api/board', { cells });
    if (!r.ok) say(r.data.msg, true);
  }
  $('sortClear').onclick = async () => {
    const r = await api('/api/board', { cells: {} }); if (!r.ok) say(r.data.msg, true);
  };
  $('sortPlan').onclick = async () => {
    const r = await api('/api/sort/plan', {}); say(r.data.msg, !r.ok);
  };
  // The server keeps the planner choice (the AR menu can change it too). Changing it re-plans.
  // While you are using the dropdown (and 2 s after), the poll does not overwrite it.
  let algTouched = 0;
  const touchAlg = () => { algTouched = performance.now(); };
  $('sortAlg').addEventListener('pointerdown', touchAlg);
  $('sortAlg').addEventListener('keydown', touchAlg);
  $('sortAlg').onchange = async (e) => {
    touchAlg();
    const r = await api('/api/sort/algorithm', { algorithm: e.target.value }); say(r.data.msg, !r.ok);
  };
  $('sortRun').onclick = async () => {
    const r = await api('/api/sort/run', {}); say(r.data.msg, !r.ok);
  };
  $('sortHome').onclick = async () => { const r = await api('/api/go_home', {}); say(r.data.msg, !r.ok); };

  function renderPlan(sort, running) {
    // The moves themselves are in the key: BFS and Dijkstra can give the same count in another order.
    const key = JSON.stringify([sort?.algorithm, sort?.moves?.map(m => [m.src, m.dst]), sort?.start,
                                sort?.current, sort?.done, sort?.measured_s, running]);
    if (key === planKey) return;
    planKey = key;
    $('sortSummary').textContent = !sort ? 'No plan yet. Set the board, then Plan or Run.'
      : sort.summary.text + (sort.measured_s ? ` Measured: ${sort.measured_s.toFixed(1)} s.` : '');
    const ol = $('sortList'); ol.innerHTML = '';
    arrows.clear();
    if (!sort) return;
    if (sort.roles) $('sortRoles').textContent = `A = ${sort.roles.A} (cells 1, 4, 7)   B = ${sort.roles.B} (2, 8)   C = ${sort.roles.C} (6)`;
    for (const m of sort.moves) {
      const li = document.createElement('li');
      const isCur = running && sort.current === m.n && sort.done < m.n;
      li.className = (m.n <= sort.done ? 'done ' : '') + (isCur ? 'cur ' : '') + m.kind;
      li.innerHTML = `<b>${m.src} → ${m.dst}</b> <span class="why"></span>`;
      li.querySelector('.why').textContent = m.why;
      ol.append(li);
      if (m.n > sort.done) {
        const first = m.n === sort.done + 1;
        arrows.add(makeArrow(m.src, m.dst, KIND_COLOR[m.kind] ?? 0xffffff, first ? 0.95 : 0.28, String(m.n)));
      }
    }
    const next = sort.moves.find(m => m.n > sort.done);
    ringSrc.visible = ringDst.visible = !!next;
    if (next) { ringSrc.position.copy(cellPos(next.src, 0.0015)); ringDst.position.copy(cellPos(next.dst, 0.0015)); }
  }

  function update(st) {
    // Synchronous on purpose: an await here would let an older state be drawn after a newer one.
    state = st;
    if (st.poses_version !== posesVersion) {
      posesVersion = st.poses_version;
      api('/api/poses').then(r => {
        fitGrid(r.data);
        $('gridFit').textContent = fit
          ? `Grid placed from ${fit.n} taught grab pose${fit.n > 1 ? 's' : ''}. Fit error ${fit.rmsMM.toFixed(1)} mm.`
          : 'Teach at least 2 grab poses to show the grid in the twin.';
        refreshVisibility();
      }).catch(() => { posesVersion = -1; });     // retry on the next poll
    }
    const running = !!st.program?.active;
    for (let n = 1; n <= 9; n++) {
      const c = st.board[String(n)], b = blocks[n - 1], btn = cellBtns[n - 1];
      if (c) { b.material = matOf(c); b.position.copy(cellPos(n, BLOCK_M / 2 + 0.0008)); outlines[n - 1].material = frameMat(c); }
      btn.querySelector('.sc').style.background = c ? '#' + (COLORS[c] ?? COLORS.block).toString(16).padStart(6, '0') : 'transparent';
      btn.title = c || 'empty';
      btn.disabled = running;
    }
    if (st.holding) held.material = matOf(st.holding);
    $('sortRun').disabled = $('sortPlan').disabled = $('sortClear').disabled = $('sortAlg').disabled = running;
    if (st.sort_algorithm && performance.now() - algTouched > 2000) $('sortAlg').value = st.sort_algorithm;
    renderPlan(st.sort, running);
    const p = st.program;
    $('sortStep').textContent = p?.active ? `${p.label} (${p.step}/${p.count})` : (p?.msg || '');
    const sc = st.scan;
    $('scanStatus').textContent = sc ? `${sc.time ? sc.time + '  ' : ''}${sc.msg}` : '';
    $('scanStatus').style.color = !sc ? '' : sc.busy ? 'var(--accent)' : sc.ok ? '#3ecf8e' : '#f0b54a';
    $('scanBtn').disabled = running || !!sc?.busy;
    refreshVisibility();
  }

  // What is drawn depends on where you look: PC (everything) or AR Sort screen (overlay only).
  function refreshVisibility() {
    const inAR = renderer.xr.isPresenting, arSort = inAR && !!ctx.showInAR?.();
    group.visible = !!fit && (!inAR || arSort);
    paper.visible = !inAR;
    const board = state?.board || {};
    for (let n = 1; n <= 9; n++) {
      const c = board[String(n)];
      blocks[n - 1].visible = !inAR && !!c;
      outlines[n - 1].visible = inAR && !!c;
    }
    held.visible = !inAR && !!state?.holding;
    banner.visible = arSort;
    if (arSort && state) updateBanner(state);
  }

  // Every AR frame: keep the banner turned towards your face.
  function arFrame(head) {
    if (!banner.visible) return;
    const p = new THREE.Vector3();
    head.getWorldPosition(p);
    banner.lookAt(p);
  }

  // PC: scan a photo file (for example from a phone) instead of the Quest camera
  $('scanBtn').onclick = () => $('scanFile').click();
  $('scanFile').onchange = async e => {
    const f = e.target.files[0];
    e.target.value = '';
    if (!f) return;
    say('Scanning ' + f.name + '...');
    try {
      const r = await fetch('/api/scan?source=upload', { method: 'POST', headers: { 'Content-Type': f.type || 'image/jpeg' }, body: f });
      const d = await r.json().catch(() => ({}));
      say(d.msg || (r.ok ? 'Scanned.' : 'Scan failed.'), !r.ok);
    } catch (err) { say('Scan failed: ' + err.message, true); }
  };

  return { update, group, fitInfo: () => fit, arFrame, refreshVisibility };
}
