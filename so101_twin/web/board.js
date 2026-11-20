// Sorting demo in the digital twin: the printed grid sheet, the blocks, and the plan.
//
// Where is the sheet? The taught grab poses say it: for every taught cell, forward kinematics of
// the twin (an invisible copy of the arm) gives the fingertip point (URDF gripper_frame_link).
// A best-fit rotation + shift maps the known 60 mm cell layout onto those points. The fit error
// (mm) is shown in the panel: it tells how well the teaching and the twin calibration agree.
//
// The blocks come from the server's board state, which the sequencer updates on every pick and
// place. While the robot holds a block, the block rides on the twin's fingertip.

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
  const ringGeo = new THREE.RingGeometry(0.019, 0.024, 40);
  const ringSrc = new THREE.Mesh(ringGeo, new THREE.MeshBasicMaterial({ color: 0xf0b54a, transparent: true, opacity: 0.95, depthWrite: false }));
  const ringDst = new THREE.Mesh(ringGeo, new THREE.MeshBasicMaterial({ color: 0x3ecf8e, transparent: true, opacity: 0.95, depthWrite: false }));
  for (const r of [ringSrc, ringDst]) { r.rotation.x = -Math.PI / 2; r.position.y = 0.0015; r.visible = false; r.renderOrder = 3; group.add(r); }

  const arrows = new THREE.Group(); group.add(arrows);

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
    group.visible = true;
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
  $('sortRun').onclick = async () => {
    const r = await api('/api/sort/run', {}); say(r.data.msg, !r.ok);
  };
  $('sortHome').onclick = async () => { const r = await api('/api/go_home', {}); say(r.data.msg, !r.ok); };

  function renderPlan(sort, running) {
    const key = JSON.stringify([sort?.moves?.length, sort?.start, sort?.current, sort?.done, running]);
    if (key === planKey) return;
    planKey = key;
    $('sortSummary').textContent = sort ? sort.summary.short : 'No plan yet. Set the board, then Plan or Run.';
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
        group.visible = !!fit && !renderer.xr.isPresenting;
      }).catch(() => { posesVersion = -1; });     // retry on the next poll
    }
    const running = !!st.program?.active;
    for (let n = 1; n <= 9; n++) {
      const c = st.board[String(n)], b = blocks[n - 1], btn = cellBtns[n - 1];
      b.visible = !!c;
      if (c) { b.material = matOf(c); b.position.copy(cellPos(n, BLOCK_M / 2 + 0.0008)); }
      btn.querySelector('.sc').style.background = c ? '#' + (COLORS[c] ?? COLORS.block).toString(16).padStart(6, '0') : 'transparent';
      btn.title = c || 'empty';
      btn.disabled = running;
    }
    held.visible = !!st.holding;
    if (st.holding) held.material = matOf(st.holding);
    $('sortRun').disabled = $('sortPlan').disabled = $('sortClear').disabled = running;
    renderPlan(st.sort, running);
    const p = st.program;
    $('sortStep').textContent = p?.active ? `${p.label} (${p.step}/${p.count})` : (p?.msg || '');
    group.visible = !!fit && !renderer.xr.isPresenting;    // AR view of the board comes in a later step
  }

  return { update, group, fitInfo: () => fit };
}
