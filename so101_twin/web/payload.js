// Payload test panel on the 2D twin page: the "simulation view" of the payload experiment.
//
// With the real robot the numbers are the servos' own readings (Present_Load, % of full drive),
// with the model's prediction beside them. With --sim there are no readings, so the page shows the
// static gravity model from the server (% of the 30 kg·cm stall torque). The colours of the twin's
// servos come from the same numbers (index.html: paintLoad).
//
// The test itself runs on the server (twin_server.py: run_payload): Home -> hover -> grab -> close ->
// hover -> lift -> HOLD 5 s -> left -> right -> lift -> hover -> grab -> open -> hover -> Home.

const MASSES = [0, 100, 200, 300, 400, 500];
const SPEEDS = [10, 20, 30];
const JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'];
const NAME = j => j.replace('shoulder_', 'shoulder ').replace('_flex', ' flex').replace('_roll', ' roll');
const LEVEL_TEXT = { ok: '', warn: '', high: 'close to the limit', flag: 'OVER' };

export function initPayload({ api, say, levelOf, valueOf, hex }) {
  const $ = id => document.getElementById(id);
  let mass = 0, speed = 20, confirm = false, info = null, posesVersion = -1, lastMsg = '';

  function seg(boxId, label, values, fmt, isOn, onPick) {
    const box = $(boxId);
    box.innerHTML = `<span class="lab">${label}</span>`;
    for (const v of values) {
      const b = document.createElement('button');
      b.textContent = fmt(v); b.dataset.v = v;
      b.onclick = () => { onPick(v); paint(); };
      box.appendChild(b);
    }
    const paint = () => box.querySelectorAll('button').forEach(b => b.classList.toggle('on', isOn(+b.dataset.v)));
    paint();
    return paint;
  }
  const paintMass = seg('payMass', 'Payload (g)', MASSES, m => (m ? String(m) : 'empty'), m => m === mass,
                        m => { mass = m; confirm = false; precheck(); });
  const paintSpeed = seg('paySpeed', 'Speed', SPEEDS, v => v + ' °/s', v => v === speed, v => { speed = v; });

  async function loadInfo() {
    const r = await api('/api/payload');
    if (!r.ok) return;
    info = r.data;
    precheck();
  }
  async function precheck() {
    const r = await api('/api/payload/precheck', { mass_g: mass });
    const pc = r.data?.precheck;
    $('payPre').textContent = r.data?.msg || '';
    $('payPre').style.color = !r.ok ? 'var(--danger)' : pc?.level === 'over' ? '#ff2d55' : pc?.level === 'high' ? '#e5484d'
                            : pc?.level === 'warn' ? '#ff7a1a' : '';
  }

  $('payRun').onclick = async () => {
    const r = await api('/api/payload/run', { mass_g: mass, speed, confirm });
    confirm = !!r.data?.confirm;                        // predicted over 100 %: the next press confirms
    $('payRun').textContent = confirm ? 'Confirm: run anyway' : 'Run test';
    $('payRun').classList.toggle('stop', confirm);
    say(r.data?.msg || '', !r.ok && !confirm);
  };
  $('payRelease').onclick = async () => {
    const r = await api('/api/payload/release', {});
    say(r.data?.msg || 'RELEASE sent', !r.ok);
  };
  $('payHome').onclick = async () => { const r = await api('/api/go_home', {}); say(r.data?.msg || '', !r.ok); };

  function update(st) {
    if (st.poses_version !== posesVersion) { posesVersion = st.poses_version; loadInfo(); }
    const L = st.loads, pl = st.payload || {}, prog = st.program || {};
    const real = L?.source === 'real', running = !!prog.active;
    const payRunning = running && pl.status === 'running';

    $('paySrc').innerHTML = (real ? '<b style="color:#3ecf8e">Live servo readings</b> (model beside them)'
                                  : '<b style="color:#4aa3ff">Model prediction</b> (no servo readings: --sim)')
      + (info ? ` · container ${info.container_g == null ? '<span style="color:#f0b54a">not set (0 g)</span>' : info.container_g + ' g'}` : '')
      + (L?.err ? ` · <span style="color:#f0b54a">${L.err}</span>` : '');
    if (info?.missing?.length) {
      $('payMsg').innerHTML = `<span style="color:#f0b54a">Before a test: ${info.missing.join(', ')} (Teach page).</span>`;
    } else $('payMsg').textContent = (info?.notes || []).join(' ') + (pl.file ? ` Log: ${pl.file}` : '');

    $('payRun').disabled = running || !!info?.missing?.length;
    $('payHome').disabled = running;
    document.querySelectorAll('#payMass button, #paySpeed button').forEach(b => { b.disabled = running; });
    if (payRunning) {                                   // show the run's own choices
      if (pl.mass_g !== mass) { mass = pl.mass_g; paintMass(); }
      if (pl.speed !== speed) { speed = pl.speed; paintSpeed(); }
    }

    // the hold
    const h = pl.hold;
    if (h) {
      const res = h.result ? `HOLD ${h.result}` : `HOLD ${h.t.toFixed(1)} / ${h.s.toFixed(0)} s`;
      $('payHold').textContent = `${res} · max error ${h.max_err.toFixed(1)}° on ${NAME(h.joint)} (limit ${h.tol}°)`
        + (h.slip ? ' · SLIPPED' : '') + (h.flags?.length ? ' · ' + h.flags.join(', ') : '');
      $('payHold').style.color = h.result === 'PASS' ? '#3ecf8e' : h.result === 'FAIL' ? '#e5484d' : '#4aa3ff';
    } else $('payHold').textContent = payRunning ? `${prog.label} (${prog.step}/${prog.count})` : '';
    if (h && payRunning && !h.result && !/^HOLD/.test(prog.label)) $('payHold').textContent += ` · ${prog.label}`;
    else if (!h && payRunning) $('payHold').style.color = '#4aa3ff';

    // live warning
    const hot = JOINTS.filter(j => ['high', 'flag'].includes(levelOf(j, L)));
    if (hot.length) {
      const j = hot[0], f = L.flags?.[j];
      $('payWarn').textContent = f?.length ? `${NAME(j)} reports ${f.join(', ')}: press RELEASE (or R)`
        : real ? `${NAME(j)} at ${valueOf(j, L).toFixed(0)} %: press RELEASE (or R) if it keeps rising`
               : `${NAME(j)} predicted at ${valueOf(j, L).toFixed(0)} % of stall${valueOf(j, L) >= 100 ? ': it cannot hold this' : ''}`;
    } else $('payWarn').textContent = '';

    // table
    const head = real ? ['motor', 'load', 'model', 'current', 'temp'] : ['motor', 'model (% of stall)'];
    let html = '<tr>' + head.map(t => `<th>${t}</th>`).join('') + '</tr>';
    for (const j of JOINTS) {
      const v = valueOf(j, L), lv = levelOf(j, L), c = hex[lv] || 'var(--dim)';
      const bar = v == null ? '' : `<span class="bar" style="width:${Math.min(v, 100) * 0.6}px;background:${c}"></span>`;
      const main = v == null ? '–' : `${bar}<b style="color:${c}">${v.toFixed(0)} %</b>`;
      html += `<tr><td>${NAME(j)}</td><td>${main}</td>`;
      if (real) {
        const pv = L.pred?.[j], cur = L.current?.[j], tc = L.temp?.[j];
        html += `<td>${pv == null ? '–' : pv.toFixed(0) + ' %'}</td><td>${cur == null ? '–' : (cur / 1000).toFixed(2) + ' A'}</td>`
              + `<td style="color:${tc >= 60 ? '#e5484d' : ''}">${tc == null ? '–' : tc.toFixed(0) + ' °C'}</td>`;
      }
      html += '</tr>';
    }
    if (!real) html += '<tr><td colspan="2" style="color:var(--dim);font-size:11px;text-align:left">shoulder pan: no gravity load on a vertical axis, its load comes from speed changes and friction (only the real servo shows it)</td></tr>';
    $('payTable').innerHTML = html;

    // result message once
    const msg = !running && pl.msg && pl.status !== 'running' ? pl.msg : '';
    if (msg && msg !== lastMsg) say('Payload test: ' + msg, pl.result !== 'PASS');
    lastMsg = msg;
  }

  return { update, colourOn: () => $('payColor').checked, precheck };
}
