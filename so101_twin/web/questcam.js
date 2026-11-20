// Camera capture for Scan, through the browser's standard camera API (getUserMedia).
//
// On the Quest Browser the cameras show up as video inputs named like "camera 0, facing front",
// "camera 1, facing back" and "camera 2, facing back". The names are the DEVICE names: they do
// not change with where you look. For Scan we want a camera that looks out at the world
// ("facing back").
//
// The browser may hide the names until a camera has been opened in this page. So we first open
// a camera (asking for the world-facing one), then read the names, and if the open camera is not
// a "facing back" one we switch to one that is. A camera picked with the Camera button (remembered
// in this browser) always wins.
//
// The first permission prompt inside AR makes the Quest leave AR, so the page offers
// "Allow camera" BEFORE entering AR. Each Scan opens the camera, waits for exposure to settle,
// grabs one frame as a JPEG and closes the camera again.

const SETTLE_MS = 700;                 // auto-exposure needs a moment after the camera starts
const SIZE = { width: { ideal: 1920 }, height: { ideal: 1440 } };
const PREF_KEY = 'so101-twin-camera';  // deviceId picked with the Camera button ('' = automatic)

export function cameraSupported() {
  return !!(navigator.mediaDevices && navigator.mediaDevices.getUserMedia);
}

export async function listCameras() {
  const devs = await navigator.mediaDevices.enumerateDevices();
  return devs.filter(d => d.kind === 'videoinput').map(d => ({ deviceId: d.deviceId, label: d.label }));
}

export function preferredCamera() {
  try { return localStorage.getItem(PREF_KEY) || ''; } catch { return ''; }
}
export function setPreferredCamera(id) {
  try { if (id) localStorage.setItem(PREF_KEY, id); else localStorage.removeItem(PREF_KEY); } catch {}
}

// "camera 1, facing back" -> "Cam 1 (back)". Unnamed cameras get their position in the list.
export function shortName(label, index) {
  const m = /camera\s*(\d+)/i.exec(label || '');
  const side = /back/i.test(label || '') ? ' (back)' : /front/i.test(label || '') ? ' (front)' : '';
  return (m ? `Cam ${m[1]}` : `Cam ${index + 1}`) + side;
}

// Ask for permission once (call from a button press on the 2D page). Returns the camera names.
export async function allowCamera() {
  if (!cameraSupported()) throw new Error('this browser has no camera API');
  const s = await navigator.mediaDevices.getUserMedia({ video: true });
  s.getTracks().forEach(t => t.stop());
  return (await listCameras()).map((d, i) => d.label || `(camera ${i + 1}, no name)`);
}

export async function cameraPermission() {
  try {
    const st = await navigator.permissions.query({ name: 'camera' });
    return st.state;                    // 'granted' | 'prompt' | 'denied'
  } catch { return 'unknown'; }         // not every browser can answer this
}

async function openStream() {
  const pref = preferredCamera();
  if (pref) {
    try { return await navigator.mediaDevices.getUserMedia({ video: { deviceId: { exact: pref }, ...SIZE } }); }
    catch { /* the remembered camera is gone: fall back to automatic */ }
  }
  let stream = await navigator.mediaDevices.getUserMedia({ video: { facingMode: { ideal: 'environment' }, ...SIZE } });
  const devs = await listCameras();                       // names are available now
  const curId = stream.getVideoTracks()[0]?.getSettings?.().deviceId;
  const cur = devs.find(d => d.deviceId === curId);
  const back = devs.filter(d => /back/i.test(d.label));
  if (back.length && !(cur && /back/i.test(cur.label))) {  // opened the wrong one: switch
    stream.getTracks().forEach(t => t.stop());
    stream = await navigator.mediaDevices.getUserMedia({ video: { deviceId: { exact: back[0].deviceId }, ...SIZE } });
  }
  return stream;
}

// One frame as a JPEG. Returns { blob, width, height, label }.
export async function captureJPEG(quality = 0.92) {
  if (!cameraSupported()) throw new Error('this browser has no camera API');
  const stream = await openStream();
  try {
    const el = document.createElement('video');
    el.muted = true; el.playsInline = true; el.srcObject = stream;
    await el.play();
    const t0 = performance.now();
    while (!el.videoWidth && performance.now() - t0 < 3000) await new Promise(r => setTimeout(r, 50));
    if (!el.videoWidth) throw new Error('the camera gave no picture');
    await new Promise(r => setTimeout(r, SETTLE_MS));
    const c = document.createElement('canvas');
    c.width = el.videoWidth; c.height = el.videoHeight;
    c.getContext('2d').drawImage(el, 0, 0);
    const blob = await new Promise((res, rej) =>
      c.toBlob(b => (b ? res(b) : rej(new Error('could not encode the photo'))), 'image/jpeg', quality));
    const label = stream.getVideoTracks()[0]?.label || 'camera';
    return { blob, width: c.width, height: c.height, label };
  } finally {
    stream.getTracks().forEach(t => t.stop());
  }
}
