// Camera capture for Take picture, through the browser's standard camera API (getUserMedia).
//
// Quest Browser cameras (Quest 3 / 3S, Horizon OS v74+, Quest Browser 40.1+):
//   "camera 0, facing front"  the AVATAR camera: a virtual camera for video calls. It never sees
//                             the room, so its pictures are black. We refuse it.
//   "camera 1, facing back"   left passthrough camera   } listed ONLY when the Browser has the
//   "camera 2, facing back"   right passthrough camera  } "Headset cameras" permission
// The Quest's own prompt comes up with "Virtual cameras" ticked and "Headset cameras" unticked, so
// a plain Allow gives only the avatar camera, and the Browser does not ask again. The fix is a
// setting: the lock icon left of the address bar > Headset cameras > Allow, or
// Settings > Privacy & safety > Browser > Headset cameras ON.
//
// The browser may hide camera names until a camera has been opened, so we open one first (asking
// for the world-facing one), read the names, and switch to a "facing back" camera if needed.
// A camera picked with the Camera button (remembered in this browser) always wins.
//
// The frame is read straight from the camera track (ImageCapture.grabFrame) when the browser
// supports it: inside AR the Quest Browser stops painting the page, so a <video> element can
// hold stale frames. Every picture is checked for blackness and retried briefly.

const SETTLE_MS = 700;                 // auto-exposure needs a moment after the camera starts
const BLACK_RETRY_MS = 2500;           // keep trying this long while the frames are black
const SIZE = { width: { ideal: 1920 }, height: { ideal: 1440 } };
const PREF_KEY = 'so101-twin-camera';  // deviceId picked with the Camera button ('' = automatic)

export const HEADSET_CAMERAS_FIX =
  'Exit AR. Lock icon left of the address bar > Headset cameras > Allow ' +
  '(or Settings > Privacy & safety > Browser > Headset cameras ON). Then reload.';

export class CameraProblem extends Error {
  constructor(short, fix) { super(short); this.name = 'CameraProblem'; this.short = short; this.fix = fix; }
}

export function cameraSupported() {
  return !!(navigator.mediaDevices && navigator.mediaDevices.getUserMedia);
}

export async function listCameras() {
  const devs = await navigator.mediaDevices.enumerateDevices();
  return devs.filter(d => d.kind === 'videoinput').map(d => ({ deviceId: d.deviceId, label: d.label }));
}

export const isAvatarCamera = label => /facing front/i.test(label || '');
export const isRoomCamera = label => /facing back/i.test(label || '');

export function preferredCamera() {
  try { return localStorage.getItem(PREF_KEY) || ''; } catch { return ''; }
}
export function setPreferredCamera(id) {
  try { if (id) localStorage.setItem(PREF_KEY, id); else localStorage.removeItem(PREF_KEY); } catch {}
}

// "camera 1, facing back" -> "Cam 1 (left)". Unnamed cameras get their position in the list.
export function shortName(label, index) {
  const m = /camera\s*(\d+)/i.exec(label || '');
  const n = m ? m[1] : String(index + 1);
  const side = isAvatarCamera(label) ? ' (avatar)' : n === '1' && isRoomCamera(label) ? ' (left)'
             : n === '2' && isRoomCamera(label) ? ' (right)' : '';
  return `Cam ${n}${side}`;
}

// Ask for permission once (call from a button press on the 2D page).
// Returns { names, roomCameras } so the page can say whether the headset cameras are allowed.
export async function allowCamera() {
  if (!cameraSupported()) throw new Error('this browser has no camera API');
  const s = await navigator.mediaDevices.getUserMedia({ video: { facingMode: { ideal: 'environment' } } });
  s.getTracks().forEach(t => t.stop());
  const cams = await listCameras();
  return { names: cams.map((d, i) => d.label || `(camera ${i + 1}, no name)`),
           roomCameras: cams.filter(d => isRoomCamera(d.label)).length };
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
  const back = devs.filter(d => isRoomCamera(d.label));
  if (back.length && !(cur && isRoomCamera(cur.label))) {  // opened the wrong one: switch
    stream.getTracks().forEach(t => t.stop());
    stream = await navigator.mediaDevices.getUserMedia({ video: { deviceId: { exact: back[0].deviceId }, ...SIZE } });
  }
  return stream;
}

function isBlack(source, w, h) {                           // mean brightness of a small copy
  const c = document.createElement('canvas'); c.width = 64; c.height = Math.max(1, Math.round(64 * h / w));
  const g = c.getContext('2d', { willReadFrequently: true });
  g.drawImage(source, 0, 0, c.width, c.height);
  const d = g.getImageData(0, 0, c.width, c.height).data;
  let sum = 0;
  for (let i = 0; i < d.length; i += 4) sum += d[i] + d[i + 1] + d[i + 2];
  return sum / (d.length / 4) / 3 < 6;                     // 0..255: below 6 is black
}

// One frame from the track (ImageCapture) or from a <video> kept in the page. Returns a drawable.
async function grabOnce(track, video) {
  if (typeof ImageCapture !== 'undefined') {
    try {
      const bmp = await new ImageCapture(track).grabFrame();
      return { img: bmp, w: bmp.width, h: bmp.height };
    } catch { /* fall back to the video element */ }
  }
  return { img: video, w: video.videoWidth, h: video.videoHeight };
}

// One frame as a JPEG. Returns { blob, width, height, label }. Throws CameraProblem with a fix.
export async function captureJPEG(quality = 0.92) {
  if (!cameraSupported()) throw new CameraProblem('This browser has no camera API.', 'Use Scan photo on the 2D page.');
  const stream = await openStream();
  const track = stream.getVideoTracks()[0];
  const label = track?.label || 'camera';
  const video = document.createElement('video');
  // kept in the page (tiny, nearly invisible, never display:none): an unpainted video may not update
  Object.assign(video.style, { position: 'fixed', left: '0', top: '0', width: '2px', height: '2px', opacity: '0.01', pointerEvents: 'none' });
  try {
    if (isAvatarCamera(label)) {
      throw new CameraProblem(`${label} is the avatar camera: it never sees the room.`, HEADSET_CAMERAS_FIX);
    }
    video.muted = true; video.playsInline = true; video.srcObject = stream;
    document.body.appendChild(video);
    await video.play().catch(() => {});
    const t0 = performance.now();
    while (!video.videoWidth && performance.now() - t0 < 3000) await new Promise(r => setTimeout(r, 50));
    await new Promise(r => setTimeout(r, SETTLE_MS));
    let f = null;
    const t1 = performance.now();
    do {
      f = await grabOnce(track, video);
      if (f.w && !isBlack(f.img, f.w, f.h)) break;
      f = null;
      await new Promise(r => setTimeout(r, 250));
    } while (performance.now() - t1 < BLACK_RETRY_MS);
    if (!f) throw new CameraProblem(`${label} gives only black pictures.`, HEADSET_CAMERAS_FIX);
    const c = document.createElement('canvas');
    c.width = f.w; c.height = f.h;
    c.getContext('2d').drawImage(f.img, 0, 0);
    const blob = await new Promise((res, rej) =>
      c.toBlob(b => (b ? res(b) : rej(new Error('could not encode the photo'))), 'image/jpeg', quality));
    return { blob, width: c.width, height: c.height, label };
  } finally {
    stream.getTracks().forEach(t => t.stop());
    video.remove();
  }
}
