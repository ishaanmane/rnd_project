// Camera capture for Scan, through the browser's standard camera API (getUserMedia).
//
// On the Quest Browser the passthrough cameras show up as video inputs, labelled for example
// "camera 1, facing back" (left) and "camera 2, facing back" (right). We use the first
// back-facing one. Labels are only filled in after camera permission has been granted.
//
// The browser asks for camera permission the first time. If that first ask happens inside AR,
// the Quest leaves AR to show the prompt, so the page offers "Allow camera" BEFORE entering AR.
//
// Each Scan opens the camera, waits a moment for exposure to settle, grabs one frame as a JPEG
// and closes the camera again (no stream runs in the background).

const SETTLE_MS = 700;     // auto-exposure needs a moment after the camera starts

export function cameraSupported() {
  return !!(navigator.mediaDevices && navigator.mediaDevices.getUserMedia);
}

async function videoInputs() {
  const devs = await navigator.mediaDevices.enumerateDevices();
  return devs.filter(d => d.kind === 'videoinput');
}

function pickCamera(devs) {
  return devs.find(d => /back/i.test(d.label)) || devs[0] || null;
}

// Ask for permission once (call from a button press on the 2D page). Returns the camera labels.
export async function allowCamera() {
  if (!cameraSupported()) throw new Error('this browser has no camera API');
  const s = await navigator.mediaDevices.getUserMedia({ video: true });
  s.getTracks().forEach(t => t.stop());
  return (await videoInputs()).map(d => d.label || '(unnamed camera)');
}

export async function cameraPermission() {
  try {
    const st = await navigator.permissions.query({ name: 'camera' });
    return st.state;                    // 'granted' | 'prompt' | 'denied'
  } catch { return 'unknown'; }         // not every browser can answer this
}

// One frame as a JPEG. Returns { blob, width, height, label }.
export async function captureJPEG(quality = 0.92) {
  if (!cameraSupported()) throw new Error('this browser has no camera API');
  const cam = pickCamera(await videoInputs());
  const video = { width: { ideal: 1920 }, height: { ideal: 1440 } };
  if (cam && cam.deviceId) video.deviceId = { exact: cam.deviceId };
  const stream = await navigator.mediaDevices.getUserMedia({ video });
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
    const label = stream.getVideoTracks()[0]?.label || cam?.label || 'camera';
    return { blob, width: c.width, height: c.height, label };
  } finally {
    stream.getTracks().forEach(t => t.stop());
  }
}
