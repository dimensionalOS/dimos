// DOM helpers shared by the teleop view + the WebRTC bring-up.

export function setStatus(msg) {
    const el = document.getElementById('teleop-status');
    if (el) el.textContent = msg;
}

// Returns the #robot-cam <video> (WebRTC sink + GL-quad texture source),
// creating a hidden one if the active view didn't render it (VR/teleop view
// shows the feed on a quad, not as a DOM element).
export function ensureRobotCam() {
    let v = document.getElementById('robot-cam');
    if (!v) {
        v = document.createElement('video');
        v.id = 'robot-cam';
        v.autoplay = true;
        v.muted = true;
        v.playsInline = true;
        v.style.display = 'none';
        document.body.appendChild(v);
    }
    return v;
}
