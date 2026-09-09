"use strict";

const $ = (id) => document.getElementById(id);
// localStorage can be unavailable (private mode, storage blocked). The app must still work,
// it just forgets who you are between reloads.
const store = {
  id: null,
  name: null,
  load() {
    try {
      this.id = localStorage.getItem("person_id");
      this.name = localStorage.getItem("name");
    } catch (e) { /* no storage: stay in memory only */ }
  },
  save(id, name) {
    this.id = id;
    this.name = name;
    try { localStorage.setItem("person_id", id); localStorage.setItem("name", name); } catch (e) {}
  },
  clear() {
    this.id = null;
    this.name = null;
    try { localStorage.removeItem("person_id"); localStorage.removeItem("name"); } catch (e) {}
  },
};
store.load();
let lastId = 0;          // highest message id we have shown
let generation = null;
let polling = false;

// --- onboarding ---------------------------------------------------------

let selfieData = null;
let stream = null;

const MAX_EDGE = 1280;   // long side of the JPEG we send

// The camera lives in the page: a phone should never see a file picker for a selfie.
async function openCamera() {
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) return useFilePicker();
  try {
    stream = await navigator.mediaDevices.getUserMedia({
      video: { facingMode: "user", width: { ideal: 1280 } },
      audio: false,
    });
  } catch (err) {
    // denied, no camera, or a non-HTTPS origin (the funnel is HTTPS, so this is rare on a phone)
    return useFilePicker();
  }
  $("video").srcObject = stream;
  $("camera").hidden = false;
  $("shutter").textContent = "Take a selfie";
}

function closeCamera() {
  if (stream) stream.getTracks().forEach((t) => t.stop());
  stream = null;
  $("camera").hidden = true;
}

function useFilePicker() {
  $("camera").hidden = true;
  $("shutter").textContent = "Take a selfie";
  // iOS Safari and Android Chrome both open the camera directly from `capture="user"`;
  // where they do not, the gallery is still a fine answer.
  $("selfie").click();
}

$("shutter").addEventListener("click", async () => {
  if (!stream) {
    $("remove-selfie").hidden = false;
    await openCamera();
    return;
  }
  if (!$("video").videoWidth) return;
  const video = $("video");
  const scale = Math.min(1, MAX_EDGE / Math.max(video.videoWidth, video.videoHeight));
  const canvas = document.createElement("canvas");
  canvas.width = Math.round(video.videoWidth * scale);
  canvas.height = Math.round(video.videoHeight * scale);
  const ctx = canvas.getContext("2d");
  ctx.translate(canvas.width, 0);
  ctx.scale(-1, 1);                        // the preview is mirrored; keep the photo the same way
  ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
  showSelfie(canvas.toDataURL("image/jpeg", 0.85));
  closeCamera();
});

$("retake").addEventListener("click", async () => {
  selfieData = null;
  $("preview").hidden = true;
  $("retake").hidden = true;
  $("shutter").hidden = false;
  await openCamera();
});

function showSelfie(dataUrl) {
  selfieData = dataUrl;
  $("preview").src = dataUrl;
  $("preview").hidden = false;
  $("shutter").hidden = true;
  $("retake").hidden = false;
  $("remove-selfie").hidden = false;
}

$("remove-selfie").addEventListener("click", () => {
  closeCamera();
  selfieData = null;
  $("selfie").value = "";
  $("preview").hidden = true;
  $("retake").hidden = true;
  $("remove-selfie").hidden = true;
  $("shutter").hidden = false;
  $("shutter").textContent = "Add a selfie";
});

$("selfie").addEventListener("change", async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  showSelfie(await shrink(file));
});

// Phone cameras hand back 4000px photos; 1280px is plenty for a face match and small to upload.
function shrink(file) {
  return new Promise((resolve, reject) => {
    const img = new Image();
    img.onload = () => {
      const scale = Math.min(1, MAX_EDGE / Math.max(img.width, img.height));
      const canvas = document.createElement("canvas");
      canvas.width = Math.round(img.width * scale);
      canvas.height = Math.round(img.height * scale);
      canvas.getContext("2d").drawImage(img, 0, 0, canvas.width, canvas.height);
      URL.revokeObjectURL(img.src);
      resolve(canvas.toDataURL("image/jpeg", 0.85));
    };
    img.onerror = reject;
    img.src = URL.createObjectURL(file);
  });
}

$("enroll").addEventListener("submit", async (e) => {
  e.preventDefault();
  const name = $("name").value.trim();
  if (!name) return $("name").focus();
  $("submit").disabled = true;
  $("enroll-error").hidden = true;
  $("enroll-status").textContent = "Saying hello to FRANK…";
  $("enroll-status").hidden = false;
  try {
    const person = await post("/api/people", { name, selfie: selfieData });
    store.save(person.person_id, person.name);
    closeCamera();
    showChat();
  } catch (err) {
    $("enroll-error").textContent = "Could not send that to FRANK (" + err.message + "). Try again.";
    $("enroll-error").hidden = false;
  } finally {
    $("submit").disabled = false;
    $("enroll-status").hidden = true;
  }
});

// --- chat ---------------------------------------------------------------

async function send(text) {
  const msg = await post(`/api/people/${store.id}/messages`, { text });
  render(msg);
}

$("composer").addEventListener("submit", async (e) => {
  e.preventDefault();
  const text = $("text").value.trim();
  if (!text) return;
  $("text").value = "";
  try {
    await send(text);
  } catch (err) {
    $("text").value = text;
  }
});

$("menu-button").addEventListener("click", () => {
  $("menu").hidden = !$("menu").hidden;
  $("menu-button").setAttribute("aria-expanded", String(!$("menu").hidden));
});

$("forget").addEventListener("click", async () => {
  if (!confirm("FRANK will forget your face, your name and this chat.")) return;
  try { await post(`/api/people/${store.id}/forget`, {}); } catch (e) {}
  store.clear();
  location.reload();
});

$("startover").addEventListener("click", () => {
  if (!confirm("Start over as someone else? FRANK keeps remembering " + store.name + ".")) return;
  store.clear();
  location.reload();
});

function render(msg) {
  if (msg.id <= lastId) return;
  lastId = msg.id;
  const el = document.createElement("div");
  el.className = "msg " + (msg.from === "frank" ? "frank" : "person");
  const sender = document.createElement("span");
  sender.className = "sender";
  sender.textContent = msg.from === "frank" ? "frank" : (store.name || "you");
  const body = document.createElement("span");
  body.className = "message-body";
  body.textContent = msg.text;
  el.append(sender, body);
  $("messages").insertBefore(el, thinking);
  showThinking(msg.from === "person");
  $("messages").scrollTop = $("messages").scrollHeight;
}

const thinking = document.createElement("div");
thinking.className = "thinking";
thinking.textContent = "Waiting for Frank...";
thinking.hidden = true;

function showThinking(on) { thinking.hidden = !on; }

async function pollForever() {
  if (polling) return;
  polling = true;
  while (store.id) {
    try {
      const url = `/api/people/${store.id}/messages?after=${lastId}&wait=25`;
      const res = await fetch(url);
      if (res.status === 404) { store.clear(); location.reload(); return; }
      const data = await res.json();
      if (generation !== null && data.generation && generation !== data.generation) {
        lastId = 0;
        $("messages").replaceChildren(thinking);
        showThinking(false);
      }
      generation = data.generation || generation;
      data.messages.forEach(render);
    } catch (err) {
      await sleep(2000);   // tunnel hiccup; try again
    }
  }
  polling = false;
}

// --- plumbing -----------------------------------------------------------

async function post(url, body) {
  const res = await fetch(url, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!res.ok) throw new Error(`HTTP ${res.status}`);
  return res.json();
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function showChat(history = []) {
  closeCamera();
  $("onboarding").hidden = true;
  $("chat").hidden = false;
  $("whoami").textContent = "you are " + store.name;
  $("startover").textContent = "Not " + store.name + "? Start over";
  $("startover").hidden = false;
  $("messages").appendChild(thinking);
  history.forEach(render);
  pollForever();
}

function showOnboarding() {
  $("chat").hidden = true;
  $("onboarding").hidden = false;
}

// A stored identity is only worth trusting if the server still knows it — the store is wiped at
// the end of a demo day, and a phone that kept the old id would chat into a void.
async function resume() {
  if (!store.id) return showOnboarding();
  let person;
  try {
    const res = await fetch(`/api/people/${store.id}`);
    if (res.status === 404) { store.clear(); return showOnboarding(); }
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    person = await res.json();
  } catch (err) {
    // server unreachable: keep the identity, open the chat, let the poller retry
    return showChat();
  }
  store.save(person.person_id, person.name);
  let history = [];
  try {
    const res = await fetch(`/api/people/${store.id}/history?limit=50`);
    if (res.ok) history = (await res.json()).messages;
  } catch (err) {}
  showChat(history);
}

resume();

// --- push to talk -------------------------------------------------------
// Hold the mic, talk, let go: the transcript goes straight to FRANK. It never touches the text
// box, so the phone keyboard stays closed. Web Speech where the browser has it, an ElevenLabs
// round trip through the server where it does not.

const Recognition = window.SpeechRecognition || window.webkitSpeechRecognition;

let recognition = null;
let recorder = null;
let chunks = [];
let holding = false;
let hintTimer = null;

function hint(msg, revertMs = 0) {
  clearTimeout(hintTimer);
  $("voice-hint").textContent = msg;
  if (revertMs) hintTimer = setTimeout(() => hint("Hold to talk"), revertMs);
}

function listening(on) {
  $("mic").classList.toggle("listening", on);
  $("mic").textContent = on ? "Release to send" : "Hold to talk";
  hint(on ? "Listening… let go when you're done" : "Hold to talk");
}

async function sendHeard(text) {
  text = text.trim();
  if (!text) return hint("Didn't catch that. Hold to talk", 3000);
  try {
    await send(text);
    hint(`Sent: “${text}”`, 3000);
  } catch (err) {
    hint("Could not send. Try again", 3000);
  }
}

// tier 1: the browser's own recognizer
function startWebSpeech() {
  recognition = new Recognition();
  recognition.lang = document.documentElement.lang || navigator.language || "en-US";
  recognition.continuous = false;
  recognition.interimResults = true;
  let heard = "";
  recognition.onresult = (e) => {
    heard = Array.from(e.results).map((r) => r[0].transcript).join("").trim();
    if (holding) hint(`Hearing: ${heard}`);   // live transcript in the hint, not the text box
  };
  recognition.onerror = (e) => {
    recognition = null;
    if (holding) startRecorder(e.error);   // fall back mid-hold
  };
  recognition.onend = () => { if (!holding) sendHeard(heard); };
  recognition.start();
}

// tier 2: record and let the server transcribe
async function startRecorder(why) {
  if (!navigator.mediaDevices || !window.MediaRecorder) return voiceUnavailable();
  try {
    const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    chunks = [];
    recorder = new MediaRecorder(stream);
    recorder.ondataavailable = (e) => e.data.size && chunks.push(e.data);
    recorder.onstop = async () => {
      stream.getTracks().forEach((t) => t.stop());
      await transcribe(new Blob(chunks, { type: recorder.mimeType || "audio/webm" }));
      recorder = null;
    };
    recorder.start();
    if (holding) listening(true);
  } catch (err) {
    voiceUnavailable();
  }
}

async function transcribe(blob) {
  hint("Transcribing…");
  const form = new FormData();
  form.append("audio", blob, "speech." + (blob.type.includes("mp4") ? "mp4" : "webm"));
  try {
    const res = await fetch(`/api/people/${store.id}/transcribe`, { method: "POST", body: form });
    if (res.status === 503) return voiceUnavailable();
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const data = await res.json();
    await sendHeard(data.text || "");
  } catch (err) {
    hint("Voice failed. Type instead", 3000);
  }
}

function voiceUnavailable() {
  $("mic").disabled = true;
  hint("Voice unavailable here, type instead");
}

function pressMic(e) {
  e.preventDefault();
  if ($("mic").disabled || holding) return;
  holding = true;
  listening(true);
  if (Recognition) startWebSpeech();
  else startRecorder("no Web Speech API");
}

function releaseMic() {
  if (!holding) return;
  holding = false;
  listening(false);
  if (recognition) { try { recognition.stop(); } catch (e) {} }
  if (recorder && recorder.state === "recording") recorder.stop();
}

$("mic").addEventListener("pointerdown", pressMic);
$("mic").addEventListener("pointerup", releaseMic);
$("mic").addEventListener("pointercancel", releaseMic);
$("mic").addEventListener("pointerleave", releaseMic);
// older iOS Safari has no pointer events on buttons
$("mic").addEventListener("touchstart", pressMic, { passive: false });
$("mic").addEventListener("touchend", releaseMic);
$("mic").addEventListener("contextmenu", (e) => e.preventDefault());

if (!Recognition && !(navigator.mediaDevices && window.MediaRecorder)) voiceUnavailable();
