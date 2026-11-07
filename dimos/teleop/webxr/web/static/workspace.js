const $ = (id) => document.getElementById(id), frame = $("scene");
let state = null,
  selected = null,
  busy = false,
  stopped = false,
  driveSocket = null,
  driveTimer = null,
  sequence = 0,
  driveGeneration = 0;
async function request(path, body) {
  const r = await fetch(`spatial/${path}`, {
    cache: "no-store",
    ...(body
      ? {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      }
      : {}),
  });
  if (!r.ok) {
    const e = await r.json().catch(() => ({}));
    throw Error(e.detail || `HTTP ${r.status}`);
  }
  return r.json();
}
function error(e) {
  $("error").hidden = !e;
  $("error").textContent = e?.message || "";
}
const model = await request("model");
document.title = model.title;
$("title").textContent = model.title;
$("environment").textContent = model.environment;
const session = new URLSearchParams(location.search).get("operator_session");
frame.src = `spatial?embedded=1&desktop=1${
  session ? `&operator_session=${encodeURIComponent(session)}` : ""
}`;
function scene(action, value) {
  frame.contentWindow?.postMessage(
    { type: `dimos.spatial.${action}`, value },
    location.origin,
  );
}
function select(id) {
  selected = id;
  for (const b of $("arms").children) {
    b.classList.toggle("selected", b.dataset.id === id);
  }
  scene("selection", id);
  $("joints").replaceChildren();
  $("joint-controls").hidden = !model.joint_jog_available;
  const group = model.groups[id];
  $("gripper").textContent = group.gripper_joint
    ? "Gripper configured"
    : "Gripper actuation unavailable";
  group.joint_names.forEach((name, i) => {
    const row = document.createElement("div");
    row.className = "joint";
    const label = document.createElement("label");
    label.textContent = `Joint ${i + 1}`;
    label.title = name;
    const out = document.createElement("output");
    out.dataset.joint = name;
    row.append(label, out);
    for (const [symbol, sign] of [["−", -1], ["+", 1]]) {
      const b = document.createElement("button");
      b.textContent = symbol;
      b.onclick = () => jog(name, sign);
      row.append(b);
    }
    $("joints").append(row);
  });
}
for (const [id, group] of Object.entries(model.groups)) {
  const b = document.createElement("button");
  b.textContent = group.label;
  b.dataset.id = id;
  b.onclick = () => select(id);
  $("arms").append(b);
}
if (Object.keys(model.groups).length) select(Object.keys(model.groups)[0]);
else {
  $("arms").textContent = "No arms configured";
  $("joints").hidden = true;
}
async function action(action) {
  if (!state) return;
  error(null);
  try {
    state = await request("control", {
      action,
      session: state.control.session,
      epoch: state.control.epoch,
    });
  } catch (e) {
    error(e);
  }
  update();
}
$("hold").onclick = () => {
  stopDrive();
  void action("hold");
};
$("resume").onclick = () => action("resume");
async function jog(joint, sign) {
  if (busy || !state?.control.robot_ready) return;
  busy = true;
  try {
    const [lo, hi] = model.limits[joint] || [];
    const value = (state.joints[joint] || 0) + sign * Number($("step").value);
    if (lo !== null && value < lo || hi !== null && value > hi) {
      throw Error("Joint limit reached");
    }
    await request("jog", {
      session: state.control.session,
      epoch: state.control.epoch,
      joint,
      value,
    });
    state = await request("state");
    error(null);
  } catch (e) {
    error(e);
  } finally {
    busy = false;
    update();
  }
}
function update() {
  if (!state) return;
  $("connection").textContent = state.control.robot_ready
    ? "● Connected"
    : "Waiting for feedback";
  $("resume").disabled = !state.control.paused || !state.control.robot_ready;
  $("control-state").textContent = state.control.paused
    ? "Ⅱ Held · release and re-grip after enabling"
    : "Headset control enabled";
  $("surface-state").textContent = state.surface_age < 1
    ? "Live RGB-D surface · sensor observations"
    : state.observation_error || "Surface delayed";
  for (const out of document.querySelectorAll("output[data-joint]")) {
    out.textContent = `${
      ((state.joints[out.dataset.joint] || 0) * 180 / Math.PI).toFixed(1)
    }°`;
  }
  for (const b of $("joints").querySelectorAll("button")) {
    b.disabled = busy || !state.control.robot_ready;
  }
}
async function poll() {
  while (!stopped) {
    try {
      const next = await request("state");
      if (session && next.control.session !== session) {
        throw Error("Blueprint restarted. Reopen the workspace.");
      }
      state = next;
      update();
    } catch (e) {
      error(e);
    }
    await new Promise((r) => setTimeout(r, 100));
  }
}
void poll();
for (const [id, title] of Object.entries(model.cameras)) {
  const card = document.createElement("div");
  card.className = "camera";
  const img = document.createElement("img");
  img.alt = title;
  const caption = document.createElement("figcaption"),
    name = document.createElement("span"),
    status = document.createElement("span");
  name.textContent = title;
  status.className = "camera-status";
  caption.append(name, status);
  card.append(img, caption);
  $("cameras").append(card);
  let url;
  async function load() {
    while (!stopped) {
      try {
        const r = await fetch(`spatial/camera/${id}`, { cache: "no-store" });
        if (!r.ok) throw Error();
        const next = URL.createObjectURL(await r.blob());
        img.src = next;
        if (url) URL.revokeObjectURL(url);
        url = next;
        status.textContent = "● Live";
      } catch {
        status.textContent = "Waiting";
      }
      await new Promise((r) => setTimeout(r, 150));
    }
  }
  void load();
}
for (const b of document.querySelectorAll("[data-view]")) {
  b.onclick = () => scene("view", b.dataset.view);
}
for (const id of ["surface", "map"]) {
  $(id).onchange = () => scene("layer", { name: id, visible: $(id).checked });
}
$("headset").onclick = () => {
  $("headset-links").hidden = !$("headset-links").hidden;
};
$("host").value = localStorage.getItem("spatial-headset-host") ||
  model.headset_hosts?.[0] || location.hostname;
function headsetLink() {
  const host = $("host").value.trim();
  if (!/^[a-zA-Z0-9.:-]+$/.test(host)) return;
  localStorage.setItem("spatial-headset-host", host);
  const url = new URL(`https://${host}:${model.headset_port}/spatial`);
  if (session) url.searchParams.set("operator_session", session);
  $("headset-url").href = url.href;
  $("headset-url").textContent = url.href;
}
$("host").oninput = headsetLink;
headsetLink();
$("drive-panel").hidden = !model.base_drive;
function stopDrive() {
  driveGeneration++;
  clearInterval(driveTimer);
  driveTimer = null;
  driveSocket?.close();
  driveSocket = null;
}
async function drive(axes) {
  stopDrive();
  const generation = driveGeneration;
  if (!state?.control.robot_ready) return;
  await action("resume");
  if (generation !== driveGeneration || state.control.paused) return;
  const url = new URL("ws", location.href);
  url.protocol = location.protocol === "https:" ? "wss:" : "ws:";
  const ws = new WebSocket(url);
  driveSocket = ws;
  ws.onopen = () => {
    if (generation !== driveGeneration) {
      ws.close();
      return;
    }
    sequence = 0;
    driveTimer = setInterval(() => {
      if (ws.readyState !== 1) return;
      ws.send(JSON.stringify({
        type: "spatial_input",
        session: state.control.session,
        control_epoch: state.control.epoch,
        sequence: sequence++,
        observed_at: state.ts,
        mode: state.control.mode,
        hands: {},
        drive: axes,
      }));
    }, 40);
  };
  ws.onclose = () => {
    if (driveSocket === ws) stopDrive();
  };
}
for (const b of document.querySelectorAll("[data-drive]")) {
  b.onpointerdown = (e) => {
    b.setPointerCapture(e.pointerId);
    void drive(b.dataset.drive.split(",").map(Number));
  };
  b.onpointerup = stopDrive;
  b.onpointercancel = stopDrive;
  b.onlostpointercapture = stopDrive;
}
$("drive-stop").onclick = stopDrive;
addEventListener("blur", stopDrive);
addEventListener("pagehide", () => {
  stopped = true;
  stopDrive();
});
addEventListener("keydown", (e) => {
  if (e.code === "Space" && !e.target.closest("input,select")) {
    e.preventDefault();
    stopDrive();
    void action("hold");
  }
});
addEventListener("message", (e) => {
  if (
    e.source === frame.contentWindow && e.origin === location.origin &&
    e.data?.type === "dimos.spatial.select" && model.groups[e.data.value]
  ) select(e.data.value);
  if (e.source === parent && e.data?.type === "dimos.theme") {
    for (const [key, value] of Object.entries(e.data.tokens || {})) {
      if (key.startsWith("--")) {
        document.documentElement.style.setProperty(key, value);
      }
    }
  }
});
