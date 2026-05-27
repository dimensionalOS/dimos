const state = {
  drafts: [],
  camera: {},
  robot: {},
  map: {},
  x: {},
  mentionPoller: {},
  mentions: [],
  peopleTags: [],
  filter: "all",
  view: "tweets",
  polling: null,
  pollCount: 0,
  bootId: Math.random().toString(36).slice(2, 6).toUpperCase(),
  lastCaptureSeen: null,
  drive: {
    button: null,
    interval: null,
    preset: null,
    move: null,
    busy: false,
    key: null,
    stopQueued: false,
  },
};

const els = {
  statusCamera: document.querySelector("#statusCamera"),
  statusRobot: document.querySelector("#statusRobot"),
  statusFrame: document.querySelector("#statusFrame"),
  statusAction: document.querySelector("#statusAction"),
  modeBtn: document.querySelector("#modeBtn"),
  xModeBtn: document.querySelector("#xModeBtn"),
  commandDebug: document.querySelector("#commandDebug"),
  commandStream: document.querySelector("#commandStream"),
  pageBeat: document.querySelector("#pageBeat"),
  cameraDot: document.querySelector("#cameraDot"),
  liveFrame: document.querySelector("#liveFrame"),
  liveCaption: document.querySelector("#liveCaption"),
  recCorner: document.querySelector("#recCorner"),
  mapHudTitle: document.querySelector("#mapHudTitle"),
  mapHudAge: document.querySelector("#mapHudAge"),
  mapHudCanvas: document.querySelector("#mapHudCanvas"),
  captureFlash: document.querySelector("#captureFlash"),
  emptyState: document.querySelector("#emptyState"),
  startBtn: document.querySelector("#startBtn"),
  captureBtn: document.querySelector("#captureBtn"),
  stopBtn: document.querySelector("#stopBtn"),
  emergencyStopBtn: document.querySelector("#emergencyStopBtn"),
  standBtn: document.querySelector("#standBtn"),
  lieDownBtn: document.querySelector("#lieDownBtn"),
  patrolBtn: document.querySelector("#patrolBtn"),
  cancelPatrolBtn: document.querySelector("#cancelPatrolBtn"),
  skillRow: document.querySelector(".skill-row"),
  dpad: document.querySelector(".dpad"),
  drafts: document.querySelector("#drafts"),
  draftCount: document.querySelector("#draftCount"),
  dispatchTabs: document.querySelector(".dispatch-tabs"),
  viewTabs: document.querySelector(".view-tabs"),
  panels: document.querySelectorAll(".panel-view"),
  xStatus: document.querySelector("#xStatus"),
  mentionCount: document.querySelector("#mentionCount"),
  mentionSettingsForm: document.querySelector("#mentionSettingsForm"),
  mentionInterval: document.querySelector("#mentionInterval"),
  mentionPollBtn: document.querySelector("#mentionPollBtn"),
  mentionInjectForm: document.querySelector("#mentionInjectForm"),
  mentionAuthor: document.querySelector("#mentionAuthor"),
  mentionText: document.querySelector("#mentionText"),
  mentionStatus: document.querySelector("#mentionStatus"),
  mentions: document.querySelector("#mentions"),
  peopleTagForm: document.querySelector("#peopleTagForm"),
  peopleHandle: document.querySelector("#peopleHandle"),
  peopleNotes: document.querySelector("#peopleNotes"),
  peopleImage: document.querySelector("#peopleImage"),
  peopleTags: document.querySelector("#peopleTags"),
  peopleTagCount: document.querySelector("#peopleTagCount"),
};

const movePresets = {
  forward: { forward: 0.5, duration: 0.05 },
  back: { forward: -0.5, duration: 0.05 },
  left: { lateral: 0.5, duration: 0.05 },
  right: { lateral: -0.5, duration: 0.05 },
  "turn-left": { yaw: 0.8, duration: 0.05 },
  "turn-right": { yaw: -0.8, duration: 0.05 },
};

const FRESH_THRESHOLD_SEC = 30;
const DRIVE_INTERVAL_MS = 90;
const DEFAULT_TAG_HANDLES = [
  "@bartomolina",
  "@stash_pomichter",
  "@themu_xyz",
  "@UnitreeRobotics",
  "@dimensionalos",
];

function dpadButtonForMove(move) {
  return document.querySelector(`button[data-move="${CSS.escape(move)}"]`);
}

function toast(message) {
  const node = document.createElement("div");
  node.className = "toast";
  node.textContent = message;
  document.body.appendChild(node);
  window.setTimeout(() => {
    node.style.transition = "opacity 220ms ease-out, transform 220ms ease-out";
    node.style.opacity = "0";
    node.style.transform = "translateX(-50%) translateY(8px)";
  }, 2200);
  window.setTimeout(() => node.remove(), 2600);
}

async function request(path, options = {}) {
  const response = await fetch(path, options);
  if (!response.ok) {
    let detail = response.statusText;
    try {
      detail = (await response.json()).detail || detail;
    } catch {
      // keep status text
    }
    throw new Error(detail);
  }
  return response.json();
}

async function refresh() {
  state.pollCount += 1;
  const data = await request("/api/status");
  state.camera = data.camera;
  state.robot = data.robot;
  state.map = data.map || {};
  state.x = data.x || {};
  state.mentionPoller = data.mention_poller || {};
  state.mentions = data.mentions || [];
  state.peopleTags = data.people_tags || [];

  const previousIds = new Set(state.drafts.map((d) => d.id));
  state.drafts = data.drafts;
  const newest = state.drafts[0];
  if (newest && !previousIds.has(newest.id) && state.lastCaptureSeen !== null) {
    triggerCaptureFlash();
  }
  state.lastCaptureSeen = newest ? newest.id : null;

  renderStatus();
  renderMentions();
  renderPeopleTags();
  renderPanelView();
  renderCommandStream();
  renderFrame();
  renderMapHud();
  if (!isDraftInteractionActive()) {
    renderDispatches();
  }
  renderPageBeat("poll");
}

function formatAge(seconds) {
  if (seconds == null) return "—";
  if (seconds < 1) return "fresh";
  if (seconds < 60) return `${seconds.toFixed(0)}s`;
  if (seconds < 3600) return `${Math.round(seconds / 60)}m`;
  return `${Math.round(seconds / 3600)}h`;
}

function fmt(value) {
  return Number(value || 0).toFixed(2);
}

function renderPageBeat(stateName = "idle") {
  if (!els.pageBeat) return;
  const drive = state.drive.move ? state.drive.move.toUpperCase() : "";
  els.pageBeat.dataset.state = state.drive.move ? "drive" : stateName;
  els.pageBeat.textContent = state.drive.move
    ? `DRIVE ${drive} P${state.pollCount} ${state.bootId}`
    : `P${state.pollCount} ${state.bootId}`;
}

function renderStatus() {
  const camera = state.camera;
  const robot = state.robot || {};

  els.statusRobot.textContent = `${robot.mock ? "mock" : "robot"} @ ${camera.robot_ip || "192.168.12.1"}`;
  els.modeBtn.textContent = camera.mock ? "MOCK" : "LIVE";
  els.modeBtn.dataset.mode = camera.mock ? "mock" : "live";

  let camText, dotState;
  if (camera.connected) {
    camText = camera.mock ? "mock camera" : "camera connected";
    dotState = "ok";
  } else if (camera.connecting) {
    camText = "camera connecting…";
    dotState = "warn";
  } else if (camera.last_error) {
    camText = `camera error: ${camera.last_error}`;
    dotState = "bad";
  } else {
    camText = "camera idle";
    dotState = "off";
  }
  els.statusCamera.textContent = camText;
  els.cameraDot.dataset.state = dotState;

  const age = camera.frame_age_sec;
  const fresh = age != null && age <= FRESH_THRESHOLD_SEC && camera.has_frame;
  if (camera.has_frame) {
    els.statusFrame.textContent = fresh ? `frame ${formatAge(age)}` : `frame stale · ${formatAge(age)} old`;
  } else {
    els.statusFrame.textContent = "no frame";
  }

  els.statusAction.textContent = robot.last_error
    ? `error: ${robot.last_error}`
    : robot.patrol_active
      ? `patrol ${robot.patrol_step_index || 0}/${robot.patrol_step_total || "?"}: ${robot.patrol_step || "running"}`
      : `last action: ${robot.last_action || "idle"}`;

  if (els.commandDebug) {
    const cmd = robot.last_command;
    if (!cmd) {
      els.commandDebug.textContent = "CMD —";
    } else if (cmd.kind === "move" || cmd.kind === "drive") {
      const wc = cmd.wireless_controller || {};
      const sport = (cmd.sport_payload && cmd.sport_payload.parameter) || {};
      const req = cmd.request || {};
      els.commandDebug.textContent =
        `CMD ${cmd.kind} f:${fmt(req.forward)} l:${fmt(req.lateral)} y:${fmt(req.yaw)} ` +
        `→ x:${fmt(sport.x)} y:${fmt(sport.y)} z:${fmt(sport.z)} ` +
        `lx:${fmt(wc.lx)} ly:${fmt(wc.ly)} rx:${fmt(wc.rx)} ok:${cmd.result}`;
    } else if (cmd.kind === "sport_move") {
      const req = cmd.request || {};
      els.commandDebug.textContent =
        `CMD sport f:${fmt(req.forward)} l:${fmt(req.lateral)} y:${fmt(req.yaw)} ` +
        `ok:${Boolean(cmd.response)} stop:${Boolean(cmd.stop_response)}`;
    } else {
      els.commandDebug.textContent = `CMD ${cmd.kind}`;
    }
  }

  // rec corner state
  if (camera.has_frame && camera.connected && fresh) {
    els.recCorner.dataset.state = "live";
    els.recCorner.textContent = "● REC";
  } else if (camera.connecting) {
    els.recCorner.dataset.state = "warn";
    els.recCorner.textContent = "○ link…";
  } else if (camera.has_frame && !fresh) {
    els.recCorner.dataset.state = "warn";
    els.recCorner.textContent = "○ stale";
  } else {
    els.recCorner.dataset.state = "off";
  }

  // live caption changes based on state
  if (camera.connected && fresh) {
    els.liveCaption.textContent = "kora is looking around";
  } else if (camera.connecting) {
    els.liveCaption.textContent = "waking up the photodog…";
  } else if (camera.has_frame && !fresh) {
    els.liveCaption.textContent = "last seen — feed went quiet";
  } else if (camera.last_error) {
    els.liveCaption.textContent = "the camera had a moment";
  } else {
    els.liveCaption.textContent = "offline — waiting on a signal";
  }

  els.captureBtn.disabled = !camera.has_frame || !fresh;
  els.startBtn.disabled = camera.connected || camera.connecting;
  els.stopBtn.disabled = !camera.connected && !camera.connecting;

  const remoteButtons = document.querySelectorAll(".ctrl-box button:not(#captureBtn), .remote-card button");
  remoteButtons.forEach((button) => {
    button.disabled = !robot.ready;
  });
  els.emergencyStopBtn.disabled = !robot.ready;
  els.standBtn.disabled = !robot.ready;
  els.lieDownBtn.disabled = !robot.ready;
  els.patrolBtn.disabled = !robot.ready || robot.patrol_active;
  els.cancelPatrolBtn.disabled = !robot.ready || !robot.patrol_active;

  const x = state.x || {};
  els.xModeBtn.textContent = x.dry_run ? "X DRY" : "X LIVE";
  els.xModeBtn.dataset.mode = x.dry_run ? "dry" : "live";
  els.xModeBtn.disabled = !x.installed && x.dry_run;
  if (x.dry_run) {
    els.xStatus.textContent = x.installed ? "X: DRY" : "X: DRY / NO XURL";
  } else if (x.authenticated) {
    els.xStatus.textContent = x.username ? `X: LIVE @${x.username}` : "X: LIVE";
  } else if (x.installed) {
    els.xStatus.textContent = "X: AUTH NEEDED";
  } else {
    els.xStatus.textContent = "X: NO XURL";
  }
  els.xStatus.title = x.detail || "";
}

function renderCommandStream() {
  if (!els.commandStream) return;
  const commands = Array.isArray(state.robot.command_log) ? state.robot.command_log.slice(0, 8) : [];
  els.commandStream.hidden = false;
  if (!commands.length) {
    els.commandStream.innerHTML = `<div class="command-line">${state.robot.mock ? "MOCK" : "LIVE"} READY / NO COMMANDS</div>`;
    return;
  }
  els.commandStream.innerHTML = commands
    .map((cmd) => {
      const req = cmd.request || {};
      const wc = cmd.wireless_controller || {};
      const sport = (cmd.sport_payload && cmd.sport_payload.parameter) || {};
      const bits = [
        `f:${fmt(req.forward)}`,
        `l:${fmt(req.lateral)}`,
        `y:${fmt(req.yaw)}`,
        `x:${fmt(sport.x)}`,
        `sy:${fmt(sport.y)}`,
        `z:${fmt(sport.z)}`,
        `lx:${fmt(wc.lx)}`,
        `ly:${fmt(wc.ly)}`,
        `rx:${fmt(wc.rx)}`,
        cmd.delta_ms == null ? "first" : `+${cmd.delta_ms}ms`,
      ];
      return `<div class="command-line">${escapeHtml(cmd.kind)} <span>${escapeHtml(bits.join(" "))} ok:${escapeHtml(String(cmd.result ?? cmd.response ?? ""))}</span></div>`;
    })
    .join("");
}

function renderFrame() {
  const camera = state.camera;
  const age = camera.frame_age_sec;
  const fresh = age != null && age <= FRESH_THRESHOLD_SEC && camera.has_frame;
  if (!camera.has_frame || !fresh) {
    els.liveFrame.style.display = "none";
    els.emptyState.style.display = "flex";
    return;
  }
  els.emptyState.style.display = "none";
  els.liveFrame.style.display = "block";
  els.liveFrame.src = `/api/frame.jpg?t=${Date.now()}`;
}

function renderMapHud() {
  const canvas = els.mapHudCanvas;
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  const map = state.map || {};
  const connected = Boolean(map.connected);
  const path = map.path && Array.isArray(map.path.points) ? map.path.points : [];
  const pose = map.robot_pose && Array.isArray(map.robot_pose.c) ? map.robot_pose.c : null;
  const points = mapPoints(map);

  els.mapHudTitle.textContent = connected ? "MAP 3D" : "MAP WAITING";
  els.mapHudAge.textContent = connected && map.age_sec != null ? formatAge(map.age_sec) : "--";

  ctx.clearRect(0, 0, w, h);
  const gradient = ctx.createLinearGradient(0, 0, 0, h);
  gradient.addColorStop(0, "#05070c");
  gradient.addColorStop(0.62, "#10131c");
  gradient.addColorStop(1, "#06070a");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, w, h);

  const origin = pose ? [Number(pose[0]) || 0, Number(pose[1]) || 0] : [0, 0];
  const heading = pose ? Number(pose[2]) || 0 : 0;
  const scale = Math.min(w / 8.2, h / 5.6);
  const project3d = (point) => {
    const px = Number(point[0]) - origin[0];
    const py = Number(point[1]) - origin[1];
    const pz = Number(point[2]) || 0;
    const c = Math.cos(-heading * 0.35);
    const s = Math.sin(-heading * 0.35);
    const rx = px * c - py * s;
    const ry = px * s + py * c;
    return [
      w / 2 + (rx - ry) * scale * 0.72,
      h * 0.68 + (rx + ry) * scale * 0.28 - pz * scale * 1.1,
      rx + ry,
    ];
  };

  drawMapGrid(ctx, w, h, project3d);

  if (path.length > 1) {
    ctx.strokeStyle = "rgba(255, 213, 0, 0.9)";
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    path.slice(-48).forEach((point, index) => {
      const [x, y] = project3d([point[0], point[1], 0.08]);
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  points
    .slice()
    .sort((a, b) => project3d(b)[2] - project3d(a)[2])
    .slice(0, 900)
    .forEach((point) => {
      const [x, y] = project3d(point);
      if (x < -4 || x > w + 4 || y < -4 || y > h + 4) return;
      const z = Number(point[2]) || 0;
      const intensity = Number(point[3]) || Math.min(1, Math.max(0.25, z / 1.3));
      ctx.fillStyle = pointColor(z, intensity);
      ctx.globalAlpha = 0.78 + Math.min(0.2, intensity * 0.2);
      ctx.fillRect(x - 1.1, y - 1.1, 2.2, 2.2);
    });
  ctx.globalAlpha = 1;

  if (connected) {
    drawRobotCube(ctx, w, h);
  }

  if (!connected) {
    ctx.fillStyle = "#9aa2b2";
    ctx.font = "8px Space Mono, monospace";
    ctx.textAlign = "center";
    ctx.fillText("start DimOS map server :7779", w / 2, h - 8);
  } else if (!points.length) {
    ctx.fillStyle = "#9aa2b2";
    ctx.font = "8px Space Mono, monospace";
    ctx.textAlign = "center";
    ctx.fillText("waiting for point cloud", w / 2, h - 8);
  }
}

function mapPoints(map) {
  const cloud = map.pointcloud;
  if (cloud && Array.isArray(cloud.points)) {
    return cloud.points
      .filter((point) => Array.isArray(point) && point.length >= 2)
      .map((point) => [Number(point[0]) || 0, Number(point[1]) || 0, Number(point[2]) || 0, Number(point[3]) || 0]);
  }

  const objects = Array.isArray(map.objects) ? map.objects : [];
  return objects
    .filter((obj) => Array.isArray(obj.c))
    .map((obj) => [Number(obj.c[0]) || 0, Number(obj.c[1]) || 0, Number(obj.c[2]) || 0.18, 0.75]);
}

function drawMapGrid(ctx, w, h, project3d) {
  ctx.save();
  ctx.strokeStyle = "rgba(155, 165, 190, 0.24)";
  ctx.lineWidth = 1;
  for (let v = -4; v <= 4; v += 0.5) {
    const [x1, y1] = project3d([-4, v, 0]);
    const [x2, y2] = project3d([4, v, 0]);
    const [x3, y3] = project3d([v, -3, 0]);
    const [x4, y4] = project3d([v, 3, 0]);
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.moveTo(x3, y3);
    ctx.lineTo(x4, y4);
    ctx.stroke();
  }
  ctx.restore();
}

function pointColor(z, intensity) {
  if (z < 0.08) return `rgba(79, 117, 236, ${0.48 + intensity * 0.28})`;
  if (z < 0.35) return `rgba(0, 215, 174, ${0.55 + intensity * 0.32})`;
  if (z < 0.85) return `rgba(255, 224, 78, ${0.58 + intensity * 0.34})`;
  return `rgba(255, 91, 38, ${0.62 + intensity * 0.34})`;
}

function drawRobotCube(ctx, w, h) {
  const cx = w / 2;
  const cy = h * 0.58;
  ctx.save();
  ctx.strokeStyle = "#00e676";
  ctx.fillStyle = "rgba(0, 230, 118, 0.18)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(cx, cy - 11);
  ctx.lineTo(cx + 12, cy - 5);
  ctx.lineTo(cx + 12, cy + 8);
  ctx.lineTo(cx, cy + 14);
  ctx.lineTo(cx - 12, cy + 8);
  ctx.lineTo(cx - 12, cy - 5);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(cx, cy - 11);
  ctx.lineTo(cx, cy + 2);
  ctx.moveTo(cx - 12, cy - 5);
  ctx.lineTo(cx, cy + 2);
  ctx.lineTo(cx + 12, cy - 5);
  ctx.stroke();
  ctx.restore();
}

function triggerCaptureFlash() {
  els.captureFlash.classList.remove("flash");
  void els.captureFlash.offsetWidth;
  els.captureFlash.classList.add("flash");
}

function relativeTime(value) {
  if (!value) return "";
  const created = new Date(value.replace(" ", "T"));
  if (Number.isNaN(created.getTime())) return value;
  const diffSec = Math.max(0, (Date.now() - created.getTime()) / 1000);
  if (diffSec < 45) return "just now";
  if (diffSec < 90) return "a minute ago";
  if (diffSec < 3600) return `${Math.round(diffSec / 60)} min ago`;
  if (diffSec < 7200) return "an hour ago";
  if (diffSec < 86400) return `${Math.round(diffSec / 3600)} hrs ago`;
  return created.toLocaleDateString();
}

function stars(score) {
  const total = 5;
  const filled = Math.max(0, Math.min(total, Math.round(score * total)));
  return "★".repeat(filled) + "☆".repeat(total - filled);
}

function renderDispatches() {
  const all = state.drafts;
  const visible = state.filter === "all" ? all : all.filter((d) => d.status === state.filter);
  els.draftCount.textContent = `TWEETS ${visible.length}/${all.length}`;

  if (!visible.length) {
    const copy =
      state.filter === "all"
        ? { hand: "no dispatches yet", sub: "capture a moment to start the column." }
        : { hand: `no ${state.filter} yet`, sub: "try another tab or capture more." };
    els.drafts.innerHTML = `
      <div class="dispatch-empty">
        <span class="caveat">${copy.hand}</span>
        <span>${copy.sub}</span>
      </div>`;
    return;
  }

  els.drafts.innerHTML = visible.map(renderDispatch).join("");
}

function renderDispatch(draft) {
  const status = draft.status || "draft";
  const rel = relativeTime(draft.created_at);
  const score = draft.score;
  const showStamp = status === "posted" || status === "draft";
  const postDisabled = status === "posted" || status === "posting" || status === "failed";
  const editDisabled = status === "posted" || status === "posting";
  const postLabel = status === "posting" ? "posting…" : status === "failed" ? "fix text" : "post →";
  const postMeta = draft.x_post_url
    ? `<a class="post-link" href="${escapeHtml(draft.x_post_url)}" target="_blank" rel="noreferrer">open on x</a>`
    : "";
  const errorMeta = draft.post_error ? `<p class="dispatch-error">${escapeHtml(draft.post_error)}</p>` : "";
  const tagHandles = uniqueHandles([
    ...DEFAULT_TAG_HANDLES,
    ...state.peopleTags.filter((tag) => tag.enabled).map((tag) => tag.handle),
  ]);
  const tagDisabled = editDisabled || !tagHandles.length;
  const tagOptions = tagHandles
    .map((handle) => `<button data-action="add-tag" data-id="${draft.id}" data-handle="${escapeHtml(handle)}">${escapeHtml(handle)}</button>`)
    .join("");

  return `
    <article class="dispatch" data-status="${status}">
      ${showStamp || status === "posting" || status === "failed" ? `<span class="dispatch-stamp" data-stamp="${status}">${status}</span>` : ""}
      <div class="dispatch-photo">
        <img src="/captures/${draft.filename}" alt="Kora capture" loading="lazy" />
      </div>
      <div class="dispatch-body">
        <textarea
          class="dispatch-caption-input"
          data-caption-id="${draft.id}"
          maxlength="280"
          ${status === "posted" || status === "posting" ? "disabled" : ""}
        >${escapeHtml(draft.caption)}</textarea>
        <div class="dispatch-meta">
          <span>${escapeHtml(rel)}</span>
          <span>·</span>
          <span class="stars" title="vibe score ${Math.round(score * 100)}%">${stars(score)}</span>
          ${postMeta ? `<span>·</span>${postMeta}` : ""}
        </div>
        <p class="dispatch-reason">${escapeHtml(draft.reason)}</p>
        ${errorMeta}
        <div class="dispatch-actions">
          <div class="tag-menu" data-tags-for="${draft.id}">
            <button data-action="toggle-tags" data-id="${draft.id}" ${tagDisabled ? "disabled" : ""} title="Add tag">tag</button>
            <div class="tag-options" hidden>
              ${tagOptions}
            </div>
          </div>
          <button class="act-save" data-action="save-caption" data-id="${draft.id}" ${editDisabled ? "disabled" : ""}>save text</button>
          <button class="act-skip" data-action="delete" data-id="${draft.id}">skip</button>
          <button class="act-post" data-action="post" data-id="${draft.id}" ${postDisabled ? "disabled" : ""}>${postLabel}</button>
        </div>
      </div>
    </article>
  `;
}

function uniqueHandles(handles) {
  const seen = new Set();
  const unique = [];
  handles.forEach((handle) => {
    const clean = String(handle || "").trim();
    const key = clean.toLowerCase();
    if (!clean || seen.has(key)) return;
    seen.add(key);
    unique.push(clean);
  });
  return unique;
}

function renderPeopleTags() {
  if (!els.peopleTags) return;
  const tags = state.peopleTags || [];
  const enabled = tags.filter((tag) => tag.enabled).length;
  els.peopleTagCount.textContent = `PEOPLE ${enabled}/${tags.length}`;
  if (!tags.length) {
    els.peopleTags.innerHTML = `
      <div class="people-empty">
        <span class="caveat">no opt-in tags</span>
        <span>add a handle and reference image.</span>
      </div>`;
    return;
  }
  els.peopleTags.innerHTML = tags
    .map((tag) => {
      const nextAction = tag.enabled ? "disable-person" : "enable-person";
      const nextLabel = tag.enabled ? "off" : "on";
      return `
        <article class="person-tag" data-enabled="${tag.enabled ? "true" : "false"}">
          <img src="/people-tags/${escapeHtml(tag.filename)}" alt="" loading="lazy" />
          <div class="person-tag-body">
            <strong>${escapeHtml(tag.handle)}</strong>
            <span>${escapeHtml(tag.notes || "no note")}</span>
            <div class="person-actions">
              <button data-action="${nextAction}" data-id="${tag.id}">${nextLabel}</button>
              <button data-action="delete-person" data-id="${tag.id}">del</button>
            </div>
          </div>
        </article>
      `;
    })
    .join("");
}

function renderPanelView() {
  els.viewTabs.querySelectorAll(".tab").forEach((tab) => {
    tab.classList.toggle("active", tab.dataset.view === state.view);
  });
  els.panels.forEach((panel) => {
    panel.classList.toggle("active", panel.dataset.panel === state.view);
  });
  els.peopleTagCount.hidden = state.view !== "people";
  els.mentionCount.hidden = state.view !== "mentions";
  els.draftCount.hidden = state.view !== "tweets";
}

function renderMentions() {
  if (!els.mentions) return;
  const mentions = state.mentions || [];
  const pending = mentions.filter((mention) => mention.action_status === "pending").length;
  els.mentionCount.textContent = `MENTIONS ${pending}/${mentions.length}`;

  const poller = state.mentionPoller || {};
  if (document.activeElement !== els.mentionInterval) {
    els.mentionInterval.value = poller.interval_sec || 0;
  }
  const bits = [];
  bits.push(poller.enabled ? `polling every ${poller.interval_sec}s` : "mentions paused");
  bits.push(poller.installed ? "xurl ready" : "xurl missing");
  if (poller.last_poll_at) bits.push(`last ${relativeTime(poller.last_poll_at)}`);
  if (poller.last_error) bits.push(`error: ${poller.last_error}`);
  els.mentionStatus.textContent = bits.join(" / ");

  if (!mentions.length) {
    els.mentions.innerHTML = `
      <div class="mention-empty">
        <span class="caveat">no mentions yet</span>
        <span>poll X or inject a local test.</span>
      </div>`;
    return;
  }

  els.mentions.innerHTML = mentions.map(renderMention).join("");
}

function renderMention(mention) {
  const command = mention.command_type || "unknown";
  const actionStatus = mention.action_status || "pending";
  const rel = relativeTime(mention.received_at);
  const canRun = actionStatus === "pending" && (command === "vision" || command === "capture");
  const url = mention.url
    ? `<a href="${escapeHtml(mention.url)}" target="_blank" rel="noreferrer">open</a>`
    : "";
  const reply = mention.reply_text ? `<p class="mention-reply">${escapeHtml(mention.reply_text)}</p>` : "";
  const error = mention.error ? `<p class="mention-error">${escapeHtml(mention.error)}</p>` : "";
  return `
    <article class="mention" data-action-status="${escapeHtml(actionStatus)}">
      <div class="mention-top">
        <strong>${escapeHtml(mention.author_handle || "@unknown")}</strong>
        <span>${escapeHtml(command)} / ${escapeHtml(actionStatus)} / ${escapeHtml(rel)}</span>
      </div>
      <p class="mention-text">${escapeHtml(mention.text)}</p>
      <p class="mention-command">${escapeHtml(mention.command_payload || "no command")}</p>
      ${reply}
      ${error}
      <div class="mention-actions">
        ${url}
        <button data-mention-action="run" data-id="${escapeHtml(mention.id)}" ${canRun ? "" : "disabled"}>run</button>
        <button data-mention-action="ignore" data-id="${escapeHtml(mention.id)}" ${actionStatus === "ignored" ? "disabled" : ""}>ignore</button>
        <button data-mention-action="delete" data-id="${escapeHtml(mention.id)}">del</button>
      </div>
    </article>
  `;
}

function isDraftInteractionActive() {
  const activeDraftTarget = document.activeElement && document.activeElement.closest("#drafts");
  const openTagMenu = els.drafts.querySelector(".tag-options:not([hidden])");
  return Boolean(activeDraftTarget || openTagMenu);
}

function captionInput(id) {
  return els.drafts.querySelector(`textarea[data-caption-id="${CSS.escape(id)}"]`);
}

function tagOptions(id) {
  return els.drafts.querySelector(`[data-tags-for="${CSS.escape(id)}"]`);
}

function closeTagMenus(except = null) {
  els.drafts.querySelectorAll(".tag-menu").forEach((menu) => {
    if (menu !== except) {
      const options = menu.querySelector(".tag-options");
      if (options) options.hidden = true;
    }
  });
}

function toggleTagMenu(id) {
  const menu = tagOptions(id);
  if (!menu) return;
  const options = menu.querySelector(".tag-options");
  if (!options) return;
  const nextHidden = !options.hidden;
  closeTagMenus(menu);
  options.hidden = nextHidden;
}

function appendHandle(id, handle) {
  const input = captionInput(id);
  if (!input || input.disabled) return;
  const current = input.value.trimEnd();
  if (current.includes(handle)) {
    input.focus();
    return;
  }
  input.value = current ? `${current} ${handle}` : handle;
  input.focus();
}

async function saveCaption(id) {
  const input = captionInput(id);
  if (!input) return;
  const caption = input.value.trim();
  if (!caption) {
    throw new Error("caption cannot be empty");
  }
  await request(`/api/drafts/${id}/caption`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ caption }),
  });
}

async function createPeopleTag(event) {
  event.preventDefault();
  const form = els.peopleTagForm;
  const file = els.peopleImage.files[0];
  if (!els.peopleHandle.value.trim()) {
    toast("handle required");
    return;
  }
  if (!file) {
    toast("image required");
    return;
  }
  const button = form.querySelector("button[type='submit']");
  button.disabled = true;
  try {
    const imageBase64 = await readFileAsDataUrl(file);
    await request("/api/people-tags", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        handle: els.peopleHandle.value.trim(),
        notes: els.peopleNotes.value.trim(),
        image_base64: imageBase64,
      }),
    });
    form.reset();
    toast("tag saved");
  } catch (error) {
    toast(error.message);
  } finally {
    button.disabled = false;
    await refresh();
  }
}

async function saveMentionSettings(event) {
  event.preventDefault();
  const intervalSec = Number.parseInt(els.mentionInterval.value || "0", 10);
  try {
    await request("/api/mentions/settings", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ interval_sec: Number.isFinite(intervalSec) ? intervalSec : 0 }),
    });
    toast(intervalSec > 0 ? "mention polling saved" : "mention polling paused");
  } catch (error) {
    toast(error.message);
  } finally {
    await refresh();
  }
}

async function pollMentions() {
  els.mentionPollBtn.disabled = true;
  try {
    await request("/api/mentions/poll", { method: "POST" });
    toast("mention poll complete");
  } catch (error) {
    toast(error.message);
  } finally {
    els.mentionPollBtn.disabled = false;
    await refresh();
  }
}

async function injectMention(event) {
  event.preventDefault();
  const text = els.mentionText.value.trim();
  if (!text) {
    toast("mention text required");
    return;
  }
  const button = els.mentionInjectForm.querySelector("button[type='submit']");
  button.disabled = true;
  try {
    await request("/api/mentions/inject", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        author_handle: els.mentionAuthor.value.trim() || "@demo",
        text,
      }),
    });
    els.mentionText.value = "";
    toast("mention injected");
  } catch (error) {
    toast(error.message);
  } finally {
    button.disabled = false;
    await refresh();
  }
}

async function mentionAction(event) {
  const button = event.target.closest("button[data-mention-action]");
  if (!button) return;
  const id = button.dataset.id;
  const action = button.dataset.mentionAction;
  button.disabled = true;
  try {
    if (action === "delete") {
      await request(`/api/mentions/${id}`, { method: "DELETE" });
      toast("mention deleted");
    } else {
      await request(`/api/mentions/${id}/${action}`, { method: "POST" });
      toast(action === "run" ? "mention handled" : "mention ignored");
    }
  } catch (error) {
    toast(error.message);
  } finally {
    await refresh();
  }
}

function readFileAsDataUrl(file) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => resolve(String(reader.result || ""));
    reader.onerror = () => reject(new Error("image read failed"));
    reader.readAsDataURL(file);
  });
}

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, (char) => {
    return {
      "&": "&amp;",
      "<": "&lt;",
      ">": "&gt;",
      '"': "&quot;",
      "'": "&#039;",
    }[char];
  });
}

async function startCamera() {
  els.startBtn.disabled = true;
  try {
    await request("/api/camera/start", { method: "POST" });
    toast("waking the camera");
  } catch (error) {
    toast(error.message);
  } finally {
    await refresh();
  }
}

async function stopCamera() {
  els.stopBtn.disabled = true;
  try {
    await request("/api/camera/stop", { method: "POST" });
    toast("camera stopped");
  } catch (error) {
    toast(error.message);
  } finally {
    await refresh();
  }
}

async function toggleMode() {
  const nextMock = !(state.camera && state.camera.mock);
  els.modeBtn.disabled = true;
  try {
    await request("/api/mode", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ mock: nextMock }),
    });
    toast(nextMock ? "mock mode enabled" : "live mode enabled");
  } catch (error) {
    toast(error.message);
  } finally {
    els.modeBtn.disabled = false;
    await refresh();
  }
}

async function toggleXMode() {
  const nextDryRun = !(state.x && state.x.dry_run);
  els.xModeBtn.disabled = true;
  try {
    state.x = await request("/api/x/mode", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ dry_run: nextDryRun }),
    });
    toast(nextDryRun ? "x dry-run enabled" : "x live posting enabled");
  } catch (error) {
    toast(error.message);
  } finally {
    els.xModeBtn.disabled = false;
    await refresh();
  }
}

async function robotCommand(path, body = null) {
  const options = { method: "POST" };
  if (body) {
    options.headers = { "Content-Type": "application/json" };
    options.body = JSON.stringify(body);
  }
  const data = await request(path, options);
  state.robot = data;
  await refresh();
  return data;
}

async function safeRobotCommand(path, body = null, message = "command sent") {
  try {
    await robotCommand(path, body);
    toast(message);
  } catch (error) {
    toast(error.message);
    await refresh();
  }
}

async function sendDriveCommand() {
  if (!state.drive.preset || state.drive.busy) return;
  state.drive.busy = true;
  try {
    const data = await request("/api/robot/drive", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(state.drive.preset),
    });
    state.robot = data;
    renderStatus();
  } catch (error) {
    toast(error.message);
    stopDrive(false);
    await refresh();
  } finally {
    state.drive.busy = false;
    if (state.drive.stopQueued) {
      state.drive.stopQueued = false;
      stopDrive(true);
    }
  }
}

function startDrive(button, move) {
  if (!movePresets[move] || !state.robot.ready) return;
  if (state.drive.move === move && state.drive.interval) return;
  clearDriveTimer();
  state.drive.button = button;
  state.drive.move = move;
  state.drive.preset = movePresets[move];
  if (els.commandDebug) els.commandDebug.textContent = `KEY ${move}`;
  if (button) button.classList.add("active");
  renderPageBeat("drive");
  sendDriveCommand();
  state.drive.interval = window.setInterval(sendDriveCommand, DRIVE_INTERVAL_MS);
}

function clearDriveTimer() {
  if (state.drive.interval) {
    window.clearInterval(state.drive.interval);
    state.drive.interval = null;
  }
  if (state.drive.button) {
    state.drive.button.classList.remove("active");
  }
  state.drive.button = null;
}

async function stopDrive(sendStop = true) {
  clearDriveTimer();
  state.drive.preset = null;
  state.drive.move = null;
  state.drive.key = null;
  renderPageBeat("idle");
  if (state.drive.busy) {
    state.drive.stopQueued = sendStop;
    return;
  }
  if (sendStop && state.robot.ready) {
    try {
      const data = await request("/api/robot/stop", { method: "POST" });
      state.robot = data;
      renderStatus();
    } catch (error) {
      toast(error.message);
      await refresh();
    }
  }
}

async function captureFrame() {
  els.captureBtn.disabled = true;
  triggerCaptureFlash();
  try {
    await request("/api/capture", { method: "POST" });
    toast("draft filed");
  } catch (error) {
    toast(error.message);
  } finally {
    await refresh();
  }
}

function moveRobot(event) {
  const button = event.target.closest("button[data-move]");
  if (!button) return;
  event.preventDefault();
  const move = button.dataset.move;
  if (move === "stop") {
    stopDrive(true);
    return;
  }
  if (event.pointerId != null && button.setPointerCapture) {
    button.setPointerCapture(event.pointerId);
  }
  startDrive(button, move);
}

async function sportCommand(event) {
  const button = event.target.closest("button[data-sport-command]");
  if (!button) return;
  const command = button.dataset.sportCommand;
  button.disabled = true;
  try {
    await robotCommand(`/api/robot/sport-command/${encodeURIComponent(command)}`);
    toast(`${command.toLowerCase()} requested`);
  } catch (error) {
    toast(error.message);
    await refresh();
  }
}

async function dispatchAction(event) {
  const button = event.target.closest("button[data-action]");
  if (!button) return;
  const id = button.dataset.id;
  const action = button.dataset.action;
  if (action === "add-tag") {
    appendHandle(id, button.dataset.handle);
    closeTagMenus();
    return;
  }
  if (action === "toggle-tags") {
    toggleTagMenu(id);
    return;
  }
  button.disabled = true;
  try {
    if (action === "save-caption") {
      await saveCaption(id);
      toast("caption saved");
    } else if (action === "post") {
      await saveCaption(id);
      await request(`/api/drafts/${id}/post`, { method: "POST" });
      toast(state.x && state.x.dry_run ? "dry-run post marked" : "posted to x");
    } else if (action === "delete") {
      await request(`/api/drafts/${id}`, { method: "DELETE" });
      toast("draft removed");
    } else {
      await request(`/api/drafts/${id}/${action}`, { method: "POST" });
      toast("draft updated");
    }
  } catch (error) {
    toast(error.message);
  } finally {
    await refresh();
  }
}

async function peopleTagAction(event) {
  const button = event.target.closest("button[data-action]");
  if (!button) return;
  const id = button.dataset.id;
  const action = button.dataset.action;
  button.disabled = true;
  try {
    if (action === "enable-person") {
      await request(`/api/people-tags/${id}/enable`, { method: "POST" });
      toast("tag enabled");
    } else if (action === "disable-person") {
      await request(`/api/people-tags/${id}/disable`, { method: "POST" });
      toast("tag disabled");
    } else if (action === "delete-person") {
      await request(`/api/people-tags/${id}`, { method: "DELETE" });
      toast("tag deleted");
    }
  } catch (error) {
    toast(error.message);
  } finally {
    await refresh();
  }
}

function setFilter(filter) {
  state.filter = filter;
  els.dispatchTabs.querySelectorAll(".tab").forEach((tab) => {
    tab.classList.toggle("active", tab.dataset.filter === filter);
  });
  renderDispatches();
}

function setView(view) {
  state.view = view;
  renderPanelView();
}

function handleKey(event) {
  const target = event.target;
  if (target && (target.tagName === "INPUT" || target.tagName === "TEXTAREA")) return;
  if (event.metaKey || event.ctrlKey || event.altKey) return;
  const key = event.key.toLowerCase();
  const keyMap = {
    " ": () => !els.captureBtn.disabled && captureFrame(),
    arrowup: "forward",
    arrowdown: "back",
    arrowleft: "turn-left",
    arrowright: "turn-right",
    w: "forward",
    s: "back",
    a: "turn-left",
    d: "turn-right",
    q: "left",
    e: "right",
    escape: () => stopDrive(true),
  };
  if (keyMap[key]) {
    event.preventDefault();
    if (event.repeat) return;
    if (typeof keyMap[key] === "function") {
      keyMap[key]();
      return;
    }
    state.drive.key = key;
    startDrive(dpadButtonForMove(keyMap[key]), keyMap[key]);
  }
}

function handleKeyUp(event) {
  if (state.drive.key && event.key.toLowerCase() === state.drive.key) {
    event.preventDefault();
    stopDrive(true);
  }
}

els.startBtn.addEventListener("click", startCamera);
els.stopBtn.addEventListener("click", stopCamera);
els.captureBtn.addEventListener("click", captureFrame);
els.modeBtn.addEventListener("click", toggleMode);
els.xModeBtn.addEventListener("click", toggleXMode);
els.mentionSettingsForm.addEventListener("submit", saveMentionSettings);
els.mentionPollBtn.addEventListener("click", pollMentions);
els.mentionInjectForm.addEventListener("submit", injectMention);
els.mentions.addEventListener("click", mentionAction);
els.peopleTagForm.addEventListener("submit", createPeopleTag);
els.peopleTags.addEventListener("click", peopleTagAction);
els.viewTabs.addEventListener("click", (event) => {
  const tab = event.target.closest("button[data-view]");
  if (tab) setView(tab.dataset.view);
});
els.emergencyStopBtn.addEventListener("click", () =>
  stopDrive(true),
);
els.standBtn.addEventListener("click", () =>
  safeRobotCommand("/api/robot/stand", null, "stand requested"),
);
els.lieDownBtn.addEventListener("click", () =>
  safeRobotCommand("/api/robot/lie-down", null, "rest requested"),
);
els.skillRow.addEventListener("click", sportCommand);
els.patrolBtn.addEventListener("click", () =>
  safeRobotCommand("/api/robot/patrol/start", null, "patrol started"),
);
els.cancelPatrolBtn.addEventListener("click", () =>
  safeRobotCommand("/api/robot/patrol/cancel", null, "patrol cancelled"),
);
els.dpad.addEventListener("pointerdown", moveRobot);
els.dpad.addEventListener("pointerup", () => stopDrive(true));
els.dpad.addEventListener("pointercancel", () => stopDrive(true));
els.dpad.addEventListener("pointerleave", () => stopDrive(true));
els.drafts.addEventListener("click", dispatchAction);
document.addEventListener("click", (event) => {
  if (!event.target.closest(".tag-menu")) {
    closeTagMenus();
  }
});
els.dispatchTabs.addEventListener("click", (event) => {
  const tab = event.target.closest("button[data-filter]");
  if (!tab) return;
  setFilter(tab.dataset.filter);
});
document.addEventListener("keydown", handleKey);
document.addEventListener("keyup", handleKeyUp);
window.addEventListener("blur", () => stopDrive(true));

refresh().catch((error) => toast(error.message));
state.polling = window.setInterval(() => {
  refresh().catch(() => {});
}, 1500);
