"use strict";
// FRANK ops dashboard: poll every 2 s, redraw. Deliberately dumb — it is a debugging window.
// Every list is drawn oldest-first so the newest row is at the bottom, where the CSS pins the view.

const $ = (id) => document.getElementById(id);
const expanded = new Set();   // context rows the operator asked to see in full

const ago = (ts) => {
  if (!ts) return "–";
  const s = Math.max(0, Date.now() / 1000 - ts);
  if (s < 60) return `${s | 0}s`;
  if (s < 3600) return `${(s / 60) | 0}m`;
  return `${(s / 3600) | 0}h`;
};
const clock = (ts) => (ts ? new Date(ts * 1000).toLocaleTimeString() : "–");
const num = (v, d = 2) => (v === null || v === undefined ? "–" : Number(v).toFixed(d));

function table(cols, rows) {
  const head = `<tr>${cols.map((c) => `<th>${c}</th>`).join("")}</tr>`;
  const body = rows.map((r) => `<tr>${r.join("")}</tr>`).join("");
  return `<table>${head}${body}</table>`;
}
const td = (v, cls = "") => `<td class="${cls}">${v === undefined || v === null ? "–" : v}</td>`;
const esc = (s) =>
  String(s).replace(/[&<>]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" })[c]);

async function get(path) {
  const res = await fetch(path);
  if (!res.ok) throw new Error(`${path} → HTTP ${res.status}`);
  return res.json();
}

// --- panels -------------------------------------------------------------

function drawPeople(d) {
  $("people").innerHTML = table(
    ["", "id", "name", "enrolled", "last chat", "last seen", "follow-ups", ""],
    d.people.map((p) => [
      td(`<img class="selfie" src="/ops/api/selfie/${p.person_id}.jpg" alt="">`),
      td(p.person_id),
      td(esc(p.name)),
      td(`${clock(p.created_at)}`),
      td(`${ago(p.last_chat_ts)} ago`),
      td(p.last_seen_ts ? `${ago(p.last_seen_ts)} ago` : "never"),
      td(p.follow_ups_today),
      td(`<button data-del="${p.person_id}" data-name="${esc(p.name)}">delete</button>`),
    ]),
  );
  $("people").querySelectorAll("button[data-del]").forEach((b) =>
    b.addEventListener("click", async () => {
      if (!confirm(`Delete ${b.dataset.name} (${b.dataset.del})? Chats, selfie and tasks go too.`)) return;
      b.disabled = true;
      await fetch(`/ops/api/people/${b.dataset.del}`, { method: "DELETE" });
      tick();
    }),
  );
}

function drawWorld(d) {
  $("world").innerHTML = table(
    ["name", "in view", "x", "y", "bearing", "range", "last seen"],
    d.people.map((p) => [
      td(`${esc(p.name)} <span class="muted">${p.person_id}</span>`),
      td(p.in_view ? "yes" : "no", p.in_view ? "yes" : "no"),
      td(num(p.x)),
      td(num(p.y)),
      td(p.bearing_deg === null ? "–" : num(p.bearing_deg, 1) + "°"),
      td(p.range_m === null ? "–" : num(p.range_m, 2) + " m"),
      td(`${ago(p.last_seen_ts)} ago`),
    ]),
  );
}

function drawEvents(d) {
  $("events").innerHTML = table(
    ["id", "type", "person", "created", "delivered", "payload"],
    [...d.events].reverse().map((e) => [
      td(e.id),
      td(e.type),
      td(e.name ? `${esc(e.name)} <span class="muted">${e.person_id}</span>` : "–"),
      td(clock(e.ts)),
      td(
        e.delivered ? (e.retired ? "yes / retired" : "yes") : "queued",
        e.delivered ? "yes" : "no",
      ),
      td(esc(JSON.stringify(e.payload)), "text"),
    ]),
  );
  $("tasks").innerHTML = table(
    ["task_id", "task", "person", "created", "closed", "outcome", "note"],
    [...d.tasks].reverse().map((t) => [
      td(t.task_id),
      td(t.task),
      td(t.name ? esc(t.name) : t.person_id || "–"),
      td(clock(t.created_at)),
      td(t.closed_at ? clock(t.closed_at) : "open"),
      td(t.outcome || "–"),
      td(t.note ? esc(t.note) : "–", "text"),
    ]),
  );
}

function drawMessages(d) {
  $("messages").innerHTML = table(
    ["time", "person", "dir", "text"],
    [...d.messages].reverse().map((m) => [
      td(clock(m.ts)),
      td(`${esc(m.name || m.person_id)}`),
      td(m.from === "frank" ? "frank →" : "→ frank"),
      td(esc(m.text), "text"),
    ]),
  );
}

function drawMotion(d) {
  const b = $("motion");
  b.textContent = d.on ? "Motion ON" : "Motion OFF";
  b.classList.toggle("on", d.on);
  b.classList.toggle("off", !d.on);
  b.title = d.on
    ? "Frank may move. Click to stop movement and switch motion off"
    : "Motion disabled. Click to allow movement";
}

$("motion").addEventListener("click", async () => {
  const on = $("motion").classList.contains("on");
  await loopAction("/ops/api/motion", { on: !on });
});

$("halt").addEventListener("click", async () => {
  $("halt").disabled = true;
  $("haltnote").textContent = "halting…";
  try {
    const res = await fetch("/ops/api/halt", { method: "POST" });
    const d = await res.json();
    $("haltnote").textContent =
      (res.ok && d.ok
        ? "Movement stopped; motion OFF. Enable Motion to move again."
        : "Motion OFF, but STOP NOT CONFIRMED. Use the Unitree remote.") +
      (d.output ? ` — ${d.output.split("\n").slice(-1)[0]}` : "");
  } catch (err) {
    $("haltnote").textContent = "halt failed: " + err.message;
  } finally {
    $("halt").disabled = false;
    tick();
  }
});

function drawLoop(d) {
  const bits = [
    d.running ? `<span class="run">running</span> pid ${d.pid}` : `<span class="stopped">stopped</span>`,
    d.session ? `session ${esc(d.session)}` : "no session recorded",
    `${esc(d.harness || "?")} · ${esc(d.model || "default model")}`,
    d.started_ts ? `up ${ago(d.started_ts)}` : "",
  ];
  $("loopstate").innerHTML = "loop: " + bits.filter(Boolean).join(" · ");
}

async function loopAction(path, body) {
  $("restart").disabled = $("stop").disabled = true;
  try {
    const res = await fetch(path, {
      method: "POST",
      headers: { "content-type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    const result = await res.json();
    if (!res.ok || result.ok === false) throw new Error(result.output || "Action failed");
    $("haltnote").textContent = result.motion_on === false || result.on === false
      ? "Motion OFF; movement stopped." : "";
  } catch (err) {
    $("haltnote").textContent = "Action failed: " + err.message;
  } finally {
    $("restart").disabled = $("stop").disabled = false;
    tick();
  }
}

// inline confirm rather than window.confirm, so the 2 s poll never eats the dialog
$("restart").addEventListener("click", () => {
  $("loopconfirm").hidden = false;
  $("restart").hidden = true;
});
$("restart-no").addEventListener("click", () => {
  $("loopconfirm").hidden = true;
  $("restart").hidden = false;
});
$("restart-yes").addEventListener("click", async () => {
  $("loopconfirm").hidden = true;
  $("restart").hidden = false;
  await loopAction("/ops/api/loop/restart", { harness: "pi" });
});
$("stop").addEventListener("click", () => loopAction("/ops/api/loop/stop"));

function drawContext(d) {
  const loop = d.loop || {};
  $("loopinfo").textContent = loop.session
    ? `${loop.harness || "?"} · ${loop.session} · ${loop.model || "default model"}`
    : "no loop_state.json (loop.py not started)";

  const msgs = d.messages || [];
  const head =
    `<div class="muted">session messages array: ${msgs.length} entries · ` +
    `session file: ${d.session_file ? esc(d.session_file) : "none found yet (Pi writes it on the first turn)"}</div>` +
    (msgs.length
      ? ""
      : `<div class="empty">New session: the messages array starts empty. Server chat history is
         NOT in Frank's context; he sees it only if he runs <code>inbox.py history</code>, which
         appears below as a tool result.</div>`);
  const totalImages = msgs.filter((m) => m.role === "image").length;
  const turns = msgs
    .map((m, i) => {
      if (m.role === "image") {
        const kb = m.bytes ? `${Math.round(m.bytes / 1024)} KB` : "";
        const cap = [`image ${m.index + 1} of ${totalImages}`, esc(m.mime), kb]
          .filter(Boolean)
          .join(" · ");
        return `<div class="turn image">
          <div class="role">image${m.tool ? ` · ${esc(m.tool)}` : ""}
            <span class="muted">${m.ts ? esc(String(m.ts)) : ""}</span></div>
          <a href="${m.src}" target="_blank" rel="noopener">
            <img class="frame" src="${m.src}" loading="lazy" alt="${cap}"></a>
          <div class="muted">${cap}</div></div>`;
      }
      const key = `${i}:${(m.text || "").length}`;
      const full = m.full && expanded.has(key);
      const text = full ? m.full : m.text || "";
      const more = m.full
        ? `<div class="more"><button data-key="${key}">${full ? "show less" : `show all (${m.chars} chars)`}</button></div>`
        : "";
      const label = m.tool ? `${m.role} · ${esc(m.tool)}` : m.role;
      return `<div class="turn ${m.role}">
        <div class="role">${label}${m.is_error ? ' <span class="err">error</span>' : ""}
          <span class="muted">${m.ts ? esc(String(m.ts)) : ""}</span></div>
        <pre>${esc(text)}</pre>${more}</div>`;
    })
    .join("");
  const ephemeral = `<div class="ephemeral">
      <div class="tag">world block — ${esc(d.world_block_note)}</div>
      <pre>${esc(d.world_block || "(cache/world.txt is empty)")}</pre>
    </div>`;
  // keep the reading position while a turn is being appended every 2 s
  const box = $("context");
  const stick = box.scrollHeight - box.scrollTop - box.clientHeight < 40;
  const keep = box.scrollTop;
  box.innerHTML = head + turns + ephemeral;
  box.scrollTop = stick ? box.scrollHeight : keep;
  $("context").querySelectorAll("button[data-key]").forEach((b) =>
    b.addEventListener("click", () => {
      const k = b.dataset.key;
      expanded.has(k) ? expanded.delete(k) : expanded.add(k);
      tick();
    }),
  );
  // the strip and the log panels are edited often; never let a missing element kill the panel
  if ($("log")) $("log").textContent = (d.log || []).join("\n") || "(no cache/loop.log yet)";
  const extra = d.stderr || [];
  if ($("stderr-wrap")) $("stderr-wrap").hidden = extra.length === 0;
  if ($("stderr")) $("stderr").textContent = extra.join("\n");
}

const runtimeBusy = new Set();
function runtimeButton(kind, id, label, disabled = false) {
  const key = `${kind}:${id}`;
  return `<button data-runtime-kind="${esc(kind)}" data-runtime-id="${esc(id)}" ${disabled || runtimeBusy.has(key) ? "disabled" : ""}>${esc(label)}</button>`;
}
function drawRuntime(d) {
  const listener = d.listener || {};
  const pi = d.pi_inbox_listener || {};
  const watchers = d.face_watcher_pids || [];
  const status = (text, active) => `<span class="runtime-status ${active ? "yes" : "muted"}">${esc(text)}</span>`;
  const row = (title, detail, state, button) => `<div class="runtime-row"><div><strong>${esc(title)}</strong><p>${esc(detail)}</p></div>${state}${button}</div>`;
  const listeners =
    row("Skill feedback", "Delivers navigation results and failures to Frank. Stopping also disables motion.",
      status(listener.connected ? "Connected" : listener.running ? "Reconnecting" : "Stopped", listener.connected),
      runtimeButton("mcp", "", "Stop listener", !listener.running)) +
    row("Chat and event inbox", "Handles phone messages and callbacks. Turn Frank off to stop this listener and its tools.",
      status(pi.active ? "Pi listening" : pi.loop_running ? "Waiting for events" : "Stopped", pi.loop_running),
      runtimeButton("pi", "", "Turn Frank off", !pi.loop_running)) +
    row("Face watcher", "Recognizes enrolled faces. Stopping also disables motion; start the watcher again with up.py.",
      status(watchers.length ? `Running · PID ${watchers.join(", ")}` : "Stopped", watchers.length > 0),
      runtimeButton("face_watcher", "", "Stop watcher", !watchers.length));
  const names = {begin_exploration: "Exploring", start_patrol: "Patrolling", follow_person: "Following a person",
    look_out_for: "Watching for objects", navigate_with_text: "Navigating", move_to: "Moving to a point"};
  const operations = (d.operations || []).map(op => row(names[op.tool] || op.tool,
    op.last_message || `Started ${ago(op.started_ts)} ago`, status(op.status, true),
    runtimeButton("operation", op.id, "Cancel", !(op.tool in names)))).join("");
  const watches = (d.watches || []).map(w => row(`Looking for ${w.name}`, w.say ? `On sight: “${w.say}”` : "Notify Frank when the face watcher recognizes this person.",
    status("Watching", true), runtimeButton("watch", w.id, "Remove watch"))).join("");
  const processes = (d.processes || []).map(p => `<span>${esc(p.role || p.name)} <span class="muted">PID ${p.pid}</span></span>`).join(" · ");
  const labels = {listener_connected:"Feedback connected", listener_stopped:"Feedback stopped", listener_error:"Feedback connection error",
    watch_armed:"Person watch armed", instant_response_started:"Instant greeting started", instant_response_spoken:"Instant greeting spoken", instant_response_failed:"Instant greeting failed", runtime_reset:"Runtime cleared", session_started:"New session started", operation_started:"Operation started", operation_result:"Tool returned",
    tool_callback:"Skill feedback", operation_cancelled:"Operation cancelled", operation_cancel_failed:"Cancel failed",
    watch_removed:"Person watch removed", face_watcher_stopped:"Face watcher stopped", frank_off:"Frank turned off", frank_restarted:"Frank restarted"};
  const audit = (d.audit || []).slice(-15).reverse().map(e => `<tr><td>${clock(e.ts)}</td><td>${esc(labels[e.event] || e.event.replaceAll("_", " "))}</td><td class="text">${esc(e.tool || e.session || e.person_id || "")}${e.message ? `<br>${esc(e.message)}` : ""}</td></tr>`).join("");
  $("runtime").innerHTML = `<h3>Listeners</h3>${listeners}
    ${listener.error ? `<p class="err">${esc(listener.error)}</p>` : ""}
    <h3>Active operations (${(d.operations || []).length})</h3>${operations || '<p class="muted">No active tool operations.</p>'}
    <h3>Person watches (${(d.watches || []).length})</h3>${watches || '<p class="muted">Not looking for anyone.</p>'}
    <h3>Processes</h3><p>${processes || '<span class="muted">Frank is off.</span>'}</p>
    <h3>Recent activity</h3><table><thead><tr><th>Time</th><th>Event</th><th>Details</th></tr></thead><tbody>${audit || '<tr><td colspan="3">No activity yet.</td></tr>'}</tbody></table>`;
}
$("runtime").addEventListener("click", async event => {
  const button = event.target.closest("button[data-runtime-kind]");
  if (!button) return;
  const kind = button.dataset.runtimeKind, id = button.dataset.runtimeId;
  const key = `${kind}:${id}`;
  if (runtimeBusy.has(key)) return;
  runtimeBusy.add(key);
  button.disabled = true;
  $("runtime-note").textContent = "Stopping...";
  try {
    const response = await fetch(kind === "pi" ? "/ops/api/loop/stop" : "/ops/api/runtime/stop", {
      method: "POST", headers: {"content-type":"application/json"}, body: JSON.stringify({kind, id}),
    });
    const result = await response.json();
    if (!response.ok || !result.ok) throw new Error(result.output || result.detail || "Stop failed");
    $("runtime-note").textContent = kind === "watch" ? "Watch removed." : result.motion_on === false ? "Stopped. Motion remains OFF." : "Operation cancelled.";
  } catch (error) {
    $("runtime-note").textContent = `Could not confirm stop: ${error.message}`;
  } finally {
    runtimeBusy.delete(key);
    tick();
  }
});

// --- loop ---------------------------------------------------------------

async function tick() {
  $("clock").textContent = new Date().toLocaleTimeString();
  const panels = [
    ["/ops/api/loop", drawLoop, "loopstate"],
    ["/ops/api/runtime", drawRuntime, "runtime"],
    ["/ops/api/motion", drawMotion, "loopstate"],
    ["/ops/api/people", drawPeople, "people"],
    ["/ops/api/world", drawWorld, "world"],
    ["/ops/api/events", drawEvents, "events"],
    ["/ops/api/messages", drawMessages, "messages"],
    ["/ops/api/context", drawContext, "context"],
  ];
  await Promise.all(
    panels.map(async ([url, draw, id]) => {
      try {
        draw(await get(url));
      } catch (err) {
        $(id).innerHTML = `<span class="err">${esc(err.message)}</span>`;
      }
    }),
  );
}

$("refresh").addEventListener("click", tick);
setInterval(() => { if ($("auto").checked) tick(); }, 2000);
tick();
