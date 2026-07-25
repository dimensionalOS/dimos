let settings = null;
let lastStatus = null;
let stage2State = null;
let pendingStage2InstructionId = null;

const $ = (id) => document.getElementById(id);

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json", ...(options.headers || {}) },
    ...options,
  });
  const body = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(body.detail || `请求失败：${response.status}`);
  return body;
}

function toast(message) {
  const node = $("toast");
  node.textContent = message;
  node.style.display = "block";
  clearTimeout(node.timer);
  node.timer = setTimeout(() => { node.style.display = "none"; }, 3500);
}

function renderSafety() {
  const locked = settings?.movement_locked !== false;
  $("lock-label").textContent = locked ? "运动已锁定" : "警告：真实运动已解锁";
  $("lock-dot").style.background = locked ? "#f0bd4e" : "#e54c43";
}

async function loadSettings() {
  settings = await api("/api/settings");
  $("robot-ip").value = settings.robot_ip;
  $("robot-name").value = settings.robot_name;
  $("signal-port").value = settings.signal_port;
  $("blueprint").value = settings.blueprint;
  $("agent-model").value = settings.agent_model;
  $("detection-model").value = settings.detection_model;
  $("navigation-speed-scale").value = settings.navigation_speed_scale;
  $("obstacle-avoidance").checked = settings.obstacle_avoidance;
  $("movement-locked").checked = settings.movement_locked;
  $("system-prompt").value = settings.system_prompt;
  renderSafety();
}

function settingsFromForm() {
  return {
    ...settings,
    robot_ip: $("robot-ip").value.trim(),
    robot_name: $("robot-name").value.trim(),
    signal_port: Number($("signal-port").value),
    blueprint: $("blueprint").value.trim(),
    agent_model: $("agent-model").value.trim(),
    detection_model: $("detection-model").value,
    navigation_speed_scale: Number($("navigation-speed-scale").value),
    obstacle_avoidance: $("obstacle-avoidance").checked,
    movement_locked: $("movement-locked").checked,
    system_prompt: $("system-prompt").value,
  };
}

async function saveSettings() {
  settings = await api("/api/settings", { method: "PUT", body: JSON.stringify(settingsFromForm()) });
  renderSafety();
  $("settings-result").textContent = "参数已保存。需要重启 DimOS 才会生效。";
  toast("参数已保存");
}

async function refreshStatus() {
  const wasRunning = lastStatus?.running === true;
  lastStatus = await api("/api/runtime/status");
  $("runtime-status").textContent = lastStatus.running
    ? `运行中 · ${lastStatus.blueprint} · PID ${lastStatus.pid}`
    : lastStatus.starting
      ? `正在启动 · PID ${lastStatus.pid}`
      : lastStatus.message;
  $("runtime-logs").textContent = lastStatus.log_tail || "暂无日志";
  if ($("visual-frame")) {
    $("visual-frame").classList.toggle("is-offline", !lastStatus.running);
    if (!wasRunning && lastStatus.running) {
      $("dimos-visualizer").src = $("dimos-visualizer").src;
    }
  }
}

function renderStage2() {
  const world = stage2State?.semantic_world || { places: [] };
  const taskStatus = stage2State?.task || { state: "unavailable", active: false };
  const task = taskStatus.task || null;
  const telemetry = stage2State?.telemetry || {};
  $("stage2-connection").textContent = stage2State?.connected
    ? "已连接"
    : `不可用${stage2State?.errors?.length ? ` · ${stage2State.errors[0]}` : ""}`;
  $("stage2-map-id").textContent = world.map_id && world.map_version
    ? `${world.map_id} · ${world.map_version}`
    : "未配置";
  $("stage2-task-state").textContent =
    `${taskStatus.state || "unavailable"}${taskStatus.active ? " · active" : ""}`;
  $("stage2-task-state").dataset.state = taskStatus.state || "unavailable";
  $("stage2-task-id").textContent = task?.task_id || "—";
  $("stage2-destination").textContent = task?.destination || "—";
  const odometry = telemetry.odometry || {};
  $("stage2-odometry").textContent = odometry.fresh
    ? `fresh · ${Number(odometry.age_s || 0).toFixed(2)}s`
    : telemetry.status || "unavailable";
  const recovery = telemetry.recovery;
  $("stage2-recovery").textContent = recovery
    ? `#${recovery.attempt} ${recovery.cause} / ${recovery.action}`
    : "—";

  const places = Array.isArray(world.places) ? world.places : [];
  const select = $("stage2-places");
  const selected = select.value;
  select.replaceChildren();
  if (!places.length) {
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "当前地图没有 confirmed place";
    select.append(option);
  } else {
    places.forEach((place) => {
      const option = document.createElement("option");
      option.value = place.name;
      option.textContent = place.aliases?.length
        ? `${place.name}（${place.aliases.join(" / ")}）`
        : place.name;
      select.append(option);
    });
    if (places.some((place) => place.name === selected)) select.value = selected;
  }
  $("stage2-navigate").disabled = !stage2State?.connected || !places.length || taskStatus.active;
  $("stage2-cancel").disabled = !stage2State?.connected || !task?.task_id || !taskStatus.active;
  $("stage2-confirm-place").disabled =
    !stage2State?.connected || !odometry.fresh || taskStatus.active;
  const reply = stage2State?.last_reply;
  $("stage2-reply").textContent = reply
    ? `${reply.text}\n${reply.instruction_id}`
    : "尚无终态回复";
}

async function loadStage2() {
  stage2State = await api("/api/stage2/status");
  renderStage2();
}

async function loadTeleopKeyStatus() {
  const result = await api("/api/teleop/key");
  $("teleop-key-status").textContent = result.message;
}

async function loadSkill() {
  const result = await api("/api/skill");
  $("skill-path").textContent = result.path;
  $("skill-source").value = result.source;
  $("skill-result").textContent = `已发现 Skill：${result.skills.join(", ")}`;
}

document.querySelectorAll(".tab").forEach((button) => {
  button.addEventListener("click", () => {
    document.querySelectorAll(".tab, .page").forEach((node) => node.classList.remove("active"));
    button.classList.add("active");
    $(`page-${button.dataset.page}`).classList.add("active");
  });
});

$("save-connection").addEventListener("click", async () => {
  settings = await api("/api/settings", { method: "PUT", body: JSON.stringify(settingsFromForm()) });
  toast("连接参数已保存");
});

$("discover-robot").addEventListener("click", async () => {
  $("robot-result").textContent = "正在搜索同一网络中的 Go2，最多约 12 秒…";
  try {
    const result = await api("/api/robot/discover", { method: "POST" });
    if (result.found && result.robots.length > 0) {
      const robot = result.robots[0];
      $("robot-ip").value = robot.ip;
      settings = await api("/api/settings", {
        method: "PUT",
        body: JSON.stringify(settingsFromForm()),
      });
      $("robot-result").textContent =
        `已发现真实 Go2\nIP: ${robot.ip}\n序列号: ${robot.serial || "未知"}\n已自动保存 IP。`;
      toast("已找到机器狗");
    } else {
      $("robot-result").textContent = result.message;
    }
  } catch (error) {
    $("robot-result").textContent = error.message;
  }
});

$("check-robot").addEventListener("click", async () => {
  $("robot-result").textContent = "检测中…";
  try {
    const result = await api("/api/robot/check", {
      method: "POST",
      body: JSON.stringify({ robot_ip: $("robot-ip").value.trim(), signal_port: Number($("signal-port").value) }),
    });
    $("robot-result").textContent = JSON.stringify(result, null, 2);
  } catch (error) {
    $("robot-result").textContent = error.message;
  }
});

$("start-sim").addEventListener("click", async () => {
  try {
    const result = await api("/api/runtime/start", {
      method: "POST", body: JSON.stringify({ mode: "simulation", confirmation: "" }),
    });
    toast(result.message);
    setTimeout(refreshStatus, 2500);
  } catch (error) { toast(error.message); }
});

$("save-teleop-key").addEventListener("click", async () => {
  const apiKey = $("teleop-key").value.trim();
  try {
    const result = await api("/api/teleop/key", {
      method: "PUT",
      body: JSON.stringify({ api_key: apiKey }),
    });
    $("teleop-key").value = "";
    $("teleop-key-status").textContent = result.message;
    toast(result.message);
  } catch (error) {
    toast(error.message);
  }
});

$("start-hosted-teleop").addEventListener("click", async () => {
  try {
    settings = await api("/api/settings", {
      method: "PUT",
      body: JSON.stringify(settingsFromForm()),
    });
    const result = await api("/api/runtime/start", {
      method: "POST",
      body: JSON.stringify({
        mode: "hosted_teleop",
        confirmation: $("teleop-confirmation").value,
      }),
    });
    toast(result.message);
    await refreshStatus();
  } catch (error) {
    toast(error.message);
  }
});

$("connect-hardware-readonly").addEventListener("click", async () => {
  try {
    settings = await api("/api/settings", {
      method: "PUT",
      body: JSON.stringify(settingsFromForm()),
    });
    const result = await api("/api/runtime/start", {
      method: "POST",
      body: JSON.stringify({ mode: "hardware_readonly", confirmation: "" }),
    });
    toast(result.message);
    setTimeout(refreshStatus, 2500);
  } catch (error) { toast(error.message); }
});

$("start-hardware").addEventListener("click", async () => {
  try {
    const result = await api("/api/runtime/start", {
      method: "POST",
      body: JSON.stringify({ mode: "hardware", confirmation: $("hardware-confirmation").value }),
    });
    toast(result.message);
    setTimeout(refreshStatus, 2500);
  } catch (error) { toast(error.message); }
});

$("stop-runtime").addEventListener("click", async () => {
  try {
    const result = await api("/api/runtime/stop", { method: "POST" });
    toast(result.message);
    await refreshStatus();
  } catch (error) { toast(error.message); }
});

$("send-agent").addEventListener("click", async () => {
  $("agent-result").textContent = "发送中…";
  try {
    const result = await api("/api/agent/send", {
      method: "POST", body: JSON.stringify({ message: $("agent-message").value }),
    });
    $("agent-result").textContent = result.result || "已发送";
  } catch (error) { $("agent-result").textContent = error.message; }
});

$("mission-estop").addEventListener("click", async () => {
  $("stage2-action-result").textContent = "正在请求 canonical MCP stop_all…";
  try {
    const result = await api("/api/stage2/stop-all", { method: "POST" });
    const failures = Array.isArray(result.failed_components)
      ? result.failed_components
      : [];
    $("stage2-action-result").textContent = failures.length
      ? `stop_all 已返回，但以下组件失败：${failures.join("、")}`
      : "stop_all 已确认停止。";
    await loadStage2();
  } catch (error) {
    $("stage2-action-result").textContent = `stop_all 未确认：${error.message}`;
  }
});

$("stage2-navigate").addEventListener("click", async () => {
  const destination = $("stage2-places").value;
  if (!destination) return;
  pendingStage2InstructionId ||= `studio-${crypto.randomUUID()}`;
  $("stage2-action-result").textContent = "正在提交到 Agent Gateway…";
  try {
    const result = await api("/api/stage2/navigate", {
      method: "POST",
      body: JSON.stringify({
        instruction_id: pendingStage2InstructionId,
        destination,
      }),
    });
    $("stage2-action-result").textContent =
      `已受理 ${result.instruction_id}；这不代表已经到达。`;
    pendingStage2InstructionId = null;
    await loadStage2();
  } catch (error) {
    $("stage2-action-result").textContent =
      `${error.message}；下次重试会复用同一 instruction ID。`;
  }
});

$("stage2-confirm-place").addEventListener("click", async () => {
  const name = $("stage2-place-name").value.trim();
  if (!name) {
    $("stage2-action-result").textContent = "请先输入地点名称。";
    return;
  }
  const aliases = $("stage2-place-aliases").value
    .split(/[，,]/u)
    .map((value) => value.trim())
    .filter(Boolean);
  $("stage2-action-result").textContent = `正在用 fresh odometry 确认“${name}”…`;
  try {
    const result = await api("/api/stage2/places/confirm-current", {
      method: "POST",
      body: JSON.stringify({ name, aliases }),
    });
    $("stage2-action-result").textContent =
      `已确认地点：${result.place?.name || name}。`;
    $("stage2-place-name").value = "";
    $("stage2-place-aliases").value = "";
    await loadStage2();
  } catch (error) {
    $("stage2-action-result").textContent = error.message;
  }
});

$("stage2-cancel").addEventListener("click", async () => {
  const taskId = stage2State?.task?.task?.task_id;
  if (!taskId) return;
  $("stage2-action-result").textContent = `正在取消 ${taskId}…`;
  try {
    const result = await api("/api/stage2/cancel", {
      method: "POST",
      body: JSON.stringify({ task_id: taskId }),
    });
    $("stage2-action-result").textContent =
      `取消结果：${result.state || "unknown"} · active=${String(result.active)}`;
    await loadStage2();
  } catch (error) {
    $("stage2-action-result").textContent = error.message;
  }
});

$("reload-visualizer").addEventListener("click", () => {
  $("dimos-visualizer").src = $("dimos-visualizer").src;
  $("stage2-action-result").textContent = lastStatus?.running
    ? "正在重新连接相机与地图…"
    : "DimOS 尚未运行，画面服务还没有启动。";
});

$("load-tools").addEventListener("click", async () => {
  $("agent-result").textContent = "读取中…";
  try {
    const result = await api("/api/mcp/tools");
    $("agent-result").textContent = result.tools.map((tool) => `${tool.name} — ${tool.description || ""}`).join("\n");
  } catch (error) { $("agent-result").textContent = error.message; }
});

$("reload-skill").addEventListener("click", () => loadSkill().catch((error) => toast(error.message)));
$("save-skill").addEventListener("click", async () => {
  try {
    const result = await api("/api/skill", {
      method: "PUT", body: JSON.stringify({ source: $("skill-source").value }),
    });
    $("skill-result").textContent = `${result.message}\nSkill：${result.skills.join(", ")}`;
  } catch (error) { $("skill-result").textContent = error.message; }
});
$("save-settings").addEventListener("click", () => saveSettings().catch((error) => toast(error.message)));
$("refresh-logs").addEventListener("click", () => refreshStatus().catch((error) => toast(error.message)));

Promise.all([
  loadSettings(),
  refreshStatus(),
  loadStage2(),
  loadSkill(),
  loadTeleopKeyStatus(),
]).catch((error) => toast(error.message));
setInterval(() => refreshStatus().catch(() => {}), 5000);
setInterval(() => loadStage2().catch(() => {}), 2000);
