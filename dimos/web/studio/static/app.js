let settings = null;
let lastStatus = null;

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
  lastStatus = await api("/api/runtime/status");
  $("runtime-status").textContent = lastStatus.running
    ? `运行中 · ${lastStatus.blueprint} · PID ${lastStatus.pid}`
    : lastStatus.starting
      ? `正在启动 · PID ${lastStatus.pid}`
      : lastStatus.message;
  $("runtime-logs").textContent = lastStatus.log_tail || "暂无日志";
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
  loadSkill(),
  loadTeleopKeyStatus(),
]).catch((error) => toast(error.message));
setInterval(() => refreshStatus().catch(() => {}), 5000);
