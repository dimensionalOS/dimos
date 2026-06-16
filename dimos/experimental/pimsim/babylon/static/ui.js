// Copyright 2025-2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

(() => {
  const startedAt = Date.now();
  const cameraTargets = {
    primary: {
      img: document.getElementById("cameraImg"),
      label: document.getElementById("cameraLabel"),
      panel: document.getElementById("cameraPanel"),
      lastUrl: null,
    },
    workspace: {
      img: document.getElementById("workspaceImg"),
      label: document.getElementById("workspaceLabel"),
      panel: document.getElementById("workspacePanel"),
      lastUrl: null,
    },
  };

  const logEl = document.getElementById("sessionLog");
  const statusEl = document.getElementById("status");
  const streamLabel = document.getElementById("streamLabel");
  const driveStateLabel = document.getElementById("driveStateLabel");
  const clickModeLabel = document.getElementById("clickModeLabel");
  const entityStateLabel = document.getElementById("entityStateLabel");
  const videoLabel = document.getElementById("videoLabel");
  const runtimeLabel = document.getElementById("runtimeLabel");

  function timeLabel(date = new Date()) {
    return date.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
  }

  function appendLog(message) {
    if (!logEl || !message) return;
    const item = document.createElement("div");
    item.className = "log-item";
    const ts = document.createElement("span");
    ts.textContent = timeLabel();
    const body = document.createElement("span");
    body.textContent = message;
    item.append(ts, body);
    logEl.prepend(item);
    while (logEl.children.length > 6) {
      logEl.lastElementChild?.remove();
    }
  }

  let lastStatus = "";
  function setStatus(message) {
    const text = String(message || "");
    if (statusEl) statusEl.textContent = text;
    if (streamLabel) streamLabel.textContent = text || "Idle";
    if (text && text !== lastStatus) {
      appendLog(text);
      lastStatus = text;
    }
  }

  function setButtonActive(id, active) {
    const button = document.getElementById(id);
    if (!button) return;
    button.dataset.active = active ? "true" : "false";
    if (id === "toggleDrive" && driveStateLabel) {
      driveStateLabel.textContent = active ? "Enabled" : "Disabled";
    }
    if (
      (id === "navClick" || id === "pointClick" || id === "graspClick" || id === "spawnClick") &&
      clickModeLabel
    ) {
      if (active) {
        clickModeLabel.textContent = button.textContent.trim();
        return;
      }
      const anyActive = ["navClick", "pointClick", "graspClick", "spawnClick"].some(
        (buttonId) => document.getElementById(buttonId)?.dataset.active === "true",
      );
      if (!anyActive) clickModeLabel.textContent = "None";
    }
  }

  function isButtonActive(id) {
    return document.getElementById(id)?.dataset.active === "true";
  }

  function setPanelActive(id, active) {
    const panel = document.getElementById(id);
    if (panel) panel.dataset.active = active ? "true" : "false";
  }

  function updateCameraFrame(cameraName, buffer, jpegOffset) {
    const jpegBytes = new Uint8Array(buffer, jpegOffset);
    const blob = new Blob([jpegBytes], { type: "image/jpeg" });
    const url = URL.createObjectURL(blob);
    const target = cameraName === "workspace" ? cameraTargets.workspace : cameraTargets.primary;
    if (target.img) {
      target.img.src = url;
      if (target.lastUrl) URL.revokeObjectURL(target.lastUrl);
      target.lastUrl = url;
    }
    if (target.label) target.label.textContent = cameraName;
    if (target.panel) target.panel.dataset.hasFrame = "true";
    if (videoLabel) {
      videoLabel.textContent = "LIVE";
      videoLabel.className = "active";
    }
  }

  function setEntityStatus(message) {
    if (entityStateLabel) entityStateLabel.textContent = message;
  }

  function tickRuntime() {
    if (!runtimeLabel) return;
    const elapsed = Math.max(0, Date.now() - startedAt);
    const totalSeconds = Math.floor(elapsed / 1000);
    const hours = String(Math.floor(totalSeconds / 3600)).padStart(2, "0");
    const minutes = String(Math.floor((totalSeconds % 3600) / 60)).padStart(2, "0");
    const seconds = String(totalSeconds % 60).padStart(2, "0");
    runtimeLabel.textContent = `${hours}:${minutes}:${seconds}`;
  }

  tickRuntime();
  window.setInterval(tickRuntime, 1000);

  window.PimSimUI = {
    appendLog,
    isButtonActive,
    setButtonActive,
    setEntityStatus,
    setPanelActive,
    setStatus,
    updateCameraFrame,
  };
})();
