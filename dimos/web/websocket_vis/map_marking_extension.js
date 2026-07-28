(() => {
  "use strict";

  if (window.__dimosSemanticMapMarkingLoaded) {
    return;
  }
  window.__dimosSemanticMapMarkingLoaded = true;

  const style = document.createElement("style");
  style.textContent = `
    #dimos-map-marking-panel {
      position: fixed;
      z-index: 10000;
      top: 12px;
      left: 12px;
      display: flex;
      align-items: center;
      gap: 8px;
      padding: 9px;
      border: 1px solid rgba(255,255,255,.28);
      border-radius: 8px;
      background: rgba(10,12,16,.92);
      color: white;
      font: 14px/1.25 -apple-system, BlinkMacSystemFont, "PingFang SC", sans-serif;
      box-shadow: 0 6px 22px rgba(0,0,0,.35);
    }
    #dimos-map-marking-panel input {
      width: 150px;
      padding: 7px 9px;
      border: 1px solid #4a5568;
      border-radius: 5px;
      background: #151a22;
      color: white;
    }
    #dimos-map-marking-panel button {
      padding: 7px 10px;
      border: 0;
      border-radius: 5px;
      background: #2f855a;
      color: white;
      cursor: pointer;
    }
    #dimos-map-marking-panel button[data-armed="true"] {
      background: #dd6b20;
    }
    #dimos-map-marking-panel .cancel {
      background: #4a5568;
    }
    #dimos-map-marking-status {
      max-width: 260px;
      color: #cbd5e0;
    }
    #dimos-semantic-marker-layer {
      position: fixed;
      inset: 0;
      z-index: 9999;
      pointer-events: none;
    }
    .dimos-semantic-marker {
      position: fixed;
      transform: translate(-50%, -100%);
      display: flex;
      flex-direction: column;
      align-items: center;
      color: white;
      font: 13px/1.1 -apple-system, BlinkMacSystemFont, "PingFang SC", sans-serif;
      text-shadow: 0 1px 3px black;
    }
    .dimos-semantic-marker::after {
      content: "";
      width: 12px;
      height: 12px;
      margin-top: 4px;
      border: 2px solid white;
      border-radius: 50% 50% 50% 0;
      background: #38a169;
      transform: rotate(-45deg);
      box-shadow: 0 2px 5px rgba(0,0,0,.6);
    }
    .dimos-semantic-marker[data-status="pending"]::after {
      background: #dd6b20;
    }
    .dimos-semantic-marker[data-status="rejected"]::after {
      background: #e53e3e;
    }
  `;
  document.head.appendChild(style);

  const panel = document.createElement("div");
  panel.id = "dimos-map-marking-panel";
  panel.innerHTML = `
    <strong>地图点位</strong>
    <input id="dimos-map-marking-name" maxlength="200" placeholder="输入点位名称">
    <button id="dimos-map-marking-arm" type="button">标记点位</button>
    <button id="dimos-map-marking-cancel" class="cancel" type="button">取消</button>
    <span id="dimos-map-marking-status">点击“标记点位”后，再点击地图空地；不会移动机器狗。</span>
  `;
  document.body.appendChild(panel);

  const layer = document.createElement("div");
  layer.id = "dimos-semantic-marker-layer";
  document.body.appendChild(layer);

  const nameInput = panel.querySelector("#dimos-map-marking-name");
  const armButton = panel.querySelector("#dimos-map-marking-arm");
  const cancelButton = panel.querySelector("#dimos-map-marking-cancel");
  const status = panel.querySelector("#dimos-map-marking-status");

  async function postJson(path, payload = {}) {
    const response = await fetch(path, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(payload),
    });
    const result = await response.json();
    if (!response.ok || result.accepted === false) {
      throw new Error(result.reason || "操作失败");
    }
    return result;
  }

  armButton.addEventListener("click", async () => {
    const name = nameInput.value.trim();
    if (!name) {
      status.textContent = "请先输入点位名称。";
      nameInput.focus();
      return;
    }
    try {
      await postJson("/api/map-marking/arm", {name});
      status.textContent = `下一次地图点击将标记“${name}”，不会移动机器狗。`;
      armButton.dataset.armed = "true";
    } catch (error) {
      status.textContent = error.message;
    }
  });

  cancelButton.addEventListener("click", async () => {
    try {
      await postJson("/api/map-marking/cancel");
      status.textContent = "已取消标记模式。";
      armButton.dataset.armed = "false";
    } catch (error) {
      status.textContent = error.message;
    }
  });

  function mapSvg() {
    const candidates = [...document.querySelectorAll("svg")];
    return candidates.find((svg) => svg.querySelector("canvas")) || candidates[0] || null;
  }

  function worldToViewport(marker, map, rect) {
    const leftAxis = 60;
    const bottomAxis = 40;
    const availableWidth = rect.width - leftAxis;
    const availableHeight = rect.height - bottomAxis;
    const scale = Math.min(
      availableWidth / map.cols,
      availableHeight / map.rows,
    );
    const mapWidth = map.cols * scale;
    const mapHeight = map.rows * scale;
    const mapLeft = rect.left + leftAxis + (availableWidth - mapWidth) / 2;
    const mapTop = rect.top + (availableHeight - mapHeight) / 2;
    const col = (marker.x - map.origin_x) / map.resolution;
    const row = (marker.y - map.origin_y) / map.resolution;
    return {
      left: mapLeft + col * scale,
      top: mapTop + mapHeight - row * scale,
    };
  }

  function renderMarkers(state) {
    const svg = mapSvg();
    if (!svg || !state.map) {
      layer.replaceChildren();
      return;
    }
    const rect = svg.getBoundingClientRect();
    const fragment = document.createDocumentFragment();
    for (const marker of state.markers || []) {
      const point = worldToViewport(marker, state.map, rect);
      const element = document.createElement("div");
      element.className = "dimos-semantic-marker";
      element.dataset.status = marker.status;
      element.style.left = `${point.left}px`;
      element.style.top = `${point.top}px`;
      element.textContent = marker.name;
      if (marker.reason) {
        element.title = marker.reason;
      }
      fragment.appendChild(element);
    }
    layer.replaceChildren(fragment);
  }

  async function refresh() {
    try {
      const response = await fetch("/api/map-marking/state", {cache: "no-store"});
      if (!response.ok) {
        throw new Error("地图标记服务未连接");
      }
      const state = await response.json();
      renderMarkers(state);
      armButton.dataset.armed = state.pending_name ? "true" : "false";
      if (state.error) {
        status.textContent = state.error;
      } else if (state.pending_name) {
        status.textContent = `下一次地图点击将标记“${state.pending_name}”，不会移动机器狗。`;
      } else if ((state.markers || []).length > 0) {
        const confirmed = state.markers.filter((item) => item.status === "confirmed").length;
        status.textContent = `已显示 ${state.markers.length} 个点位，确认 ${confirmed} 个。`;
      }
    } catch (error) {
      status.textContent = error.message;
    }
  }

  window.addEventListener("resize", refresh);
  window.setInterval(refresh, 700);
  refresh();
})();
