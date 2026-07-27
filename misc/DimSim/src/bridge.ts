/**
 * DimosBridge — Browser-side WebSocket client for dimos integration.
 *
 * Uses separate WebSocket connections so large sensor streams do not block
 * each other or the real-time control channel:
 *   wsControl  → JSON commands out; server pose + embodiment config in
 *   wsSensors  → Rapier world snapshots (server-side physics + lidar)
 *   wsRgb      → /color_image
 *   wsDepth    → /depth_image
 *
 * Sensor messages are LCM-encoded binary packets using @dimos/msgs, sent over
 * WebSocket to the bridge server which relays them to dimos via LCM/UDP.
 */

// @ts-ignore — CDN import (runs in browser, no Deno/Node type resolution)
import {
  encodePacket,
  sensor_msgs,
  std_msgs,
} from "https://esm.sh/jsr/@dimos/msgs@0.1.4";

// -- Channels ----------------------------------------------------------------
const CH_IMAGE = "/color_image#sensor_msgs.Image";
const CH_DEPTH = "/depth_image#sensor_msgs.Image";

// -- Default publish rates (ms) ----------------------------------------------
const DEFAULT_RATES: PublishRates = { images: 200 }; // 5 Hz images

// -- Types --------------------------------------------------------------------

export interface PublishRates { images: number; }
export interface SensorEnable { depth: boolean; }

export interface RgbFrame {
  data: Uint8Array;
  width: number;
  height: number;
}

export interface DepthFrame {
  data: Float32Array;
  width: number;
  height: number;
}

export interface SensorSources {
  captureRgb: () => RgbFrame | null;
  captureDepth: () => DepthFrame | null;
}

export interface DimosBridgeOptions {
  wsUrl?: string;
  agent: any;
  sensorSources: SensorSources;
  rates?: Partial<PublishRates>;
  sensorEnable?: Partial<SensorEnable>;
}

const CONTROL_HEARTBEAT_TIMEOUT_MS = 30_000;

export class DimosBridge {
  wsUrl: string;
  agent: any;
  sensors: SensorSources;
  rates: PublishRates;
  sensorEnable: SensorEnable;

  // Separate sockets so depth backlog does not starve RGB.
  wsControl: WebSocket | null;   // JSON commands + server pose (tiny, real-time)
  wsSensors: WebSocket | null;   // Rapier snapshots
  wsRgb: WebSocket | null;       // color image
  wsDepth: WebSocket | null;     // depth image

  // Keep legacy .ws alias pointing to control for compatibility
  get ws(): WebSocket | null { return this.wsControl; }

  _timers: Record<string, ReturnType<typeof setInterval>>;
  _connected: boolean;
  _publishingPaused: boolean;
  _controlHeartbeatTimer: ReturnType<typeof setInterval> | null;
  _lastControlHeartbeatAck: number;

  constructor({ wsUrl, agent, sensorSources, rates, sensorEnable }: DimosBridgeOptions) {
    const protocol = location.protocol === "https:" ? "wss:" : "ws:";
    this.wsUrl = wsUrl || `${protocol}//${location.host}`;
    this.agent = agent;
    this.sensors = sensorSources;
    this.rates = { ...DEFAULT_RATES, ...rates };
    this.sensorEnable = { depth: true, ...sensorEnable };
    this.wsControl = null;
    this.wsSensors = null;
    this.wsRgb = null;
    this.wsDepth = null;
    this._timers = {};
    this._connected = false;
    this._publishingPaused = false;
    this._lastRgbPublishedAt = 0;
    this._serverPhysicsReady = false;
    this._controlHeartbeatTimer = null;
    this._lastControlHeartbeatAck = 0;
  }

  connect(): void {
    // Read channel from URL param (for multi-page parallel evals)
    const channel = new URLSearchParams(location.search).get("channel") || "";
    const channelSuffix = channel ? `&channel=${channel}` : "";
    const isLive = (socket: WebSocket | null): boolean =>
      socket !== null &&
      (socket.readyState === WebSocket.CONNECTING ||
        socket.readyState === WebSocket.OPEN);

    // Control socket: JSON commands out, server pose + embodiment config in
    if (!isLive(this.wsControl)) {
      const control = new WebSocket(
        this.wsUrl + "?ch=control&client=browser" + channelSuffix,
      );
      this.wsControl = control;
      control.binaryType = "arraybuffer";

      control.onopen = () => {
        if (this.wsControl !== control) {
          control.close();
          return;
        }
        console.log("[DimosBridge] control WS connected");
        this._connected = true;
        this._startControlHeartbeat(control);
        this._startPublishing();
        this._flushPendingCommands();
      };

      // Keep the bridge's transport handler independent from consumers such
      // as EvalHarness. Replacing `onmessage` in a consumer can otherwise
      // disable heartbeat acknowledgements and authoritative pose updates,
      // causing a reconnect loop while an eval appears to keep running.
      control.addEventListener("message", (event: MessageEvent) => {
        // Text messages: server-side physics pose updates + embodiment config
        if (typeof event.data === "string") {
          try {
            const msg = JSON.parse(event.data);
            if (msg.type === "heartbeatAck") {
              this._lastControlHeartbeatAck = Date.now();
              if (msg.command) {
                queueMicrotask(() => {
                  if (this.wsControl !== control) return;
                  control.dispatchEvent(
                    new MessageEvent("message", {
                      data: JSON.stringify(msg.command),
                    }),
                  );
                });
              }
            } else if (msg.type === "pose") {
              this._handleServerPose(msg.x, msg.y, msg.z, msg.yaw);
            } else if (msg.type === "physicsReady") {
              this._serverPhysicsReady = true;
            } else if (msg.type === "embodimentConfig") {
              this._handleEmbodimentConfig(msg);
            }
          } catch {}
        }
      });

      control.onclose = () => {
        if (this.wsControl !== control) return;
        this._stopControlHeartbeat();
        console.log(
          "[DimosBridge] control WS disconnected, reconnecting in 2s...",
        );
        this._connected = false;
        this._serverPhysicsReady = false;
        this._stopPublishing();
        setTimeout(() => {
          if (this.wsControl === control) this.connect();
        }, 2000);
      };

      control.onerror = () => {};
    }

    // Sensor socket: Rapier snapshots out (no incoming expected)
    if (!isLive(this.wsSensors)) {
      const sensors = new WebSocket(
        this.wsUrl + "?ch=sensors" + channelSuffix,
      );
      this.wsSensors = sensors;
      sensors.binaryType = "arraybuffer";
      sensors.onopen = () => {
        if (this.wsSensors === sensors) {
          console.log("[DimosBridge] sensor WS connected");
        }
      };
      sensors.onclose = () => {
        if (this.wsSensors !== sensors) return;
        console.log("[DimosBridge] sensor WS disconnected");
        setTimeout(() => {
          if (this.wsSensors === sensors) this.connect();
        }, 2000);
      };
      sensors.onerror = () => {};
    }

    if (!isLive(this.wsRgb)) {
      const rgb = new WebSocket(this.wsUrl + "?ch=rgb" + channelSuffix);
      this.wsRgb = rgb;
      rgb.binaryType = "arraybuffer";
      rgb.onclose = () => {
        if (this.wsRgb !== rgb) return;
        console.log("[DimosBridge] RGB WS disconnected");
        setTimeout(() => {
          if (this.wsRgb === rgb) this.connect();
        }, 2000);
      };
      rgb.onerror = () => {};
    }

    if (!isLive(this.wsDepth)) {
      const depth = new WebSocket(
        this.wsUrl + "?ch=depth" + channelSuffix,
      );
      this.wsDepth = depth;
      depth.binaryType = "arraybuffer";
      depth.onclose = () => {
        if (this.wsDepth !== depth) return;
        console.log("[DimosBridge] depth WS disconnected");
        setTimeout(() => {
          if (this.wsDepth === depth) this.connect();
        }, 2000);
      };
      depth.onerror = () => {};
    }
  }

  // -- Incoming messages ------------------------------------------------------

  _startControlHeartbeat(control: WebSocket): void {
    this._stopControlHeartbeat();
    this._lastControlHeartbeatAck = Date.now();
    this._controlHeartbeatTimer = setInterval(() => {
      if (this.wsControl !== control) {
        this._stopControlHeartbeat();
        return;
      }
      if (
        control.readyState !== WebSocket.OPEN ||
        Date.now() - this._lastControlHeartbeatAck >
          CONTROL_HEARTBEAT_TIMEOUT_MS
      ) {
        this.wsControl = null;
        this._stopControlHeartbeat();
        try {
          control.close();
        } catch { /* ignore */ }
        this.connect();
        return;
      }
      try {
        control.send(JSON.stringify({ type: "heartbeat", ts: Date.now() }));
      } catch { /* watchdog reconnects on the next tick */ }
      // Background Chrome tabs can throttle the completion-scheduled camera
      // setTimeout for a minute or more even while WebSocket heartbeats remain
      // reliable. Use that live heartbeat as a bounded sensor fallback.
      const cameraFallbackMs = Math.max(5_000, this.rates.images * 3);
      if (
        !this._publishingPaused &&
        this.rates.images > 0 &&
        Date.now() - this._lastRgbPublishedAt >= cameraFallbackMs
      ) {
        this._publishImages();
      }
    }, 3000);
  }

  _stopControlHeartbeat(): void {
    if (this._controlHeartbeatTimer) {
      clearInterval(this._controlHeartbeatTimer);
      this._controlHeartbeatTimer = null;
    }
  }

  /** Handle server-side physics pose update (Three.js Y-up frame). */
  _handleServerPose(x: number, y: number, z: number, yaw: number): void {
    if (!this.agent) return;
    // Move the agent body to the server-authoritative position
    if (this.agent.body) {
      this.agent.body.setTranslation?.({ x, y, z }, true);
      this.agent.body.setNextKinematicTranslation({ x, y, z });
    }
    if (this.agent.group) {
      this.agent.group.position?.set?.(x, y, z);
      this.agent.group.rotation.y = yaw;
    }
    // Update engine's _dimosYaw for sensor capture / odom pose reading
    if ((window as any).__dimosSetYaw) {
      (window as any).__dimosSetYaw(yaw);
    }
    // Store for odom/sensor capture
    this._serverPose = { x, y, z, yaw };
  }

  _serverPose: { x: number; y: number; z: number; yaw: number } | null = null;
  _serverPhysicsReady = false;
  _lastRgbPublishedAt = 0;

  _handleEmbodimentConfig(msg: any): void {
    console.log("[DimosBridge] embodiment config received:", msg.embodimentType || "quadruped");
    // Swap the agent's avatar model if avatarUrl changed
    if (this.agent && msg.avatarUrl) {
      const urls = Array.isArray(msg.avatarUrl) ? msg.avatarUrl : [msg.avatarUrl];
      this.agent.avatarUrl = urls;
      // Update dimensions on the agent so _applyGLB auto-fits correctly
      if (msg.radius != null) this.agent.radius = msg.radius;
      if (msg.halfHeight != null) this.agent.halfHeight = msg.halfHeight;
      // Remove current model and reload
      if (this.agent.model) {
        this.agent.group.remove(this.agent.model);
        this.agent.model = null;
      }
      this.agent._loadGLB();
      // Scenes that return `embodiment: null` ship with the agent's group
      // hidden (engine.js sets `group.visible = false`).  Dimos sending an
      // embodimentConfig is the signal that an external agent is now
      // driving — re-enable visibility so the model actually renders.
      if (this.agent.group) this.agent.group.visible = true;
    }
  }

  // -- Outgoing sensor data ---------------------------------------------------

  _startPublishing(): void {
    // Reconnects and explicit resume calls must not create duplicate timers.
    this._stopPublishing();
    if (this._publishingPaused) return;
    // Images default 5 Hz (configurable via rates.images).
    // Odom and lidar are published server-side via LCM directly.
    if (this.rates.images > 0) {
      const publishImages = () => {
        if (this._publishingPaused) return;
        this._publishImages();
        // Schedule from completion instead of using setInterval. Software
        // WebGL capture can overrun its nominal period; setInterval then runs
        // another expensive capture immediately and starves control/eval
        // messages for tens of seconds.
        if (!this._publishingPaused && this.rates.images > 0) {
          this._timers["images"] = setTimeout(
            publishImages,
            this.rates.images,
          );
        }
      };
      this._timers["images"] = setTimeout(
        publishImages,
        this.rates.images,
      );
    }
  }

  _makeHeader(frameId: string): any {
    const now = Date.now();
    return new std_msgs.Header({
      stamp: new std_msgs.Time({ sec: Math.floor(now / 1000), nsec: (now % 1000) * 1_000_000 }),
      frame_id: frameId,
    });
  }

  _publishImages(): void {
    if (!this._isSocketOpen(this.wsRgb) && !this._isSocketOpen(this.wsDepth)) return;
    const camHeader = this._makeHeader("camera_optical");
    if (this._isSocketOpen(this.wsRgb)) this._publishRgbSync(camHeader);
    if (this.sensorEnable.depth && this._isSocketOpen(this.wsDepth)) this._publishDepthSync(camHeader);
  }

  _stopPublishing(): void {
    for (const k of Object.keys(this._timers)) clearInterval(this._timers[k]);
    this._timers = {};
  }

  pausePublishing(): void {
    this._publishingPaused = true;
    this._stopPublishing();
  }

  resumePublishing(): void {
    this._publishingPaused = false;
    this._startPublishing();
  }

  /** Send on a sensor WebSocket (images — large data). */
  _sendSensor(ws: WebSocket | null, channel: string, msg: any): void {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(encodePacket(channel, msg));
  }

  // -- RGB --------------------------------------------------------------------

  _publishRgbSync(header: any): void {
    try {
      if (!this._isSocketOpen(this.wsRgb)) return;
      const frame = this.sensors.captureRgb();
      if (!frame) return;

      this._sendSensor(
        this.wsRgb,
        CH_IMAGE,
        new sensor_msgs.Image({
          header,
          height: frame.height,
          width: frame.width,
          encoding: "jpeg",
          is_bigendian: 0,
          step: 0, // not applicable for compressed format
          data_length: frame.data.length,
          data: frame.data,
        }),
      );
      this._lastRgbPublishedAt = Date.now();
    } catch (e) {
      console.warn("[DimosBridge] RGB publish error:", e);
    }
  }

  // -- Depth ------------------------------------------------------------------

  _depthU16: Uint16Array | null = null;

  _publishDepthSync(header: any): void {
    try {
      if (!this._isSocketOpen(this.wsDepth)) return;
      const frame = this.sensors.captureDepth();
      if (!frame) return;

      // Quantize float32 meters → uint16 millimeters (0–65.535m range, 1mm precision)
      const n = frame.data.length;
      if (!this._depthU16 || this._depthU16.length !== n) {
        this._depthU16 = new Uint16Array(n);
      }
      const f32 = frame.data;
      const u16 = this._depthU16;
      for (let i = 0; i < n; i++) {
        const mm = f32[i] * 1000;
        u16[i] = mm > 65535 ? 65535 : mm < 0 ? 0 : mm;
      }
      const depthBytes = new Uint8Array(u16.buffer, u16.byteOffset, u16.byteLength);

      this._sendSensor(this.wsDepth, CH_DEPTH, new sensor_msgs.Image({
        header,
        height: frame.height,
        width: frame.width,
        encoding: "16UC1",
        is_bigendian: 0,
        step: frame.width * 2,
        data_length: depthBytes.length,
        data: depthBytes,
      }));
    } catch (e) {
      console.warn("[DimosBridge] depth publish error:", e);
    }
  }

  _pendingCommands: Record<string, any>[] = [];

  /** Send a JSON command on the control WebSocket.  Queues if the socket
   * isn't OPEN yet — _flushPendingCommands() drains on onopen. */
  sendCommand(cmd: Record<string, any>): void {
    const ws = this.wsControl;
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(cmd));
    } else {
      this._pendingCommands.push(cmd);
    }
  }

  _flushPendingCommands(): void {
    const ws = this.wsControl;
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    const pending = this._pendingCommands;
    this._pendingCommands = [];
    if (pending.length) console.log(`[DimosBridge] flushing ${pending.length} queued command(s)`);
    for (const cmd of pending) ws.send(JSON.stringify(cmd));
  }

  _isSocketOpen(ws: WebSocket | null): boolean {
    return !!ws && ws.readyState === WebSocket.OPEN;
  }
}
