/**
 * DimosBridge — Browser-side WebSocket client for dimos integration.
 *
 * Uses separate WebSocket connections so large sensor streams do not block
 * each other or real-time odom/cmd_vel:
 *   wsControl  → /odom, /cmd_vel  (tiny packets, real-time)
 *   wsSensors  → /lidar + snapshots
 *   wsRgb      → /color_image
 *   wsDepth    → /depth_image
 *
 * All messages are LCM-encoded binary packets using @dimos/msgs, sent over
 * WebSocket to the bridge server which relays them to/from dimos via LCM/UDP.
 */

// @ts-ignore — CDN import (runs in browser, no Deno/Node type resolution)
import {
  encodePacket,
  decodePacket,
  geometry_msgs,
  sensor_msgs,
  std_msgs,
} from "https://esm.sh/jsr/@dimos/msgs@0.1.4";

// -- Channels ----------------------------------------------------------------
const CH_CMD_VEL = "/cmd_vel#geometry_msgs.Twist";
const CH_ODOM = "/odom#geometry_msgs.PoseStamped";
const CH_IMAGE = "/color_image#sensor_msgs.Image";
const CH_DEPTH = "/depth_image#sensor_msgs.Image";
const CH_LIDAR = "/lidar#sensor_msgs.PointCloud2";

// -- Default publish rates (ms) ----------------------------------------------
const DEFAULT_RATES: PublishRates = { odom: 20, lidar: 200, images: 200 }; // 50 Hz odom, 5 Hz lidar, 5 Hz images
const CMD_VEL_TIMEOUT_MS = 500;
const BRIDGE_DEBUG = false;
const LIDAR_POINT_STEP = 16;

// -- Types --------------------------------------------------------------------

export interface PublishRates { odom: number; lidar: number; images: number; }
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

export interface LidarFrame {
  numPoints: number;
  points: Float32Array;    // N*3 interleaved XYZ
  intensity?: Float32Array; // N
}

export interface OdomPose {
  x: number; y: number; z: number;
  qx: number; qy: number; qz: number; qw: number;
}

export interface SensorSources {
  captureRgb: () => RgbFrame | null;
  captureDepth: () => DepthFrame | null;
  captureLidar: () => LidarFrame | null;
  getOdomPose: () => OdomPose | null;
}

export type FrameTransform = "identity" | "ros";

export interface DimosBridgeOptions {
  wsUrl?: string;
  agent: any;
  sensorSources: SensorSources;
  rates?: Partial<PublishRates>;
  sensorEnable?: Partial<SensorEnable>;
  frameTransform?: FrameTransform;
}

export class DimosBridge {
  wsUrl: string;
  agent: any;
  sensors: SensorSources;
  rates: PublishRates;
  sensorEnable: SensorEnable;
  frameTransform: FrameTransform;

  // Separate sockets so depth backlog does not starve RGB.
  wsControl: WebSocket | null;   // odom + cmd_vel (tiny, real-time)
  wsSensors: WebSocket | null;   // lidar + snapshots
  wsRgb: WebSocket | null;       // color image
  wsDepth: WebSocket | null;     // depth image

  // Keep legacy .ws alias pointing to control for compatibility
  get ws(): WebSocket | null { return this.wsControl; }

  _timers: Record<string, ReturnType<typeof setInterval>>;
  _dirty: { odom: boolean; lidar: boolean; images: boolean };
  _rafId: number | null;
  _connected: boolean;

  _cmdVel: { linX: number; linY: number; linZ: number; angX: number; angY: number; angZ: number } | null;
  _cmdVelStamp: number;
  _serverLidar: boolean;
  _lidarBuf: ArrayBuffer;
  _lidarView: DataView;
  _lidarCapacityPoints: number;
  _pc2Fields: any[];

  constructor({ wsUrl, agent, sensorSources, rates, sensorEnable, frameTransform }: DimosBridgeOptions) {
    const protocol = location.protocol === "https:" ? "wss:" : "ws:";
    this.wsUrl = wsUrl || `${protocol}//${location.host}`;
    this.agent = agent;
    this.sensors = sensorSources;
    this.rates = { ...DEFAULT_RATES, ...rates };
    this.sensorEnable = { depth: true, ...sensorEnable };
    this.frameTransform = frameTransform || "ros";
    this.wsControl = null;
    this.wsSensors = null;
    this.wsRgb = null;
    this.wsDepth = null;
    this._timers = {};
    this._dirty = { odom: false, lidar: false, images: false };
    this._rafId = null;
    this._connected = false;
    this._cmdVel = null;
    this._cmdVelStamp = 0;
    this._serverLidar = false;
    this._lidarBuf = new ArrayBuffer(0);
    this._lidarView = new DataView(this._lidarBuf);
    this._lidarCapacityPoints = 0;
    this._pc2Fields = [
      new sensor_msgs.PointField({ name: "x", offset: 0, datatype: 7, count: 1 }),
      new sensor_msgs.PointField({ name: "y", offset: 4, datatype: 7, count: 1 }),
      new sensor_msgs.PointField({ name: "z", offset: 8, datatype: 7, count: 1 }),
      new sensor_msgs.PointField({ name: "intensity", offset: 12, datatype: 7, count: 1 }),
    ];
  }

  connect(): void {
    // Read channel from URL param (for multi-page parallel evals)
    const channel = new URLSearchParams(location.search).get("channel") || "";
    const channelSuffix = channel ? `&channel=${channel}` : "";

    // Control socket: odom out, cmd_vel in
    this.wsControl = new WebSocket(this.wsUrl + "?ch=control" + channelSuffix);
    this.wsControl.binaryType = "arraybuffer";

    this.wsControl.onopen = () => {
      console.log("[DimosBridge] control WS connected");
      this._connected = true;
      this._startPublishing();
      this._flushPendingCommands();
    };

    this.wsControl.onmessage = (event: MessageEvent) => {
      // Text messages: server-side physics pose updates + embodiment config
      if (typeof event.data === "string") {
        try {
          const msg = JSON.parse(event.data);
          if (msg.type === "pose") {
            this._handleServerPose(msg.x, msg.y, msg.z, msg.yaw);
          } else if (msg.type === "embodimentConfig") {
            this._handleEmbodimentConfig(msg);
          }
        } catch {}
        return;
      }
      // Binary messages: LCM packets (cmd_vel relay)
      if (!(event.data instanceof ArrayBuffer)) return;
      try {
        const raw = new Uint8Array(event.data);
        const { channel, data } = decodePacket(raw);
        this._handlePacket(channel, data);
      } catch {}
    };

    this.wsControl.onclose = () => {
      console.log("[DimosBridge] control WS disconnected, reconnecting in 2s...");
      this._connected = false;
      this._stopPublishing();
      setTimeout(() => this.connect(), 2000);
    };

    this.wsControl.onerror = () => {};

    // Sensor socket: lidar + snapshots out (no incoming expected)
    this.wsSensors = new WebSocket(this.wsUrl + "?ch=sensors" + channelSuffix);
    this.wsSensors.binaryType = "arraybuffer";

    this.wsSensors.onopen = () => {
      console.log("[DimosBridge] sensor WS connected");
    };

    this.wsSensors.onclose = () => {
      console.log("[DimosBridge] sensor WS disconnected");
    };

    this.wsSensors.onerror = () => {};

    this.wsRgb = new WebSocket(this.wsUrl + "?ch=rgb" + channelSuffix);
    this.wsRgb.binaryType = "arraybuffer";
    this.wsRgb.onclose = () => {
      console.log("[DimosBridge] RGB WS disconnected");
    };
    this.wsRgb.onerror = () => {};

    this.wsDepth = new WebSocket(this.wsUrl + "?ch=depth" + channelSuffix);
    this.wsDepth.binaryType = "arraybuffer";
    this.wsDepth.onclose = () => {
      console.log("[DimosBridge] depth WS disconnected");
    };
    this.wsDepth.onerror = () => {};
  }

  // -- Incoming packets -------------------------------------------------------

  _handlePacket(channel: string, data: any): void {
    if (channel === CH_CMD_VEL) {
      this._handleCmdVel(data);
    }
  }

  _handleCmdVel(twist: any): void {
    const lin = twist.linear;
    const ang = twist.angular;

    let linX: number, linY: number, linZ: number;
    let angX: number, angY: number, angZ: number;

    if (this.frameTransform === "ros") {
      // ROS → Three.js: inverse of the cyclic permutation (x→y, y→z, z→x)
      linX = lin.y;
      linY = lin.z;
      linZ = lin.x;
      angX = ang.y;
      angY = ang.z;
      angZ = ang.x;
    } else {
      linX = lin.x; linY = lin.y; linZ = lin.z;
      angX = ang.x; angY = ang.y; angZ = ang.z;
    }

    this._cmdVel = { linX, linY, linZ, angX, angY, angZ };
    this._cmdVelStamp = Date.now();
  }

  /** Get current velocity, auto-zeroing after CMD_VEL_TIMEOUT_MS (safety stop). */
  getCmdVel(): { linX: number; linY: number; linZ: number; angX: number; angY: number; angZ: number } {
    if (!this._cmdVel || Date.now() - this._cmdVelStamp > CMD_VEL_TIMEOUT_MS) {
      return { linX: 0, linY: 0, linZ: 0, angX: 0, angY: 0, angZ: 0 };
    }
    return this._cmdVel;
  }

  /** Handle server-side physics pose update (Three.js Y-up frame). */
  _handleServerPose(x: number, y: number, z: number, yaw: number): void {
    if (!this.agent) return;
    // Move the agent body to the server-authoritative position
    if (this.agent.body) {
      this.agent.body.setNextKinematicTranslation({ x, y, z });
    }
    if (this.agent.group) {
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

  _handleEmbodimentConfig(msg: any): void {
    console.log("[DimosBridge] embodiment config received:", msg.embodimentType || "quadruped");
    // Apply config to engine globals (dimensions, speed, type)
    if ((window as any).applyEmbodiment) {
      (window as any).applyEmbodiment(msg);
    }
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

  sceneReady = false;

  _startPublishing(): void {
    // No lidar timer — server-side lidar handles it via LCM directly.
    // Images default 5 Hz (configurable via rates.images)
    if (this.rates.images > 0) {
      this._timers["images"] = setInterval(() => this._publishImages(), this.rates.images);
    }
  }

  _makeHeader(frameId: string): any {
    const now = Date.now();
    return new std_msgs.Header({
      stamp: new std_msgs.Time({ sec: Math.floor(now / 1000), nsec: (now % 1000) * 1_000_000 }),
      frame_id: frameId,
    });
  }

  _publishOdom(): void {
    if (!this._isControlSocketOpen()) return;
    this._publishOdomSync(this._makeHeader("world"));
  }

  _publishLidar(): void {
    if (!this._isSocketOpen(this.wsSensors)) return;
    this._publishLidarSync(this._makeHeader("world"));
  }

  _publishImages(): void {
    if (!this._isSocketOpen(this.wsRgb) && !this._isSocketOpen(this.wsDepth)) return;
    const camHeader = this._makeHeader("camera_optical");
    if (this._isSocketOpen(this.wsRgb)) this._publishRgbSync(camHeader);
    if (this.sensorEnable.depth && this._isSocketOpen(this.wsDepth)) this._publishDepthSync(camHeader);
  }

  // -- Odom -------------------------------------------------------------------

  _odomDbgN = 0;

  _publishOdomSync(header: any): void {
    try {
      const pose = this.sensors.getOdomPose();
      if (!pose) return;

      this._odomDbgN++;

      // Three.js (Y-up) → ROS (Z-up) cyclic permutation: x→y, y→z, z→x
      const rosQx = pose.qz;
      const rosQy = pose.qx;
      const rosQz = pose.qy;
      const rosQw = pose.qw;

      const q = new geometry_msgs.Quaternion();
      q.x = rosQx; q.y = rosQy; q.z = rosQz; q.w = rosQw;
      const pt = new geometry_msgs.Point();
      pt.x = pose.z; pt.y = pose.x; pt.z = pose.y;
      const p = new geometry_msgs.Pose();
      p.position = pt;
      p.orientation = q;

      header.seq = this._odomDbgN;
      const odomMsg = new geometry_msgs.PoseStamped();
      odomMsg.header = header;
      odomMsg.pose = p;

      if (BRIDGE_DEBUG && (this._odomDbgN <= 3 || this._odomDbgN % 100 === 0)) {
        console.log(`[odom TX seq=${this._odomDbgN}] qz=${rosQz.toFixed(4)} qw=${rosQw.toFixed(4)}`);
      }

      // Send on CONTROL socket (not sensor socket)
      this._sendControl(CH_ODOM, odomMsg);
    } catch (e) {
      console.warn("[DimosBridge] odom publish error:", e);
    }
  }

  _stopPublishing(): void {
    for (const k of Object.keys(this._timers)) clearInterval(this._timers[k]);
    this._timers = {};
    if (this._rafId) cancelAnimationFrame(this._rafId);
    this._rafId = null;
  }

  /** Send on the control WebSocket (odom, small real-time data) */
  _sendControl(channel: string, msg: any): void {
    const ws = this.wsControl;
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(encodePacket(channel, msg));
  }

  /** Send on a sensor WebSocket (images, lidar — large data). */
  _sendSensor(ws: WebSocket | null, channel: string, msg: any): void {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(encodePacket(channel, msg));
  }

  /** Legacy _send — routes to appropriate socket based on channel */
  _send(channel: string, msg: any): void {
    if (channel === CH_ODOM) {
      this._sendControl(channel, msg);
    } else if (channel === CH_IMAGE) {
      this._sendSensor(this.wsRgb, channel, msg);
    } else if (channel === CH_DEPTH) {
      this._sendSensor(this.wsDepth, channel, msg);
    } else {
      this._sendSensor(this.wsSensors, channel, msg);
    }
  }

  // -- RGB --------------------------------------------------------------------

  _publishRgbSync(header: any): void {
    try {
      if (!this._isSocketOpen(this.wsRgb)) return;
      const frame = this.sensors.captureRgb();
      if (!frame) return;

      this._sendSensor(this.wsRgb, CH_IMAGE, new sensor_msgs.Image({
        header,
        height: frame.height,
        width: frame.width,
        encoding: "jpeg",
        is_bigendian: 0,
        step: 0,  // not applicable for compressed format
        data_length: frame.data.length,
        data: frame.data,
      }));
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

  // -- LiDAR ------------------------------------------------------------------

  _lidarDbgN = 0;
  _publishLidarSync(header: any): void {
    try {
      if (!this._isSocketOpen(this.wsSensors)) return;
      const frame = this.sensors.captureLidar();
      this._lidarDbgN++;
      if (BRIDGE_DEBUG && (this._lidarDbgN <= 3 || this._lidarDbgN % 100 === 0)) {
        console.log(`[DimosBridge] lidar #${this._lidarDbgN}: ${frame ? frame.numPoints : 'null'} pts, sensorWS=${this.wsSensors?.readyState}`);
      }
      if (!frame) return;

      const numPoints = frame.numPoints || 0;
      if (numPoints === 0) return;

      this._ensureLidarCapacity(numPoints);
      const pointStep = LIDAR_POINT_STEP;
      const view = this._lidarView;
      const pts = frame.points;
      const intensity = frame.intensity;

      // Points are Three.js world-frame (Y-up).
      // Convert to ROS world-frame (Z-up): cyclic permutation x→y, y→z, z→x
      for (let i = 0; i < numPoints; i++) {
        const off = i * pointStep;
        const tx = pts[i * 3 + 0], ty = pts[i * 3 + 1], tz = pts[i * 3 + 2];
        view.setFloat32(off,     tz, true);   // ROS x = Three.js z
        view.setFloat32(off + 4, tx, true);   // ROS y = Three.js x
        view.setFloat32(off + 8, ty, true);   // ROS z = Three.js y
        view.setFloat32(off + 12, intensity ? intensity[i] : 1.0, true);
      }

      this._sendSensor(this.wsSensors, CH_LIDAR, new sensor_msgs.PointCloud2({
        header,
        height: 1,
        width: numPoints,
        fields_length: this._pc2Fields.length,
        fields: this._pc2Fields,
        is_bigendian: 0,
        point_step: pointStep,
        row_step: numPoints * pointStep,
        data_length: numPoints * pointStep,
        data: new Uint8Array(this._lidarBuf, 0, numPoints * pointStep),
        is_dense: 1,
      }));
    } catch (e) {
      console.warn("[DimosBridge] LiDAR publish error:", e);
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

  dispose(): void {
    this._stopPublishing();
    if (this.wsControl) { this.wsControl.onclose = null; this.wsControl.close(); }
    if (this.wsSensors) { this.wsSensors.onclose = null; this.wsSensors.close(); }
    if (this.wsRgb) { this.wsRgb.onclose = null; this.wsRgb.close(); }
    if (this.wsDepth) { this.wsDepth.onclose = null; this.wsDepth.close(); }
    this.wsControl = null;
    this.wsSensors = null;
    this.wsRgb = null;
    this.wsDepth = null;
  }

  _isControlSocketOpen(): boolean {
    return !!this.wsControl && this.wsControl.readyState === WebSocket.OPEN;
  }

  _isSocketOpen(ws: WebSocket | null): boolean {
    return !!ws && ws.readyState === WebSocket.OPEN;
  }

  _ensureLidarCapacity(numPoints: number): void {
    if (numPoints <= this._lidarCapacityPoints) return;
    this._lidarCapacityPoints = numPoints;
    this._lidarBuf = new ArrayBuffer(numPoints * LIDAR_POINT_STEP);
    this._lidarView = new DataView(this._lidarBuf);
  }
}
