import { type Msg } from "../vendor/dimos/web/shared/protocol.ts";
import { Registry, type ViewerPeer } from "../vendor/dimos/web/relay/registry.ts";

import roster from "../assets/scenes/apartment/multiplayer.json" with {
  type: "json",
};

export const ROBOTS = Object.keys(roster.robots);
export const VISITORS = ROBOTS;
export const RECONNECT_MS = 60_000;
const READ_CHANNELS = new Set([
  "world_state",
  "world_compare_image",
  "color_image",
  ...Array.from({ length: 6 }, (_, i) => `duck${i + 1}_ball_camera`),
]);
const CONTROL = new Set(["teleop_start", "teleop_stop", "twist", "stop", "tx"]);
export class NameError extends Error {}
export function normalizeName(value: unknown, fallback: string): string {
  if (value === undefined) return fallback;
  if (typeof value !== "string") {
    throw new NameError("Enter a player name using text.");
  }
  const name = value.normalize("NFKC").replace(/[\p{Cc}\p{Cf}]/gu, "").trim()
    .replace(/\s+/gu, " ");
  if (Array.from(name).length > 24) {
    throw new NameError("Keep your player name to 24 characters.");
  }
  return name || fallback;
}
export interface Participant {
  token: string;
  userId?: string;
  userLogin?: string;
  role: "observe" | "visitor" | "host";
  robot: string;
  generation: string;
  displayName: string;
  disconnectedAt: number;
  lastSeen: number;
  connections: Map<number, () => void>;
}

/** Application ownership policy, composed with the stock DimOS relay. */
export class Lobby extends Registry {
  participants = new Map<string, Participant>();
  peers = new Map<number, Participant>();
  constructor(readonly now: () => number = Date.now) {
    super();
  }

  sweep(): void {
    const now = this.now();
    for (const p of this.participants.values()) {
      if (p.connections.size === 0 && now - p.disconnectedAt >= RECONNECT_MS) {
        p.role = "observe";
        p.robot = "world";
        p.displayName = "";
      }
      if (!p.connections.size && now - p.lastSeen > 3_600_000) {
        this.participants.delete(p.token);
      }
    }
  }

  create(): Participant | null {
    this.sweep();
    if (this.participants.size >= 128) return null;
    const p: Participant = {
      token: crypto.randomUUID() + crypto.randomUUID(),
      role: "observe",
      robot: "world",
      generation: "",
      displayName: "",
      disconnectedAt: this.now(),
      lastSeen: this.now(),
      connections: new Map(),
    };
    this.participants.set(p.token, p);
    return p;
  }

  forUser(userId: string): Participant | null {
    this.sweep();
    const existing = [...this.participants.values()].find((p) => p.userId === userId);
    if (existing) return existing;
    const p = this.create();
    if (p) p.userId = userId;
    return p;
  }

  revokeUser(userId: string): void {
    for (const p of this.participants.values()) {
      if (p.userId !== userId) continue;
      this.change(p, "observe");
      for (const [id, close] of p.connections) {
        this.peers.delete(id);
        close();
      }
      p.connections.clear();
      this.participants.delete(p.token);
    }
  }

  find(token: string | null): Participant | undefined {
    this.sweep();
    return token === null ? undefined : this.participants.get(token);
  }

  change(
    p: Participant,
    action: "observe" | "join" | "host",
    requested?: string,
    requestedName?: unknown,
  ): boolean {
    this.sweep();
    if (requested !== undefined && !ROBOTS.includes(requested)) return false;
    const occupied = new Set(
      [...this.participants.values()]
        .filter((other) => other !== p && other.role !== "observe").map((
          other,
        ) => other.robot),
    );
    const robot = action === "join"
      ? requested ??
        (p.role === "visitor" ? p.robot : VISITORS.find((id) => !occupied.has(id)))
      : action === "host"
      ? "duck1"
      : "world";
    if (!robot || (robot !== "world" && occupied.has(robot))) return false;
    const role = robot === "world" ? "observe" : "visitor";
    const displayName = robot === "world" ? "" : normalizeName(
      requestedName === undefined ? (p.role === "visitor" ? p.displayName : "") : requestedName,
      roster.robots[robot as keyof typeof roster.robots].name,
    );
    if (
      displayName &&
      [...this.participants.values()].some((other) =>
        other !== p && other.role === "visitor" &&
        other.displayName.toLocaleLowerCase() ===
          displayName.toLocaleLowerCase()
      )
    ) throw new NameError("That player name is in use. Choose another name.");
    // Renaming the same occupied slot never restarts its robot or loses knowledge.
    if (p.role === role && p.robot === robot) {
      p.displayName = displayName;
      return true;
    }
    // Revoke old command authority before closing any transport. A close callback
    // may run asynchronously; queued messages must already be unauthorized.
    for (const [id, close] of p.connections) {
      this.peers.delete(id);
      close();
    }
    p.connections.clear();
    p.role = role;
    p.robot = robot;
    p.displayName = displayName;
    p.generation = crypto.randomUUID();
    p.disconnectedAt = p.lastSeen = this.now();
    return true;
  }

  attach(viewer: ViewerPeer, p: Participant, close: () => void): void {
    // One live tab per ticket. Reload replaces the old transport without allowing
    // two sessions to drive one participant at the same time.
    for (const [id, previous] of p.connections) {
      this.peers.delete(id);
      previous();
    }
    p.connections.clear();
    p.connections.set(viewer.id, close);
    p.lastSeen = this.now();
    this.peers.set(viewer.id, p);
  }

  override viewerClosed(viewer: ViewerPeer): void {
    const p = this.peers.get(viewer.id);
    this.peers.delete(viewer.id);
    if (p) {
      p.connections.delete(viewer.id);
      if (!p.connections.size) p.disconnectedAt = this.now();
    }
    super.viewerClosed(viewer);
  }

  runtime(p: Participant): string {
    return p.role === "visitor" ? `${p.robot}-${p.generation}` : p.robot;
  }

  override onViewerMsg(
    viewer: ViewerPeer,
    msg: Msg,
    reply: (msg: Msg) => void,
  ): boolean {
    const p = this.peers.get(viewer.id);
    if (!p) return false;
    p.lastSeen = this.now();
    const denied = (CONTROL.has(msg.t) &&
      (p.role === "observe" || viewer.watched !== this.runtime(p))) ||
      (msg.t === "watch" && msg.robotId !== this.runtime(p)) ||
      (msg.t === "sub" && p.role === "observe" && !READ_CHANNELS.has(msg.ch));
    if (denied) {
      reply({
        t: "error",
        code: "forbidden",
        message: "This session cannot control that robot.",
      });
      return true;
    }
    return super.onViewerMsg(viewer, msg, reply);
  }

  state(p?: Participant) {
    this.sweep();
    return {
      role: p?.role ?? null,
      robot: p?.robot ?? null,
      displayName: p?.displayName || null,
      runtime: p ? this.runtime(p) : null,
      slots: ROBOTS.map((id) => {
        const occupant = [...this.participants.values()].find((p) =>
          p.role !== "observe" && p.robot === id
        );
        return {
          id,
          ...roster.robots[id as keyof typeof roster.robots],
          displayName: occupant?.displayName || null,
          generation: occupant?.generation ?? null,
          occupied: !!occupant,
          connected: !!occupant?.connections.size,
          mine: !!occupant && occupant.token === p?.token,
          reconnectSeconds: occupant && !occupant.connections.size
            ? Math.max(
              0,
              Math.ceil(
                (RECONNECT_MS - (this.now() - occupant.disconnectedAt)) / 1000,
              ),
            )
            : 0,
        };
      }),
      graceSeconds: RECONNECT_MS / 1000,
    };
  }

  scorerIdentities(): Record<string, { generation: string; userId: string; handle: string }> {
    this.sweep();
    return Object.fromEntries([...this.participants.values()]
      .filter(p => p.role === "visitor" && p.userId && p.userLogin)
      .map(p => [p.robot, { generation: p.generation, userId: p.userId!, handle: p.userLogin! }]));
  }

  assignments(): Record<string, string> {
    this.sweep();
    return Object.fromEntries(
      [...this.participants.values()]
        .filter((p) => p.role === "visitor").map(
          (p) => [p.robot, p.generation],
        ),
    );
  }
}
