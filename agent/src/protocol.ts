import { randomUUID } from "node:crypto";
import { createConnection, type Socket } from "node:net";
import { StringDecoder } from "node:string_decoder";
import type {
  AgentSession,
  AgentSessionEvent,
} from "@earendil-works/pi-coding-agent";
import type { AuthEvent, AuthPrompt } from "@earendil-works/pi-ai";
import { z } from "zod";

const image = z.object({
  type: z.literal("image"),
  data: z.string(),
  mimeType: z.string(),
});
export const commandSchema = z.discriminatedUnion("type", [
  z.object({ type: z.literal("new_session"), cwd: z.string().optional() }),
  z.object({
    type: z.literal("attach"),
    sessionId: z.string(),
    writable: z.boolean().default(true),
  }),
  z.object({ type: z.literal("list_sessions") }),
  z.object({ type: z.literal("shutdown") }),
  z.object({
    type: z.literal("prompt"),
    message: z.string(),
    images: z.array(image).optional(),
  }),
  z.object({ type: z.literal("steer"), message: z.string() }),
  z.object({ type: z.literal("follow_up"), message: z.string() }),
  z.object({ type: z.literal("abort") }),
  z.object({ type: z.literal("get_state") }),
  z.object({ type: z.literal("get_available_models") }),
  z.object({
    type: z.literal("set_model"),
    provider: z.string(),
    modelId: z.string(),
  }),
  z.object({
    type: z.literal("login"),
    provider: z.string(),
    authType: z.enum(["api_key", "oauth"]),
  }),
  z.object({ type: z.literal("logout"), provider: z.string() }),
  z.object({
    type: z.literal("ui_response"),
    promptId: z.string(),
    value: z.string().optional(),
  }),
  z.object({ type: z.literal("reload") }),
]);
export const requestSchema = z.object({ id: z.uuid(), command: commandSchema });
export type Command = z.infer<typeof commandSchema>;
export type Request = z.infer<typeof requestSchema>;
export type Kind = Command["type"];
export type SessionEvent =
  | Exclude<AgentSessionEvent, { type: "message_update" }>
  | { type: "text_delta"; delta: string };
type WithoutSignal<T> = T extends unknown ? Omit<T, "signal"> : never;
export type Event =
  | SessionEvent
  | { type: "notice" | "turn_error"; message: string }
  | { type: "auth_prompt"; promptId: string; prompt: WithoutSignal<AuthPrompt> }
  | { type: "auth_info"; info: AuthEvent }
  | { type: "idle" };
export interface Snapshot {
  sessionId: string;
  cwd: string;
  seq: number;
  busy: boolean;
  writable: boolean;
  messages: AgentSession["messages"];
  text: string;
  notices: string[];
  tools: Array<
    Extract<
      AgentSessionEvent,
      { type: "tool_execution_start" | "tool_execution_update" }
    >
  >;
}
export interface Results {
  new_session: Snapshot;
  attach: Snapshot;
  get_state: Snapshot;
  list_sessions: Array<{ id: string; cwd: string; name: string }>;
  shutdown: null;
  prompt: null;
  steer: null;
  follow_up: null;
  abort: null;
  get_available_models: Array<{ provider: string; id: string; name: string }>;
  set_model: null;
  login: null;
  logout: null;
  ui_response: null;
  reload: null;
}
export type Packet =
  | { type: "response"; id: string; data?: Results[Kind]; error?: string }
  | { type: "event"; seq: number; event: Event };

const MAX_BYTES = 16 * 1024 * 1024;
export function send(socket: Socket, packet: Packet | Request): void {
  const line = JSON.stringify(packet) + "\n";
  if (
    Buffer.byteLength(line) > MAX_BYTES ||
    socket.writableLength > MAX_BYTES
  ) {
    socket.destroy(
      new Error("Terminal connection exceeded its bounded buffer"),
    );
    return;
  }
  socket.write(line);
}
export function readLines(
  socket: Socket,
  line: (value: unknown) => void,
): void {
  const decoder = new StringDecoder("utf8");
  let pending = "";
  socket.on("data", (chunk: Buffer) => {
    pending += decoder.write(chunk);
    if (Buffer.byteLength(pending) > MAX_BYTES) {
      socket.destroy(new Error("Message too large"));
      return;
    }
    let end: number;
    while ((end = pending.indexOf("\n")) >= 0) {
      const next = pending.slice(0, end);
      pending = pending.slice(end + 1);
      try {
        line(JSON.parse(next));
      } catch {
        socket.destroy(new Error("Invalid protocol message"));
        return;
      }
    }
  });
}
export class Connection {
  private readonly socket: Socket;
  private readonly pending = new Map<
    string,
    { resolve: (value: Results[Kind]) => void; reject: (error: Error) => void }
  >();
  onEvent: (seq: number, event: Event) => void = () => {};
  onClose: () => void = () => {};
  constructor(path: string) {
    this.socket = createConnection(path);
    readLines(this.socket, (value) => {
      // Both ends share Packet; only the user-owned local socket is accepted.
      const packet = value as Packet;
      if (packet.type === "event") this.onEvent(packet.seq, packet.event);
      else if (packet.type === "response") {
        const pending = this.pending.get(packet.id);
        this.pending.delete(packet.id);
        if (packet.error) pending?.reject(new Error(packet.error));
        else pending?.resolve(packet.data ?? null);
      } else throw new Error("Unexpected packet");
    });
    this.socket.on("error", (error) => this.fail(error));
    this.socket.on("close", () => {
      this.fail(new Error("Gateway disconnected"));
      this.onClose();
    });
  }
  call<K extends Kind>(
    command: Extract<Command, { type: K }>,
    id = randomUUID(),
  ): Promise<Results[K]> {
    if (this.socket.destroyed)
      return Promise.reject(new Error("Gateway disconnected"));
    return new Promise<Results[K]>((resolve, reject) => {
      this.pending.set(id, {
        resolve: (value) => resolve(value as Results[K]),
        reject,
      });
      send(this.socket, { id, command });
    });
  }
  close(): void {
    this.socket.destroy();
  }
  private fail(error: Error): void {
    for (const pending of this.pending.values()) pending.reject(error);
    this.pending.clear();
  }
}

export function reachable(path: string): Promise<boolean> {
  return new Promise((resolve) => {
    const socket = createConnection(path);
    socket.once("connect", () => {
      socket.destroy();
      resolve(true);
    });
    socket.once("error", () => resolve(false));
    socket.setTimeout(500, () => {
      socket.destroy();
      resolve(false);
    });
  });
}
