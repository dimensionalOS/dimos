import { randomUUID } from "node:crypto";
import { chmod, lstat, mkdir, unlink } from "node:fs/promises";
import { createServer, type Socket } from "node:net";
import { dirname } from "node:path";
import { SessionManager } from "@earendil-works/pi-coding-agent";
import type { AuthInteraction, AuthPrompt } from "@earendil-works/pi-ai";
import type { Config, Paths } from "./config.js";
import { openSession, type SessionHandle } from "./session.js";
import {
  type Command,
  type Event,
  type Packet,
  type Results,
  type Snapshot,
  readLines,
  reachable,
  requestSchema,
  send,
} from "./protocol.js";

interface Entry {
  handle: SessionHandle;
  peers: Set<Socket>;
  owner?: Socket;
  seq: number;
  running: boolean;
  text: string;
  notices: string[];
  tools: Map<string, Snapshot["tools"][number]>;
  accepted: Set<string>;
  dialogs: Map<
    string,
    { resolve: (value: string) => void; reject: (error: Error) => void }
  >;
}
export type SessionFactory = typeof openSession;
export class Gateway {
  private closing = false;
  private readonly sockets = new Set<Socket>();
  private readonly opening = new Map<string, Promise<Entry>>();
  private readonly entries = new Map<string, Entry>();
  private readonly peers = new Map<Socket, Entry>();
  private readonly server = createServer((socket) => this.connect(socket));
  constructor(
    private readonly config: Config,
    private readonly paths: Paths,
    private readonly factory: SessionFactory = openSession,
  ) {}
  async start(): Promise<void> {
    await mkdir(dirname(this.paths.socket), { recursive: true, mode: 0o700 });
    const listen = () =>
      new Promise<void>((resolve, reject) => {
        const failed = (error: Error) => {
          this.server.off("listening", ready);
          reject(error);
        };
        const ready = () => {
          this.server.off("error", failed);
          resolve();
        };
        this.server.once("error", failed);
        this.server.once("listening", ready);
        this.server.listen(this.paths.socket);
      });
    try {
      await listen();
    } catch (error) {
      if (
        !(
          error instanceof Error &&
          "code" in error &&
          error.code === "EADDRINUSE"
        ) ||
        !(await lstat(this.paths.socket)).isSocket() ||
        (await reachable(this.paths.socket))
      )
        throw error;
      await unlink(this.paths.socket);
      await listen();
    }
    await chmod(this.paths.socket, 0o600);
  }
  private emit(entry: Entry, event: Event): void {
    if (event.type === "notice" || event.type === "turn_error") {
      entry.notices.push(event.message);
      entry.notices = entry.notices.slice(-10);
    }
    const packet: Packet = { type: "event", seq: ++entry.seq, event };
    for (const peer of entry.peers) send(peer, packet);
  }
  private async restore(
    id?: string,
    cwd = this.config.workspace,
  ): Promise<Entry> {
    const key = id ?? randomUUID();
    const pending = this.opening.get(key);
    if (pending) return pending;
    const creating = this.makeEntry(id, cwd);
    this.opening.set(key, creating);
    try {
      return await creating;
    } finally {
      this.opening.delete(key);
    }
  }
  private async makeEntry(
    id?: string,
    cwd = this.config.workspace,
  ): Promise<Entry> {
    if (id && this.entries.has(id)) return this.entries.get(id)!;
    const saved = id
      ? (await SessionManager.listAll(this.paths.sessions)).find(
          (item) => item.id === id,
        )
      : undefined;
    if (id && !saved) throw new Error("Unknown session");
    const manager = saved
      ? SessionManager.open(saved.path, this.paths.sessions)
      : SessionManager.create(cwd, this.paths.sessions);
    const notices: string[] = [];
    let entry: Entry | undefined;
    const handle = await this.factory(
      this.config,
      this.paths,
      manager,
      (message) =>
        entry
          ? this.emit(entry, { type: "notice", message })
          : notices.push(message),
    );
    if (this.closing) {
      await handle.close();
      throw new Error("Gateway shutting down");
    }
    entry = {
      handle,
      peers: new Set(),
      seq: 0,
      running: false,
      text: "",
      notices,
      tools: new Map(),
      accepted: new Set(),
      dialogs: new Map(),
    };
    for (const item of manager.getEntries())
      if (
        item.type === "custom" &&
        item.customType === "dimcode.prompt" &&
        typeof item.data === "string"
      )
        entry.accepted.add(item.data);
    const current = entry;
    handle.session.subscribe((event) => {
      if (event.type === "message_update") {
        const update = event.assistantMessageEvent;
        if (update.type === "text_delta") {
          current.text += update.delta;
          this.emit(current, { type: "text_delta", delta: update.delta });
        }
        return;
      }
      if (event.type === "message_start" && event.message.role === "assistant")
        current.text = "";
      if (event.type === "message_end" && event.message.role === "assistant")
        current.text = "";
      if (
        event.type === "tool_execution_start" ||
        event.type === "tool_execution_update"
      )
        current.tools.set(event.toolCallId, event);
      if (event.type === "tool_execution_end")
        current.tools.delete(event.toolCallId);
      this.emit(current, event);
    });
    const baseUI = handle.session.extensionRunner.getUIContext();
    const interaction = this.interaction(current);
    await handle.session.bindExtensions({
      mode: "rpc",
      uiContext: {
        ...baseUI,
        select: (title, options, opts) =>
          interaction.prompt({
            type: "select",
            message: title,
            options: options.map((value) => ({ id: value, label: value })),
            signal: opts?.signal,
          }),
        confirm: async (title, message, opts) =>
          (await interaction.prompt({
            type: "select",
            message: title + "\n" + message,
            options: [
              { id: "yes", label: "Yes" },
              { id: "no", label: "No" },
            ],
            signal: opts?.signal,
          })) === "yes",
        input: (title, placeholder, opts) =>
          interaction.prompt({
            type: "text",
            message: title,
            placeholder,
            signal: opts?.signal,
          }),
        editor: (title, prefill) =>
          interaction.prompt({
            type: "text",
            message: title,
            placeholder: prefill,
          }),
        notify: (message) => this.emit(current, { type: "notice", message }),
        setStatus: (key, message) => {
          if (message)
            this.emit(current, {
              type: "notice",
              message: key + ": " + message,
            });
        },
        custom: async () => {
          throw new Error(
            "Executable UI factories cannot cross a socket; install the component in the dimcode terminal renderer.",
          );
        },
      },
      onError: (error) =>
        this.emit(current, { type: "notice", message: error.error }),
    });
    this.entries.set(handle.session.sessionId, current);
    if (saved)
      notices.push(
        "Resumed saved history. Interrupted external actions are not replayed; inspect DimOS status before continuing.",
      );
    for (const message of notices)
      handle.session.sessionManager.appendCustomEntry(
        "dimcode.notice",
        message,
      );
    return current;
  }
  private snapshot(entry: Entry, socket: Socket): Snapshot {
    const session = entry.handle.session;
    return {
      sessionId: session.sessionId,
      cwd: session.sessionManager.getCwd(),
      seq: entry.seq,
      busy: entry.running,
      writable: entry.owner === socket,
      messages: session.messages,
      text: entry.text,
      notices: entry.notices,
      tools: [...entry.tools.values()],
    };
  }
  private detach(socket: Socket): void {
    const entry = this.peers.get(socket);
    if (!entry) return;
    entry.peers.delete(socket);
    this.peers.delete(socket);
    if (entry.owner === socket) {
      entry.owner = undefined;
      for (const dialog of entry.dialogs.values())
        dialog.reject(new Error("Input terminal detached"));
      entry.dialogs.clear();
    }
  }
  private attach(entry: Entry, socket: Socket, writable: boolean): Snapshot {
    if (writable && entry.owner && entry.owner !== socket)
      throw new Error(
        "Session has an input owner; attach with --view or detach the other terminal.",
      );
    this.detach(socket);
    entry.peers.add(socket);
    this.peers.set(socket, entry);
    if (writable) entry.owner = socket;
    return this.snapshot(entry, socket);
  }
  private connect(socket: Socket): void {
    this.sockets.add(socket);
    socket.on("error", () => {});
    socket.on("close", () => {
      this.detach(socket);
      this.sockets.delete(socket);
    });
    readLines(socket, (value) => {
      const parsed = requestSchema.safeParse(value);
      if (!parsed.success) {
        socket.destroy();
        return;
      }
      const { id, command } = parsed.data;
      void this.dispatch(socket, command, id).then(
        (data) => send(socket, { type: "response", id, data }),
        (error) =>
          send(socket, {
            type: "response",
            id,
            error: error instanceof Error ? error.message : String(error),
          }),
      );
    });
  }
  private interaction(entry: Entry): AuthInteraction {
    return {
      prompt: (prompt: AuthPrompt) =>
        new Promise<string>((resolve, reject) => {
          if (!entry.owner) {
            reject(
              new Error("This interaction needs an attached input terminal"),
            );
            return;
          }
          if (entry.dialogs.size) {
            reject(new Error("Finish the existing terminal dialog first"));
            return;
          }
          if (prompt.signal?.aborted) {
            reject(new Error("Interaction cancelled"));
            return;
          }
          const promptId = randomUUID();
          const { signal, ...wirePrompt } = prompt;
          const aborted = () =>
            entry.dialogs
              .get(promptId)
              ?.reject(new Error("Interaction cancelled"));
          const cleanup = () => {
            entry.dialogs.delete(promptId);
            signal?.removeEventListener("abort", aborted);
          };
          entry.dialogs.set(promptId, {
            resolve: (value) => {
              cleanup();
              resolve(value);
            },
            reject: (error) => {
              cleanup();
              reject(error);
            },
          });
          signal?.addEventListener("abort", aborted, { once: true });
          send(entry.owner, {
            type: "event",
            seq: ++entry.seq,
            event: { type: "auth_prompt", promptId, prompt: wirePrompt },
          });
        }),
      notify: (info) => {
        if (entry.owner)
          send(entry.owner, {
            type: "event",
            seq: ++entry.seq,
            event: { type: "auth_info", info },
          });
      },
    };
  }
  private async dispatch(
    socket: Socket,
    command: Command,
    id: string,
  ): Promise<Results[Command["type"]]> {
    if (this.closing) throw new Error("Gateway shutting down");
    if (command.type === "shutdown") {
      setImmediate(() => void this.close());
      return null;
    }
    if (command.type === "new_session")
      return this.attach(
        await this.restore(undefined, command.cwd),
        socket,
        true,
      );
    if (command.type === "attach")
      return this.attach(
        await this.restore(command.sessionId),
        socket,
        command.writable,
      );
    if (command.type === "list_sessions") {
      const saved = await SessionManager.listAll(this.paths.sessions);
      const list = new Map(
        saved.map((item) => [
          item.id,
          { id: item.id, cwd: item.cwd, name: item.name ?? item.firstMessage },
        ]),
      );
      for (const [sessionId, entry] of this.entries)
        if (!list.has(sessionId))
          list.set(sessionId, {
            id: sessionId,
            cwd: entry.handle.session.sessionManager.getCwd(),
            name: "",
          });
      return [...list.values()];
    }
    const entry = this.peers.get(socket);
    if (!entry) throw new Error("Attach to a session first");
    const session = entry.handle.session;
    if (command.type === "get_state") return this.snapshot(entry, socket);
    if (command.type === "get_available_models")
      return session.modelRuntime.getModels().map((model) => ({
        provider: model.provider,
        id: model.id,
        name: model.name,
      }));
    if (entry.owner !== socket) throw new Error("This terminal is read-only");
    switch (command.type) {
      case "prompt": {
        if (entry.accepted.has(id)) return null;
        if (entry.running)
          throw new Error("Agent is busy; use steer or follow_up");
        entry.accepted.add(id);
        session.sessionManager.appendCustomEntry("dimcode.prompt", id);
        entry.running = true;
        // Acknowledge acceptance without tying turn lifetime to the connection.
        void session
          .prompt(command.message, { images: command.images })
          .then(() => {
            const last = session.messages.at(-1);
            if (last?.role === "assistant" && last.stopReason === "error")
              throw new Error(last.errorMessage ?? "Model request failed");
          })
          .catch((error) =>
            this.emit(entry, { type: "turn_error", message: String(error) }),
          )
          .finally(() => {
            entry.running = false;
            this.emit(entry, { type: "idle" });
          });
        return null;
      }
      case "steer":
        await session.steer(command.message);
        return null;
      case "follow_up":
        await session.followUp(command.message);
        return null;
      case "abort":
        await session.abort();
        this.emit(entry, {
          type: "notice",
          message:
            "Agent turn cancelled. Use the DimOS skill's stop operation to confirm any background robot action stopped.",
        });
        return null;
      case "ui_response": {
        const dialog = entry.dialogs.get(command.promptId);
        if (!dialog) throw new Error("Unknown or expired dialog");
        if (command.value === undefined) dialog.reject(new Error("Cancelled"));
        else dialog.resolve(command.value);
        return null;
      }
      case "login":
        if (command.provider === "anthropic" && command.authType === "oauth")
          throw new Error("Use an Anthropic API key for dimcode.");
        await session.modelRuntime.login(
          command.provider,
          command.authType,
          this.interaction(entry),
        );
        return null;
      case "logout":
        await session.modelRuntime.logout(command.provider);
        return null;
      case "set_model": {
        if (entry.running)
          throw new Error("Change models after the current turn");
        const model = session.modelRuntime.getModel(
          command.provider,
          command.modelId,
        );
        if (!model) throw new Error("Unknown provider/model");
        await session.setModel(model);
        return null;
      }
      case "reload":
        if (entry.running) throw new Error("Reload after the current turn");
        await session.reload();
        return null;
    }
  }
  async close(): Promise<void> {
    if (this.closing) return;
    this.closing = true;
    for (const socket of this.sockets) socket.destroy();
    await Promise.allSettled(
      [...this.entries.values()].map(async (entry) => {
        try {
          await entry.handle.session.abort();
        } finally {
          await entry.handle.close();
        }
      }),
    );
    this.entries.clear();
    await new Promise<void>((resolve) => this.server.close(() => resolve()));
    await unlink(this.paths.socket).catch(() => {});
  }
}
