import { readFile } from "node:fs/promises";
import { resolve } from "node:path";
import {
  Container,
  Markdown,
  Spacer,
  ProcessTerminal,
  Text,
  TuiMainScreen,
} from "@earendil-works/pi-tui";
import sharp from "sharp";
import {
  ToolExecutionComponent,
  AssistantMessageComponent,
  UserMessageComponent,
  getMarkdownTheme,
  initTheme,
  createBashToolDefinition,
  createReadToolDefinition,
  createEditToolDefinition,
  createWriteToolDefinition,
} from "@earendil-works/pi-coding-agent";
import { z } from "zod";
import { Connection, type Event, type Snapshot } from "./protocol.js";
import { imageComponent, renderDetails } from "./render.js";
import { MediaPool, type MediaLease } from "./media.js";
import { ChatInput } from "./input.js";
import type { Slot } from "@dimos/sdk";

const resultSchema = z.object({
  content: z
    .array(
      z.union([
        z.object({ type: z.literal("text"), text: z.string() }),
        z.object({
          type: z.literal("image"),
          data: z.string(),
          mimeType: z.string(),
        }),
      ]),
    )
    .default([]),
  details: z.unknown().optional(),
});
export async function terminal(
  socket: string,
  options: { sessionId?: string; cwd?: string; view?: boolean } = {},
): Promise<void> {
  initTheme("dark", false);
  const client = new Connection(socket),
    pool = new MediaPool();
  const tui = new TuiMainScreen(new ProcessTerminal()),
    transcript = new Container();
  const status = new Text("Connecting…", 0, 0),
    input = new ChatInput();
  const cards = new Map<
    string,
    {
      block: Container;
      component: ToolExecutionComponent;
      output: unknown;
      expanded: boolean;
      lease?: MediaLease<Slot>;
      close?: () => void;
    }
  >();
  let current: Snapshot,
    assistant = new Container(),
    text = "",
    stopped = false;
  let auth: Extract<Event, { type: "auth_prompt" }> | undefined;
  let restoring = true;
  const pending: Array<{ sessionId: string; seq: number; event: Event }> = [];
  const add = (value: string) => transcript.addChild(new Text(value, 0, 0));
  const setAssistant = (value: string) => {
    assistant.clear();
    assistant.addChild(new Markdown(value, 1, 1, getMarkdownTheme()));
  };
  const ready = () =>
    "  " +
    (current.writable ? "Ready" : "Read-only") +
    " · " +
    current.sessionId.slice(0, 8);
  const notice = (error: unknown) => {
    add(String(error));
    tui.requestRender();
  };
  const closeCards = () => {
    for (const card of cards.values()) card.close?.();
    cards.clear();
  };
  const draw = (
    id: string,
    output: unknown,
    toolName = "tool",
    args: unknown = {},
    partial = false,
    isError = false,
  ) => {
    let card = cards.get(id);
    if (!card) {
      const cwd = current?.cwd ?? options.cwd ?? process.cwd();
      const definitions = [
        createBashToolDefinition(cwd),
        createReadToolDefinition(cwd),
        createEditToolDefinition(cwd),
        createWriteToolDefinition(cwd),
      ];
      const component = new ToolExecutionComponent(
        toolName,
        id,
        args,
        { showImages: true },
        definitions.find((tool) => tool.name === toolName),
        tui,
        cwd,
      );
      component.markExecutionStarted();
      component.setArgsComplete();
      card = { block: new Container(), component, output, expanded: false };
      cards.set(id, card);
      transcript.addChild(card.block);
    }
    card.close?.();
    card.close = undefined;
    card.output = output;
    card.block.clear();
    card.block.addChild(card.component);
    const result = resultSchema.safeParse(output);
    if (!result.success) {
      card.block.addChild(new Text("Running…", 0, 0));
      return;
    }
    const details = renderDetails.safeParse(result.data.details);
    card.component.updateResult({ ...result.data, isError }, partial);
    card.component.setExpanded(card.expanded);
    if (details.success && details.data.live && !card.close) {
      const live = details.data.live,
        held = card;
      const preview = new Container();
      held.block.addChild(preview);
      let disposed = false,
        drawing = false,
        changed = true,
        previous = -1;
      const tick = async () => {
        if (disposed || drawing || !changed) return;
        const slot = held.lease?.current();
        if (
          !slot ||
          slot.version === previous ||
          !(slot.value instanceof Uint8Array)
        )
          return;
        drawing = true;
        changed = false;
        try {
          const bytes = await sharp(slot.value)
            .resize({ width: 960, height: 720, fit: "inside" })
            .png()
            .toBuffer();
          if (disposed) return;
          previous = slot.version;
          preview.clear();
          preview.addChild(
            imageComponent(
              bytes.toString("base64"),
              "image/png",
              "LIVE · " + live.channel + " · seq " + slot.seq,
            ),
          );
          tui.requestRender();
        } catch (error) {
          if (!disposed) notice(error);
        } finally {
          drawing = false;
        }
      };
      const timer = setInterval(() => void tick(), 100);
      held.close = () => {
        disposed = true;
        clearInterval(timer);
        held.lease?.close();
      };
      void pool
        .acquire(
          live,
          live.channel,
          (slot) => slot,
          () => {
            changed = true;
          },
        )
        .then((lease) => {
          if (disposed) lease.close();
          else held.lease = lease;
        })
        .catch(notice);
    }
  };
  const apply = (event: Event) => {
    if (event.type === "auth_prompt") {
      auth = event;
      input.secret =
        event.prompt.type === "secret" || event.prompt.type === "manual_code";
      input.setValue("");
      status.setText(
        event.prompt.message +
          (event.prompt.type === "select"
            ? "\n" +
              event.prompt.options
                .map((option) => option.id + ": " + option.label)
                .join(" · ")
            : ""),
      );
    } else if (event.type === "auth_info") {
      const info = event.info;
      status.setText(
        info.type === "auth_url"
          ? info.url + "\n" + (info.instructions ?? "")
          : info.type === "device_code"
            ? info.verificationUri + " · " + info.userCode
            : info.message,
      );
    } else if (event.type === "text_delta") {
      text += event.delta;
      setAssistant(text);
    } else if (
      event.type === "message_end" &&
      event.message.role === "assistant"
    ) {
      text = event.message.content
        .filter((item) => item.type === "text")
        .map((item) => item.text)
        .join("\n");
      assistant.clear();
      assistant.addChild(new AssistantMessageComponent(event.message, true));
    } else if (event.type === "message_start") {
      if (event.message.role === "assistant") {
        text = "";
        assistant = new Container();
        transcript.addChild(assistant);
      } else if (event.message.role === "user") {
        const content = event.message.content;
        transcript.addChild(
          new UserMessageComponent(
            typeof content === "string"
              ? content
              : content
                  .filter((item) => item.type === "text")
                  .map((item) => item.text)
                  .join("\n"),
          ),
        );
      }
    } else if (event.type === "tool_execution_start") {
      draw(
        event.toolCallId,
        { content: [{ type: "text", text: "Running…" }] },
        event.toolName,
        event.args,
        true,
      );
    } else if (event.type === "tool_execution_update")
      draw(
        event.toolCallId,
        event.partialResult,
        event.toolName,
        event.args,
        true,
      );
    else if (event.type === "tool_execution_end")
      draw(
        event.toolCallId,
        event.result,
        event.toolName,
        {},
        false,
        event.isError,
      );
    else if (event.type === "notice" || event.type === "turn_error")
      notice(event.message);
    else if (event.type === "idle") status.setText(ready());
    tui.requestRender();
  };
  client.onEvent = (seq, event, sessionId) => {
    if (restoring) pending.push({ sessionId, seq, event });
    else if (sessionId === current.sessionId) apply(event);
  };
  const restore = (snapshot: Snapshot) => {
    closeCards();
    transcript.clear();
    current = snapshot;
    add("\x1b[1;36m  dimcode\x1b[0m  ·  Dimensional agent");
    add("\x1b[2m  " + current.cwd + "\x1b[0m");
    if (!current.messages.length) {
      transcript.addChild(new Spacer(1));
      add("  Build apps. Run blueprints. Explore sensor memory.");
      add(
        "\x1b[2m  Try: inspect this workspace, or show available DimOS blueprints.\x1b[0m",
      );
      transcript.addChild(new Spacer(1));
    }
    const argumentsByCall = new Map<string, unknown>();
    for (const message of current.messages) {
      if (message.role === "assistant")
        for (const block of message.content) {
          if (block.type === "toolCall")
            argumentsByCall.set(block.id, block.arguments);
        }
      if ("content" in message) {
        if (message.role === "toolResult")
          draw(
            message.toolCallId,
            message,
            message.toolName,
            argumentsByCall.get(message.toolCallId),
            false,
            message.isError,
          );
        else {
          const content =
            typeof message.content === "string"
              ? message.content
              : message.content
                  .filter((item) => item.type === "text")
                  .map((item) => item.text)
                  .join("\n");
          if (message.role === "assistant")
            transcript.addChild(new AssistantMessageComponent(message, true));
          else if (message.role === "user")
            transcript.addChild(new UserMessageComponent(content));
          else if (content) add(content);
        }
      }
    }
    for (const message of current.notices) add(message);
    text = current.text;
    assistant = new Container();
    if (text) setAssistant(text);
    transcript.addChild(assistant);
    for (const event of current.tools) apply(event);
    status.setText(current.busy ? "  Running · Esc to cancel" : ready());
    restoring = false;
    for (const packet of pending.splice(0))
      if (packet.sessionId === current.sessionId && packet.seq > current.seq)
        apply(packet.event);
    tui.requestRender();
  };
  const stop = () => {
    if (stopped) return;
    stopped = true;
    closeCards();
    pool.close();
    tui.stop();
    client.close();
  };
  try {
    restore(
      await client.call(
        options.sessionId
          ? {
              type: "attach",
              sessionId: options.sessionId,
              writable: !options.view,
            }
          : { type: "new_session", cwd: options.cwd },
      ),
    );
    tui.addChild(transcript);
    tui.addChild(new Spacer(1));
    tui.addChild(status);
    tui.addChild(input);
    tui.addChild(
      new Text(
        "\x1b[2m  /help commands · /models choose model · Esc cancel · Ctrl-C detach\x1b[0m",
        0,
        0,
      ),
    );
    tui.setFocus(input);
    input.onSubmit = (value) => {
      input.setValue("");
      if (auth) {
        const promptId = auth.promptId;
        auth = undefined;
        input.secret = false;
        void client
          .call({ type: "ui_response", promptId, value })
          .catch(notice);
        return;
      }
      void (async () => {
        if (value === "/exit") {
          stop();
          return;
        }
        if (value === "/help") {
          add(
            "/new · /sessions · /resume ID · /model PROVIDER MODEL · /models · /login PROVIDER [oauth] · /logout PROVIDER · /abort · /steer TEXT · /follow TEXT · /reload · /image PATH · /expand · /exit",
          );
          return;
        }
        if (value === "/sessions") {
          notice(
            await client
              .call({ type: "list_sessions" })
              .then((list) =>
                list.map((item) => item.id + " " + item.name).join("\n"),
              ),
          );
          return;
        }
        if (value === "/new" || value.startsWith("/resume ")) {
          restoring = true;
          try {
            restore(
              await client.call(
                value === "/new"
                  ? { type: "new_session", cwd: current.cwd }
                  : {
                      type: "attach",
                      sessionId: value.slice(8).trim(),
                      writable: true,
                    },
              ),
            );
          } catch (error) {
            restoring = false;
            throw error;
          }
          return;
        }
        if (value === "/expand") {
          for (const card of cards.values()) {
            card.expanded = !card.expanded;
            card.component.setExpanded(card.expanded);
          }
          return;
        }
        if (value === "/abort") {
          await client.call({ type: "abort" });
          return;
        }
        if (value === "/reload") {
          await client.call({ type: "reload" });
          return;
        }
        if (value === "/models") {
          notice(
            (await client.call({ type: "get_available_models" }))
              .map((model) => model.provider + " " + model.id)
              .join("\n"),
          );
          return;
        }
        const [command, provider, option] = value.split(/\s+/);
        if (command === "/login") {
          await client.call({
            type: "login",
            provider,
            authType: option === "oauth" ? "oauth" : "api_key",
          });
          status.setText("Login complete");
          return;
        }
        if (command === "/logout") {
          await client.call({ type: "logout", provider });
          return;
        }
        if (command === "/model") {
          await client.call({ type: "set_model", provider, modelId: option });
          return;
        }
        if (command === "/steer" || command === "/follow") {
          await client.call({
            type: command === "/steer" ? "steer" : "follow_up",
            message: value.slice(command.length + 1),
          });
          return;
        }
        if (command === "/image") {
          const bytes = await sharp(
            await readFile(resolve(current.cwd, value.slice(7))),
          )
            .resize({ width: 1280, height: 960, fit: "inside" })
            .png()
            .toBuffer();
          await client.call({
            type: "prompt",
            message: "Inspect this attached image.",
            images: [
              {
                type: "image",
                mimeType: "image/png",
                data: bytes.toString("base64"),
              },
            ],
          });
          return;
        }
        if (value.trim()) {
          status.setText("  Running · Esc to cancel");
          await client.call({ type: "prompt", message: value });
        }
      })()
        .catch(notice)
        .finally(() => tui.requestRender());
    };
    input.onEscape = () => {
      if (auth) {
        const promptId = auth.promptId;
        auth = undefined;
        input.secret = false;
        input.setValue("");
        void client.call({ type: "ui_response", promptId }).catch(notice);
      } else void client.call({ type: "abort" }).catch(notice);
    };
    tui.addInputListener((data) => {
      if (data === "\x03") {
        stop();
        return { consume: true };
      }
      return undefined;
    });
    tui.start();
    await new Promise<void>((resolve) => {
      client.onClose = () => {
        stop();
        resolve();
      };
    });
  } finally {
    stop();
  }
}
