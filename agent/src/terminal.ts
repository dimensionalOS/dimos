import { readFile } from "node:fs/promises";
import { basename, resolve } from "node:path";
import {
  Container,
  Markdown,
  Spacer,
  ProcessTerminal,
  Text,
  TuiAltScreen,
  ScrollView,
  VStack,
  matchesKey,
  type Component,
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
import { PromptEditor, commands } from "./input.js";
import { SpatialView, readCloud } from "./spatial.js";
import { ResultImages } from "./tool-images.js";
import { accent, clean, muted, StatusLine } from "./terminal-style.js";
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
  const tui = new TuiAltScreen(new ProcessTerminal()),
    transcript = new Container();
  const status = new Text("Connecting…", 0, 0),
    input = new PromptEditor(tui);
  let lastSpatial: SpatialView | undefined;
  let lastImages: ResultImages | undefined;
  let inspectorOpen = false;
  let layout: VStack;
  const localViews = new Set<SpatialView>();
  const inspect = (view = lastSpatial) => {
    if (!view) {
      notice(
        "No point cloud yet. Use /inspect PATH or ask the agent to render one.",
      );
      return;
    }
    inspectorOpen = true;
    view.setExpanded(true);
    const expanded = {
      render: (width) => view.render(width),
      invalidate: () => view.invalidate(),
      handleInput: (data) => {
        if (matchesKey(data, "escape")) {
          view.setExpanded(false);
          inspectorOpen = false;
          tui.setLayoutRoot(layout);
          tui.setFocus(input);
        } else view.handleInput(data);
      },
    } satisfies Component;
    tui.setLayoutRoot(
      new VStack([
        new StatusLine(() => [
          accent("⠿ dimcode / spatial inspector"),
          muted("Esc returns to chat"),
        ]),
        {
          component: new ScrollView(expanded, { primary: true }),
          basis: 0,
          grow: 1,
        },
      ]),
    );
    tui.setFocus(expanded);
  };
  const cards = new Map<
    string,
    {
      block: Container;
      component: ToolExecutionComponent;
      output: unknown;
      expanded: boolean;
      toolName: string;
      args: unknown;
      spatial?: SpatialView;
      images?: ResultImages;
      loading?: boolean;
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
    "  " + (current.writable ? "Ready" : "Read-only") + " · /help commands";
  const notice = (error: unknown) => {
    add(String(error));
    tui.requestRender();
  };
  const closeCards = () => {
    for (const card of cards.values()) {
      card.close?.();
      card.spatial?.close();
      card.images?.close();
    }
    for (const view of localViews) view.close();
    localViews.clear();
    lastSpatial = undefined;
    lastImages = undefined;
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
      card = {
        block: new Container(),
        component,
        output,
        expanded: false,
        toolName,
        args,
      };
      cards.set(id, card);
      transcript.addChild(card.block);
    }
    card.close?.();
    card.close = undefined;
    card.output = output;
    card.block.clear();
    const result = resultSchema.safeParse(output);
    const details = renderDetails.safeParse(
      result.success ? result.data.details : undefined,
    );
    const named = z
      .object({ tool: z.string() })
      .safeParse(result.success ? result.data.details : undefined);
    const title =
      (details.success && details.data.title) ||
      (named.success && named.data.tool) ||
      card.toolName;
    const builtin = ["bash", "read", "edit", "write"].includes(card.toolName);
    if (builtin || card.expanded) card.block.addChild(card.component);
    else {
      card.block.addChild(new Spacer(1));
      card.block.addChild(
        new Text(
          accent(" ↳ ") +
            clean(title) +
            muted(isError ? " · failed" : partial ? " · running" : " · done"),
          0,
          0,
        ),
      );
    }
    if (!result.success) {
      card.block.addChild(new Text("Running…", 0, 0));
      return;
    }
    card.component.updateResult({ ...result.data, isError }, partial);
    card.component.setExpanded(card.expanded);
    const images = result.data.content.filter((item) => item.type === "image");
    if (
      images.length &&
      !card.spatial &&
      (!card.images ||
        images.length !== card.images.images.length ||
        images.some((image, i) => image.data !== card.images?.images[i].data))
    ) {
      card.images?.close();
      card.images = new ResultImages(
        images.map((image, i) => ({
          ...image,
          label:
            (details.success && details.data.views?.[i].label) ||
            `Image ${i + 1}`,
        })),
        () => tui.requestRender(),
      );
      lastImages = card.images;
    }
    if (!builtin && !card.expanded) {
      const summary = details.success
        ? details.data.summary.split(";")[0]
        : result.data.content
            .filter((item) => item.type === "text")
            .map((item) => item.text)
            .join("\n");
      card.block.addChild(
        new Text(muted(" " + clean(summary.slice(0, 220))), 0, 0),
      );
      if (card.spatial) card.block.addChild(card.spatial);
      else if (card.images) card.block.addChild(card.images);
    }
    const pointArgs = z
      .object({ kind: z.literal("points") })
      .safeParse(card.args);
    if (
      details.success &&
      pointArgs.success &&
      !partial &&
      !isError &&
      !card.spatial &&
      !card.loading
    ) {
      card.loading = true;
      const held = card;
      void readCloud(details.data.source, details.data.sha256)
        .then(({ cloud, sha256 }) => {
          if (stopped || cards.get(id) !== held) return;
          held.spatial = new SpatialView(
            cloud,
            details.data.source,
            sha256,
            () => tui.requestRender(),
          );
          lastSpatial = held.spatial;
          draw(id, held.output, held.toolName, held.args);
        })
        .catch((error) => {
          if (!stopped && cards.get(id) === held) notice(error);
        });
    }
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
    else if (event.type === "idle") {
      current.busy = false;
      status.setText(ready());
    }
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
    input.setWorkspace(current.cwd);
    if (!current.messages.length) {
      transcript.addChild(new Spacer(1));
      add(accent("  ⠿ Build apps. Run blueprints. Explore sensor memory."));
      add(
        muted(
          "  Ask about this workspace, or show available DimOS blueprints.",
        ),
      );
      add(
        muted(
          "  Memory results appear inside tool cards · /panel selects a view",
        ),
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
    layout = new VStack([
      new Spacer(1),
      new StatusLine(() => [
        accent("⠿ dimcode") + "   " + clean(basename(current.cwd)),
        muted(current.writable ? "GATEWAY ATTACHED" : "VIEW ONLY"),
      ]),
      new StatusLine(() => [muted(clean(current.cwd)), ""]),
      new Spacer(1),
      {
        component: new ScrollView(transcript, {
          follow: "end",
          primary: true,
        }),
        basis: 0,
        grow: 1,
        minSize: 1,
      },
      status,
      input,
      new StatusLine(() => [
        current.model
          ? clean(current.model.provider + " / " + current.model.id)
          : muted("/models choose model"),
        muted("Shift+Enter newline · Ctrl-C detach"),
      ]),
    ]);
    tui.setLayoutRoot(layout);
    tui.setFocus(input);
    input.onSubmit = (value) => {
      if (!auth && value.trim()) input.addToHistory(value);
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
          add(commands.map((command) => "/" + command).join(" · "));
          add("/resume ID · /model PROVIDER MODEL · /login PROVIDER [oauth]");
          add(
            "/panel N: result view (0 overview) · /inspect PATH: saved XYZ · /view: rotate/zoom · /expand: latest tool details",
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
          const card = [...cards.entries()].at(-1);
          if (card) {
            const [id, tool] = card;
            tool.expanded = !tool.expanded;
            draw(id, tool.output, tool.toolName, tool.args);
          }
          return;
        }
        if (value === "/view") {
          inspect();
          return;
        }
        if (value.startsWith("/panel ")) {
          const index = Number(value.slice(7).trim());
          if (!lastImages)
            throw new Error(
              "No image result yet. Ask the agent to display the memory query's existing exports.",
            );
          if (
            !Number.isInteger(index) ||
            index < 0 ||
            index > lastImages.images.length
          )
            throw new Error(`Choose /panel 0–${lastImages.images.length}.`);
          lastImages.select(index);
          return;
        }
        if (value.startsWith("/inspect ")) {
          const source = resolve(current.cwd, value.slice(9).trim());
          const { cloud, sha256 } = await readCloud(source);
          const view = new SpatialView(cloud, source, sha256, () =>
            tui.requestRender(),
          );
          lastSpatial = view;
          localViews.add(view);
          transcript.addChild(view);
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
          current.model = { provider, id: option };
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
          current.busy = true;
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
      if (inspectorOpen && data !== "\x03") return undefined;
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
