import { owner } from "../../sdk/failures.ts";
import { retryByRequestId } from "../../sdk/retry.ts";
import type { Ref } from "@earendil-works/chord";
import { ConversationContract } from "../../shared/protocol/conversation/contract.ts";
import type { AgentModuleName } from "../../shared/protocol/runtime/event.ts";
import type { ConversationId, JsonObject } from "@earendil-works/pi-durable";
import type { EpochMs } from "../../shared/protocol/time.ts";
import type { FileRenderer } from "../../shared/render/render.ts";
import type { ImageContent, ModelThinkingLevel } from "@earendil-works/pi-ai";
import type { JobAction, JobName } from "../../shared/protocol/jobs/state.ts";
import type { Role } from "../../sdk/state.ts";
import type { TUI } from "@earendil-works/pi-tui";
import type { TimelineRow } from "../../shared/view-models/conversation/timeline.ts";
import type { TranscriptView } from "./conversation/transcript.ts";
import type { Ui } from "./session.ts";
import { ProgressPanel } from "./conversation/progress.ts";
import { muted } from "./style.ts";
import { backgroundTarget, runningCalls } from "../../shared/view-models/conversation/tools.ts";
import { basename, resolve } from "node:path";
import { clean } from "../../shared/view-models/text.ts";
import { editorTextOf } from "../../shared/view-models/conversation/queue.ts";
import { initTheme } from "@earendil-works/pi-coding-agent";
import { inspectPath } from "./inspect.ts";
import { isRunning } from "../../shared/view-models/conversation/timeline.ts";
import { newActionRequest, newUserRequest } from "../../shared/protocol/actions/request.ts";

type Shown = { readonly conversation: Ref<typeof ConversationContract>; readonly id: ConversationId };

/** What slash commands act on; main.ts provides it. */
export type TerminalApp = {
  readonly ui: Ui;
  readonly tui: TUI;
  readonly transcript: TranscriptView;
  readonly renderer: FileRenderer;
  /** The shown conversation and its number, or null before one is attached. */
  shown(): Shown | null;
  /** The shown conversation's timeline, as the chat draws it. */
  rows(): readonly TimelineRow[];
  /** The shown conversation's directory. */
  cwd(): string;
  /** Shows another conversation, or a new one when null; the current one is released first. */
  switchTo(conversation: ConversationId | null, role: Role): Promise<void>;
  /** Puts text withdrawn from `from` into the editor after what is typed there, or holds it until `from` shows. */
  restore(from: ConversationId, text: string): void;
  /** Shows a component above the editor while it works, or nothing. */
  showProgress(panel: ProgressPanel | null): void;
  /** Full-screen `/menu`. */
  openMenu(): void;
  /** The conversation switcher overlay, as Ctrl+O opens it. */
  openSwitcher(): void;
  /** Selects view `index` of the newest rendering and gives it the keys when it takes any. */
  openPanel(index: number): void;
  exit(code: number): void;
};

export type SlashCommand = {
  readonly name: string;
  readonly usage: string;
  run(args: string, app: TerminalApp): Promise<void>;
};

/** pi-ai's thinking levels, typed by its union so a level it adds or drops fails to compile here. */
const THINKING: { readonly [L in ModelThinkingLevel]: L } = {
  off: "off",
  minimal: "minimal",
  low: "low",
  medium: "medium",
  high: "high",
  xhigh: "xhigh",
  max: "max",
};

function thinkingLevel(word: string | undefined): ModelThinkingLevel | null {
  if (word === undefined) return null;
  if (!Object.hasOwn(THINKING, word)) throw new Error(`Thinking is one of ${Object.keys(THINKING).join(", ")}.`);
  return THINKING[word as ModelThinkingLevel];
}

function conversationNumber(text: string, usage: string): ConversationId {
  const id = Number(text.trim().replace(/^#/, ""));
  if (!Number.isSafeInteger(id) || id < 1) throw new Error(`Usage: ${usage}`);
  return id as ConversationId;
}

function shownOf(app: TerminalApp): Shown {
  const shown = app.shown();
  if (shown === null) throw new Error("No conversation is shown; /new or /resume ID first.");
  return shown;
}

/** The first `count` words of `text`, a "quoted phrase" counting as one, and what follows them. */
function take(text: string, count: number): { readonly words: readonly string[]; readonly rest: string } {
  const words: string[] = [];
  let rest = text;
  while (words.length < count) {
    rest = rest.trimStart();
    if (rest === "") break;
    if (rest.startsWith('"')) {
      const end = rest.indexOf('"', 1);
      if (end < 0) throw new Error(`Unclosed quote in ${rest}`);
      words.push(rest.slice(1, end));
      rest = rest.slice(end + 1);
      continue;
    }
    const space = rest.search(/\s/);
    words.push(space < 0 ? rest : rest.slice(0, space));
    rest = space < 0 ? "" : rest.slice(space);
  }
  return { words, rest: rest.trim() };
}

function jsonObject(text: string): JsonObject {
  const parsed: unknown = JSON.parse(text);
  if (typeof parsed !== "object" || parsed === null || Array.isArray(parsed))
    throw new Error('Skill arguments are a JSON object, such as {} or {"x": 1}.');
  return parsed as JsonObject;
}

const MODULE_OPS = { restart: "restartModule", unload: "unloadModule", delete: "deleteModule" } as const;

const JOB_USAGE = '/job create|update NAME "CRON" notify TEXT | NAME "CRON" skill SKILL [JSON]';

function jobAction(kind: string | undefined, rest: string): JobAction {
  if (kind === "notify" && rest !== "") return { kind: "notify", text: rest };
  if (kind === "skill") {
    const { words, rest: json } = take(rest, 1);
    const skill = words[0];
    if (skill === undefined) throw new Error(`Usage: ${JOB_USAGE}`);
    return { kind: "skill", skill, arguments: jsonObject(json === "" ? "{}" : json) };
  }
  throw new Error(`Usage: ${JOB_USAGE}`);
}

/** Every key the chat answers to besides the editor's own, as `/help` lists them. */
const KEYS: readonly string[] = [
  "Ctrl+T  show or hide the stream sidebar",
  "Shift+Tab  focus the stream sidebar or return to chat",
  "Sidebar: Up/Down select a stream; Left/Right fold or show it; Esc returns to chat",
  "Ctrl+O  switch conversations",
  "Ctrl+B  send the running tool call to the background",
  "Esc     while running, stop the run; with an empty editor, select a message (↑/↓, then e edit, u summarize up to, f summarize from)",
  "Up      with an empty editor, bring queued user messages back for editing",
  "Ctrl-C  clear the editor; with an empty editor, detach (the conversation keeps running)",
];

/** Sends typed text: a steer while a run is in progress, so it joins the run; otherwise placed at once. */
async function send(
  app: TerminalApp,
  { conversation, id }: Shown,
  text: string,
  images: readonly ImageContent[],
  followUp: boolean,
): Promise<void> {
  const whenBusy: "followUp" | "steer" = followUp || !isRunning(conversation.live.value) ? "followUp" : "steer";
  const args = { request: newUserRequest(), text, images: [...images], whenBusy };
  await app.ui.actions.observe(
    whenBusy === "steer" ? "send" : "queue a follow-up",
    retryByRequestId(app.ui, args, (args) => owner(conversation).send(args)),
    { action: "conversation.send", conversation: id, sending: { request: args.request, text } },
  );
}

export function slashCommands(): readonly SlashCommand[] {
  let expanded = false;
  const commands: SlashCommand[] = [
    {
      name: "help",
      usage: "/help",
      run: async (_args, app) =>
        app.transcript.note(
          [...commands.map((command) => command.usage), "", "Keys", ...KEYS].map((text) => muted(text)).join("\n"),
        ),
    },
    { name: "exit", usage: "/exit  detach; the conversation keeps running", run: async (_args, app) => app.exit(0) },
    { name: "new", usage: "/new  start a conversation", run: async (_args, app) => app.switchTo(null, "owner") },
    {
      name: "resume",
      usage: "/resume ID  open a conversation for input",
      run: async (args, app) => app.switchTo(conversationNumber(args, "/resume ID"), "owner"),
    },
    {
      name: "view",
      usage: "/view ID  show a conversation read-only",
      run: async (args, app) => app.switchTo(conversationNumber(args, "/view ID"), "viewer"),
    },
    {
      name: "conversations",
      usage: "/conversations  the switcher: every conversation, forks under their source (Ctrl+O)",
      run: async (_args, app) => app.openSwitcher(),
    },
    {
      name: "abort",
      usage: "/abort  stop the current run; queued user messages come back (Esc)",
      run: async (_args, app) => {
        const { conversation, id } = shownOf(app);
        const back = await app.ui.actions.observe(
          "stop the run",
          retryByRequestId(app.ui, { request: newActionRequest() }, (args) => owner(conversation).abort(args)),
        );
        if (back.ok && back.value.length > 0) app.restore(id, editorTextOf(back.value));
      },
    },
    {
      name: "follow",
      usage: "/follow TEXT  queue TEXT for after the current run (Enter steers it)",
      run: async (args, app) => {
        if (args.trim() === "") throw new Error("Usage: /follow TEXT");
        await send(app, shownOf(app), args, [], true);
      },
    },
    {
      name: "background",
      usage: "/background  move the newest running call that allows it to the background (Ctrl+B)",
      run: async (_args, app) => {
        const { conversation } = shownOf(app);
        const target = backgroundTarget(runningCalls(app.rows(), Date.now() as EpochMs));
        const call = conversation.detachable.value.find((candidate) => candidate.call === target?.call.id);
        if (call === undefined) throw new Error("No running call can move to the background.");
        await app.ui.actions.observe(
          `send call ${call.call} to the background`,
          owner(conversation).background({ call: call.call }),
        );
      },
    },
    {
      name: "model",
      usage: "/model PROVIDER MODEL [THINKING]  this conversation's model",
      run: async (args, app) => {
        const [provider = "", modelId = "", level] = args.trim().split(/\s+/);
        if (provider === "" || modelId === "") throw new Error("Usage: /model PROVIDER MODEL [THINKING]");
        await app.ui.actions.observe(
          `switch to ${provider}/${modelId}`,
          owner(shownOf(app).conversation).configure({
            model: { provider, modelId },
            thinkingLevel: thinkingLevel(level),
          }),
        );
      },
    },
    {
      name: "default-model",
      usage: "/default-model PROVIDER MODEL [THINKING]  the model new conversations start on",
      run: async (args, app) => {
        const [provider = "", modelId = "", level] = args.trim().split(/\s+/);
        if (provider === "" || modelId === "") throw new Error("Usage: /default-model PROVIDER MODEL [THINKING]");
        await app.ui.actions.observe(
          "change settings",
          owner(app.ui.gateway).updateSettings({
            model: { provider, modelId },
            thinkingLevel: thinkingLevel(level),
          }),
        );
      },
    },
    {
      name: "models",
      usage: "/models [PROVIDER]  models you can use now",
      run: async (args, app) => {
        const wanted = args.trim();
        const models = app.ui.gateway.accounts.value.models.filter(
          (model) => model.available && (wanted === "" || model.provider === wanted),
        );
        app.transcript.note(
          models.map((model) => `${model.provider} ${model.id}`).join("\n") ||
            muted("No models available; /login PROVIDER first."),
        );
      },
    },
    {
      name: "login",
      usage: "/login [PROVIDER [oauth]]  without PROVIDER, lists them",
      run: async (args, app) => {
        const [provider = "", method] = args.trim().split(/\s+/);
        if (provider === "") {
          app.transcript.note(
            app.ui.gateway.accounts.value.providers
              .map(
                (choice) =>
                  `${choice.provider} ${choice.method}${choice.status === null ? "" : muted(" · configured")} ${muted(clean(choice.name))}`,
              )
              .join("\n"),
          );
          return;
        }
        await app.ui.actions.observe(
          `log in to ${provider}`,
          owner(app.ui.gateway).login({ provider, method: method === "oauth" ? "oauth" : "api_key" }),
        );
      },
    },
    {
      name: "logout",
      usage: "/logout PROVIDER",
      run: async (args, app) => {
        const provider = args.trim();
        if (provider === "") throw new Error("Usage: /logout PROVIDER");
        await app.ui.actions.observe(`log out of ${provider}`, owner(app.ui.gateway).logout({ provider }));
      },
    },
    {
      name: "theme",
      usage: "/theme NAME  this terminal's colours, kept for the next start",
      run: async (args, app) => {
        const name = args.trim();
        if (name === "") throw new Error("Usage: /theme NAME, such as dark or light");
        const changed = await app.ui.actions.observe(
          "change settings",
          owner(app.ui.gateway).updateSettings({ theme: name }),
        );
        if (!changed.ok) return;
        initTheme(name, false);
        app.tui.invalidate();
      },
    },
    {
      name: "workspace",
      usage: "/workspace [DIR]  where new conversations start; without DIR, the DimOS checkout",
      run: async (args, app) => {
        const dir = args.trim();
        await app.ui.actions.observe(
          "change settings",
          owner(app.ui.gateway).updateSettings({ workspace: dir === "" ? null : resolve(app.cwd(), dir) }),
        );
      },
    },
    { name: "menu", usage: "/menu  the main menu", run: async (_args, app) => app.openMenu() },
    {
      name: "module",
      usage: "/module create NAME PATH | restart NAME | unload NAME | delete NAME",
      run: async (args, app) => {
        const [op = "", name = "", path = ""] = args.trim().split(/\s+/);
        const module = name as AgentModuleName;
        switch (op) {
          case "create":
            if (name === "" || path === "") throw new Error("Usage: /module create NAME PATH");
            await app.ui.actions.observe(
              `create module ${name}`,
              owner(app.ui.gateway).createModule({
                name: module,
                path: resolve(app.cwd(), path),
                owner: app.shown()?.id ?? null,
              }),
            );
            return;
          case "restart":
          case "unload":
          case "delete":
            if (name === "") throw new Error(`Usage: /module ${op} NAME`);
            await app.ui.actions.observe(`${op} ${name}`, owner(app.ui.gateway)[MODULE_OPS[op]]({ name: module }));
            return;
          default:
            throw new Error("Usage: /module create NAME PATH | restart NAME | unload NAME | delete NAME");
        }
      },
    },
    {
      name: "job",
      usage: `${JOB_USAGE} | delete NAME | run NAME`,
      run: async (args, app) => {
        const { words, rest } = take(args, 2);
        const [op = "", name = ""] = words;
        const job = name as JobName;
        switch (op) {
          case "create":
          case "update": {
            const { words: spec, rest: actionRest } = take(rest, 2);
            const [cron, kind] = spec;
            if (name === "" || cron === undefined) throw new Error(`Usage: ${JOB_USAGE}`);
            await app.ui.actions.observe(
              `${op} job ${name}`,
              owner(app.ui.gateway)[op === "create" ? "createJob" : "updateJob"]({
                name: job,
                conversation: shownOf(app).id,
                cron,
                timeZone: null,
                action: jobAction(kind, actionRest),
              }),
            );
            return;
          }
          case "delete":
            if (name === "") throw new Error("Usage: /job delete NAME");
            await app.ui.actions.observe(`delete job ${name}`, owner(app.ui.gateway).deleteJob({ name: job }));
            return;
          case "run":
            if (name === "") throw new Error("Usage: /job run NAME");
            await app.ui.actions.observe(
              `run job ${name} now`,
              retryByRequestId(app.ui, { name: job, request: newActionRequest() }, (args) =>
                owner(app.ui.gateway).runJob(args),
              ),
            );
            return;
          default:
            throw new Error(`Usage: ${JOB_USAGE} | delete NAME | run NAME`);
        }
      },
    },
    {
      name: "stop-gateway",
      usage: "/stop-gateway  stop the gateway; every terminal disconnects",
      run: async (_args, app) =>
        void (await app.ui.actions.observe("stop the gateway", owner(app.ui.lifecycle).stop({}))),
    },
    {
      name: "expand",
      usage: "/expand  full tool output",
      run: async (_args, app) => {
        expanded = !expanded;
        app.transcript.setExpanded(expanded);
      },
    },
    {
      name: "panel",
      usage: "/panel N  view N of the newest rendering (0 overview)",
      run: async (args, app) => {
        const index = Number(args.trim());
        if (!Number.isInteger(index) || index < 0) throw new Error("Usage: /panel N");
        app.openPanel(index);
      },
    },
    {
      name: "inspect",
      usage: "/inspect PATH  render a saved file here",
      run: async (args, app) => {
        const path = args.trim();
        if (path === "") throw new Error("Usage: /inspect PATH");
        const progress = new ProgressPanel(app.tui);
        app.showProgress(progress);
        try {
          app.transcript.showRendering(await inspectPath(path, app.cwd(), app.renderer, progress));
        } finally {
          app.showProgress(null);
          progress.dispose();
        }
      },
    },
    {
      name: "image",
      usage: "/image PATH [TEXT]  send an image to the agent",
      run: async (args, app) => {
        const [path = "", ...words] = args.trim().split(/\s+/);
        if (path === "") throw new Error("Usage: /image PATH [TEXT]");
        const shown = shownOf(app);
        const absolute = resolve(app.cwd(), path);
        const rendering = await app.renderer.render([{ path: absolute, sha256: null }], basename(absolute));
        if (rendering === null || rendering.images.length === 0) throw new Error(`${absolute} is not an image`);
        if (app.shown()?.conversation !== shown.conversation)
          throw new Error(`Not sent: the chat left #${shown.id} while ${basename(absolute)} rendered.`);
        await send(app, shown, words.join(" ") || "Inspect this attached image.", rendering.images, false);
      },
    },
  ];
  return commands;
}
