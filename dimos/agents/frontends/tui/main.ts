import { RefusalError, owner } from "../../sdk/failures.ts";
import { retryByRequestId } from "../../sdk/retry.ts";
import { isChordError, type Ref } from "@earendil-works/chord";
import { done } from "../../shared/protocol/outcome.ts";
import { ConversationContract } from "../../shared/protocol/conversation/contract.ts";
import type { CallId } from "../../shared/protocol/conversation/calls.ts";
import type { Component, OverlayHandle } from "@earendil-works/pi-tui";
import type { ConversationId, EntryId } from "@earendil-works/pi-durable";
import type { EpochMs } from "../../shared/protocol/time.ts";
import type { Gallery } from "./views/gallery.ts";
import type { ImageAddress } from "../../shared/protocol/conversation/images.ts";
import type { ImageContent } from "@earendil-works/pi-ai";
import type { Outcome } from "../../shared/protocol/outcome.ts";
import type { ProgressPanel } from "./conversation/progress.ts";
import type { Role } from "../../sdk/state.ts";
import type { Selectable } from "../../shared/view-models/conversation/history.ts";
import type { SelectionChoice } from "./conversation/selection.ts";
import type { SidebarSource } from "./sidebar/sidebar.ts";
import type { TerminalApp } from "./commands.ts";
import type { TimelineRow } from "../../shared/view-models/conversation/timeline.ts";
import {
  Container,
  HStack,
  ProcessTerminal,
  ScrollView,
  Spacer,
  TuiAltScreen,
  VStack,
  isKeyRelease,
  isKeyRepeat,
  matchesKey,
  truncateToWidth,
  wrapTextWithAnsi,
} from "@earendil-works/pi-tui";
import { BackgroundPanel } from "./conversation/background.ts";
import { FileRenderer } from "../../shared/render/render.ts";
import { LoginPanel } from "./dialogs.ts";
import { MessageSelection } from "./conversation/selection.ts";
import { PromptEditor } from "./conversation/input.ts";
import { MenuView } from "./menu/view.ts";
import { SIDEBAR_COLUMNS, Sidebar } from "./sidebar/sidebar.ts";
import { StatusLine, accent, amber, levelColor, muted } from "./style.ts";
import { RemoteFrames } from "../../sdk/streams/remote.ts";
import { SwitcherOverlay } from "./conversation/switcher.ts";
import { TranscriptView } from "./conversation/transcript.ts";
import { actionLine, pendingSends } from "../../shared/view-models/actions.ts";
import { attachRefusal, viewingRefusal } from "../../sdk/conversation.ts";
import { backgroundTarget, movedRows, runningCalls } from "../../shared/view-models/conversation/tools.ts";
import { clean, cleanMultiline } from "../../shared/view-models/text.ts";
import { unixSocket } from "../../sdk/node.ts";
import { connect } from "../../sdk/main.ts";
import { GatewayContract } from "../../shared/protocol/gateway/contract.ts";
import { InventoryContract } from "../../shared/protocol/inventory/contract.ts";
import { LifecycleContract } from "../../shared/protocol/lifecycle/contract.ts";
import { Feedback } from "./feedback.ts";
import type { Ui } from "./session.ts";
import { connectionSegment } from "../../shared/view-models/connection.ts";
import { duration } from "../../shared/view-models/time.ts";
import { editImages, editText, selectables } from "../../shared/view-models/conversation/history.ts";
import { editorTextOf, queuedRows, userQueued } from "../../shared/view-models/conversation/queue.ts";
import { footerOf } from "../../shared/view-models/conversation/footer.ts";
import { initTheme } from "@earendil-works/pi-coding-agent";
import { isRunning, timelineOf } from "../../shared/view-models/conversation/timeline.ts";
import { linkSegments } from "../../shared/view-models/gateway/link.ts";
import { listen } from "../../sdk/state.ts";
import { newActionRequest, newUserRequest } from "../../shared/protocol/actions/request.ts";
import { refusalText } from "../../shared/view-models/outcome.ts";
import { runSetup } from "./setup.ts";
import { slashCommands } from "./commands.ts";
import { stripVTControlCharacters } from "node:util";
import { switcherRows } from "../../shared/view-models/gateway/switcher.ts";

type TerminalOptions = {
  readonly socketPath: string;
  /** Show this conversation; null starts a new one in `cwd`. */
  readonly conversation: ConversationId | null;
  readonly role: Role;
  readonly cwd: string | null;
  /** `~/.cache/dimos/agent/render/`, shared with the gateway, for `/inspect` and `/image`. */
  readonly renderCacheDir: string;
};

/** The conversation this terminal shows, and the unsubscribe of its members. */
type Showing = {
  readonly conversation: Ref<typeof ConversationContract>;
  readonly id: ConversationId;
  readonly stop: () => void;
};

/** A line this terminal sets itself on the action line until a newer action record replaces it. */
type LocalLine = { readonly text: string; readonly afterSeq: number };

/**
 * What the input area is doing: the editor takes typed text, the login panel takes a credential, message selection
 * takes the arrows and its choice keys, and an edit sends the editor's text and `images` into a fork before `message`.
 */
type InputMode =
  | { readonly kind: "chat" }
  | { readonly kind: "login" }
  | { readonly kind: "select"; readonly selection: MessageSelection }
  | {
      readonly kind: "edit";
      readonly message: EntryId;
      readonly position: number;
      readonly of: ConversationId;
      readonly images: readonly ImageAddress[];
    };

const RENDER_CACHE_BYTES = 128 * 1024 * 1024;

const RENDER_SOURCE_BYTES = 64 * 1024 * 1024;

/** The sidebar shows only when the chat keeps at least this many columns beside it. */
const CHAT_MIN_COLUMNS = 60;

const SWITCHER_COLUMNS = 100;

/** One line drawn from served values when it renders; nothing when `text` gives null. */
function lineOf(text: () => string | null): Component {
  return {
    render: (width) => {
      const value = text();
      return value === null ? [] : [truncateToWidth(value, width)];
    },
    invalidate: () => {},
  };
}

/**
 * The TUI. Connects as `tui`, runs setup when no default model is set, shows a conversation as owner or viewer, and lays
 * out the header with the DimOS link, the chat, the action line, the editor and the footer, with the stream sidebar
 * beside them. Every component reads the members it draws when it renders; subscriptions only ask for a frame.
 * Resolves with the exit code.
 */
export async function runTerminal(options: TerminalOptions): Promise<number> {
  const client = await connect(unixSocket({ path: options.socketPath, label: "tui" }));
  try {
    const lifecycle = await client.host.get(LifecycleContract, {});
    try {
      const gateway = await client.host.get(GatewayContract, {});
      try {
        return await terminal(options, { ...client, gateway, lifecycle, actions: new Feedback(client.ended) });
      } finally {
        gateway.release();
      }
    } finally {
      lifecycle.release();
    }
  } finally {
    await client.close();
  }
}

async function terminal(options: TerminalOptions, ui: Ui): Promise<number> {
  const connected = ui.connection.value;
  if (connected.state === "refused") {
    process.stderr.write(`${refusalText(connected.refusal)}\n`);
    return 1;
  }
  if (ui.gateway.settings.value.model === null && options.role === "owner" && options.conversation === null) {
    const setup = await runSetup(ui);
    if (setup.kind === "cancelled") {
      return 130;
    }
  }
  initTheme(ui.gateway.settings.value.theme ?? "dark", false);
  const device = new ProcessTerminal();
  const tui = new TuiAltScreen(device);
  const commands = slashCommands();
  const renderer = new FileRenderer({
    cacheDir: options.renderCacheDir,
    maxCacheBytes: RENDER_CACHE_BYTES,
    maxSourceBytes: RENDER_SOURCE_BYTES,
  });
  const requestRender = () => tui.requestRender();

  let showing = null as Showing | null;
  let localLine: LocalLine | null = null;
  let mode: InputMode = { kind: "chat" };
  let overlay: OverlayHandle | null = null;

  const cwd = (): string => {
    const id = showing?.id;
    const row = ui.gateway.conversations.value.find((candidate) => candidate.id === id);
    if (row !== undefined && row.cwd !== null) return row.cwd;
    return options.cwd ?? process.cwd();
  };
  const rows = (): readonly TimelineRow[] => {
    if (showing === null) return [];
    const state = showing.conversation;
    return timelineOf(
      {
        transcript: state.transcript.value,
        live: state.live.value,
        queue: state.queue.value,
        outcomes: state.outcomes.value,
        detachable: state.detachable.value,
      },
      pendingSends(ui.actions.value, showing.id),
    );
  };
  const running = () => showing !== null && isRunning(showing.conversation.live.value);
  const backgroundable = () => backgroundTarget(runningCalls(rows(), Date.now() as EpochMs)) !== null;
  const setLocalLine = (text: string) => {
    localLine = { text, afterSeq: ui.actions.value.at(-1)?.seq ?? 0 };
    requestRender();
  };

  const transcript = new TranscriptView(
    tui,
    cwd,
    () => (showing === null ? null : { rows: rows(), omitted: showing.conversation.transcript.value.omitted }),
    (gallery, index) => openPanel(gallery, index),
  );
  const chatScroll = new ScrollView(transcript, { follow: "end", primary: true });
  const editor = new PromptEditor(
    tui,
    commands.map((command) => command.name),
    cwd(),
  );
  const login = new LoginPanel(tui, ui);
  const progressSlot = new Container();
  const inputSlot = new Container();
  inputSlot.addChild(editor);

  const sidebarSource = (): SidebarSource | null => {
    if (showing === null) return null;
    const state = showing.conversation;
    return {
      streams: state.streams.value,
      modules: state.modules.value,
      jobs: state.jobs.value,
      transcript: state.transcript.value,
      queue: state.queue.value,
      system: { streams: ui.gateway.streams.value, modules: ui.gateway.modules.value },
    };
  };
  const frames = new RemoteFrames((error: unknown) => note(error));
  let inventory: Ref<typeof InventoryContract> | null = null;
  const configureFrames = () => {
    const view = inventory?.inventory.value;
    frames.configure(
      ui.connection.value.state === "live" && ui.gateway.link.value.state === "up" && view?.current
        ? view.inventory.environment.relay
        : null,
    );
  };
  const sidebar = new Sidebar(sidebarSource, frames, requestRender, () => device.rows);

  /** One moved call's output, following its end; once the call ends it keeps the last output it showed. */
  const movedOutput = (id: ConversationId, call: CallId): Component => {
    let last: { readonly output: string; readonly truncated: boolean } | null = null;
    return {
      render: (width) => {
        const moved =
          showing?.id === id ? showing.conversation.moved.value.find((row) => row.call === call) : undefined;
        if (moved !== undefined) last = moved;
        const lines: string[] = [];
        if (last?.truncated === true) lines.push(muted("[earlier output was cut]"));
        const text = cleanMultiline(stripVTControlCharacters(last?.output ?? "")).replaceAll("\t", "  ");
        if (text !== "") lines.push(...text.split("\n").flatMap((line) => wrapTextWithAnsi(line, width)));
        else if (moved !== undefined) lines.push(muted("No output yet."));
        if (moved === undefined) lines.push(muted("It is no longer running; its result is in the chat."));
        return lines;
      },
      invalidate: () => {},
    };
  };
  const background = new BackgroundPanel(
    () => (showing === null ? [] : movedRows(showing.conversation.moved.value, Date.now() as EpochMs)),
    {
      open: (call) => {
        if (showing === null) return;
        const row = movedRows(showing.conversation.moved.value, Date.now() as EpochMs).find(
          (candidate) => candidate.call === call,
        );
        fullScreen(
          clean(row?.label ?? call),
          movedOutput(showing.id, call),
          true,
          () => {
            if (!background.isEmpty) tui.setFocus(background);
          },
          "end",
        );
      },
      stop: (call) => {
        if (showing === null) return;
        const { conversation } = showing;
        try {
          void ui.actions.observe(`stop call ${call}`, owner(conversation).stopBackground({ call })).catch(note);
        } catch (error) {
          note(error);
        }
      },
      leave: () => {
        tui.setFocus(focusOf(mode));
        requestRender();
      },
    },
  );

  const conversationText = (): string => {
    if (showing === null) return "";
    if (showing.conversation.variant === "owner") return `#${showing.id}`;
    return `#${showing.id} VIEW`;
  };
  const linkText = (): string => {
    const gateway = ui.gateway;
    const segments = [
      ...linkSegments(gateway.link.value, gateway.load.value, gateway.runs.value, Date.now() as EpochMs),
    ];
    const connection = ui.connection.value;
    if (connection.state !== "live") segments.unshift(connectionSegment(connection));
    return segments.map((segment) => levelColor(segment.level, segment.text)).join(muted(" · "));
  };
  const actionText = (): string | null => {
    const newest = ui.actions.value.at(-1);
    if (localLine !== null && (newest === undefined || newest.seq <= localLine.afterSeq)) return ` ${localLine.text}`;
    return newest === undefined ||
      (newest.status === "done" && (newest.sending !== null || newest.action === "conversation.attach"))
      ? null
      : ` ${clean(actionLine(newest))}`;
  };
  const hintText = (): string | null => {
    if (showing === null) return muted("  Connecting…");
    const state = showing.conversation;
    switch (mode.kind) {
      case "select":
      case "login":
        return null;
      case "edit": {
        const count = mode.images.length;
        const images = count === 0 ? "" : ` + ${count === 1 ? "image" : `${count} images`}`;
        return muted(
          `  Editing message ${mode.position} of #${mode.of}${images} · Enter sends into a new fork · Esc cancels`,
        );
      }
      case "chat":
        break;
    }
    if (showing.conversation.variant === "viewer")
      return muted(`  ${refusalText(viewingRefusal(showing.id))} /conversations to switch`);
    if (!running()) return null;
    const queued = userQueued(queuedRows(state.queue.value)).length > 0 ? " · Up edits queue" : "";
    return muted(`  Esc stops · Enter steers · /follow queues${queued}`);
  };
  const footerNow = () => {
    if (showing === null) return null;
    const state = showing.conversation;
    return footerOf({
      transcript: state.transcript.value,
      live: state.live.value,
      queue: state.queue.value,
      usage: state.usage.value,
      agent: state.agent.value,
      settings: ui.gateway.settings.value,
      accounts: ui.gateway.accounts.value,
    });
  };
  /** " 2m": a run's start is the gateway's switcher row, where every busy conversation is named. */
  const runningFor = (id: ConversationId): string => {
    const busy = ui.gateway.conversations.value.find((row) => row.id === id)?.running;
    return busy === undefined || busy === null ? "" : ` ${duration(Date.now() - busy.since)}`;
  };
  const footerText = (): string => {
    const footer = footerNow();
    if (showing === null || footer === null) return "";
    const parts: string[] = [];
    if (footer.model === null) parts.push(muted("/model PROVIDER MODEL"));
    else {
      parts.push(clean(`${footer.model.provider} / ${footer.model.modelId}`));
      if (showing.conversation.agent.value.model === undefined) parts.push(muted("default model"));
    }
    if (footer.thinking !== null) parts.push(footer.thinking);
    parts.push(footer.running ? `running${runningFor(showing.id)}` : "idle");
    const queued = [
      footer.queued.steer > 0 ? `${footer.queued.steer} steer` : null,
      footer.queued.followUp > 0 ? `${footer.queued.followUp} follow` : null,
      footer.queued.notices > 0 ? `⚡${footer.queued.notices}` : null,
    ].filter((part) => part !== null);
    if (queued.length > 0) parts.push(`queue ${queued.join(" ")}`);
    parts.push(`$${footer.costUsd.toFixed(2)}`);
    if (footer.context !== null)
      parts.push(levelColor(footer.context.level, `ctx ${Math.round(footer.context.fraction * 100)}% left`));
    return parts.join(muted(" · "));
  };
  const chat = new VStack([
    new StatusLine(() => [accent("⠿ dimos agent"), muted(conversationText())]),
    new StatusLine(() => [muted(clean(cwd())), linkText()]),
    new Spacer(1),
    { component: chatScroll, basis: 0, grow: 1, minSize: 1 },
    progressSlot,
    lineOf(actionText),
    lineOf(hintText),
    inputSlot,
    background,
    new StatusLine(() => [footerText(), muted("/help")]),
  ]);
  const layout = new HStack([
    { component: chat, basis: 0, grow: 1, minSize: CHAT_MIN_COLUMNS },
    {
      component: sidebar.column,
      basis: SIDEBAR_COLUMNS,
      visible: (viewport) => {
        const visible = sidebar.isOpen && overlay === null && viewport.width >= CHAT_MIN_COLUMNS + SIDEBAR_COLUMNS;
        sidebar.setVisible(visible);
        if (!visible && sidebar.focused) tui.setFocus(editor);
        return visible;
      },
    },
  ]);

  let resolveExit: (code: number) => void = () => {};
  const exited = new Promise<number>((resolve) => {
    resolveExit = resolve;
  });

  const focusOf = (current: InputMode): Component => {
    switch (current.kind) {
      case "login":
        return login;
      case "select":
        return current.selection;
      case "chat":
      case "edit":
        return editor;
    }
  };
  const setMode = (next: InputMode) => {
    mode = next;
    const focused = focusOf(next);
    inputSlot.clear();
    inputSlot.addChild(focused);
    tui.setFocus(focused);
    requestRender();
  };
  /** Back to the editor from selection or an edit: the mark goes, the chat follows its end again. */
  const leaveToChat = () => {
    if (mode.kind === "edit") editor.setValue("");
    if (mode.kind === "select") {
      transcript.select(null);
      chatScroll.scrollToEnd();
    }
    setMode({ kind: "chat" });
  };
  /** The login panel takes the editor's place while this terminal's login flow runs. */
  const syncLogin = () => {
    const wanted = ui.gateway.loginFlow.value.phase === "running";
    if (wanted && mode.kind !== "login") {
      leaveToChat();
      setMode({ kind: "login" });
    } else if (!wanted && mode.kind === "login") setMode({ kind: "chat" });
  };

  /** Closes the full-screen view back to the chat; null while the chat shows. */
  let closeFullScreen: (() => void) | null = null;
  /**
   * Shows `component` in place of the chat. Esc closes it unless `closesOnEscape` is false, for a view that handles Esc.
   * `onClose` runs once the chat is back, so it may move the focus; `follow: "end"` keeps a growing view at its end.
   */
  const fullScreen = (
    title: string,
    component: Component,
    closesOnEscape: boolean,
    onClose: () => void,
    follow: "none" | "end" = "none",
  ) => {
    sidebar.setVisible(false);
    const close = () => {
      closeFullScreen = null;
      tui.setLayoutRoot(layout);
      tui.setFocus(focusOf(mode));
      onClose();
      tui.requestRender();
    };
    closeFullScreen = close;
    const focused: Component = {
      render: (width) => component.render(width),
      invalidate: () => component.invalidate(),
      handleMouse: (event) => component.handleMouse?.(event),
      handleInput: (data) => {
        if (closesOnEscape && matchesKey(data, "escape")) close();
        else component.handleInput?.(data);
        tui.requestRender();
      },
    };
    tui.setLayoutRoot(
      new VStack([
        new StatusLine(() => [accent(`⠿ dimos agent / ${title}`), muted("Esc returns to chat")]),
        { component: new ScrollView(focused, { primary: true, follow }), basis: 0, grow: 1 },
      ]),
    );
    tui.setFocus(focused);
    tui.requestRender();
  };

  const openPanel = (gallery: Gallery, index: number) => {
    gallery.select(index);
    if (gallery.interactive)
      fullScreen(gallery.details.views[index - 1]?.label ?? "view", gallery, true, () => gallery.select(0));
  };

  /** The unsent text of each conversation that is not shown; it returns to the editor when its conversation shows. */
  const drafts = new Map<ConversationId, string>();
  const restore = (from: ConversationId, text: string) => {
    if (showing?.id === from) {
      const typed = editor.getValue();
      editor.setValue(typed.trim() === "" ? text : `${typed}\n\n${text}`);
    } else {
      const before = drafts.get(from);
      drafts.set(from, before === undefined ? text : `${before}\n\n${text}`);
      transcript.note(
        muted(`Queued messages of #${from} came back; they return to the editor when #${from} shows:\n${clean(text)}`),
      );
    }
    requestRender();
  };

  /** Counts switchTo calls: one that a newer call overtook while it waited gives up, and releases what it got. */
  let switches = 0;
  const switchTo = async (conversation: ConversationId | null, role: Role): Promise<void> => {
    const switching = ++switches;
    let id = conversation;
    if (id === null) {
      const created = await ui.actions.observe(
        "new conversation",
        retryByRequestId(
          ui,
          {
            request: newActionRequest(),
            cwd: options.cwd,
          },
          (args) => owner(ui.gateway).createConversation(args),
        ),
      );
      if (!created.ok || switching !== switches) return;
      id = created.value.conversation;
    }
    if (mode.kind === "select" || mode.kind === "edit") leaveToChat();
    const previous = showing;
    showing = null;
    if (previous !== null) {
      const typed = editor.getValue();
      if (typed.trim() !== "") drafts.set(previous.id, typed);
      editor.setValue("");
      previous.stop();
      await previous.conversation.release();
    }
    if (switching !== switches) return;
    transcript.reset();
    const attached = await ui.actions.observe(
      `attach #${id}`,
      ui.host
        .get(ConversationContract, { conversation: id, role })
        .then(done)
        .catch((error: unknown) => {
          if (isChordError(error) && error.code !== "aborted" && error.reason !== "ready_timeout")
            throw new RefusalError({ code: "not_found", what: "conversation", name: `#${id}` });
          throw error;
        }),
      { action: "conversation.attach", conversation: id },
    );
    if (!attached.ok) return;
    const shown = attached.value;
    if (switching !== switches) {
      shown.release();
      return;
    }
    const stop = listen(
      [
        shown.record,
        shown.transcript,
        shown.live,
        shown.queue,
        shown.usage,
        shown.agent,
        shown.outcomes,
        shown.detachable,
        shown.moved,
        shown.modules,
        shown.jobs,
        shown.streams,
      ],
      requestRender,
    );
    let previousRole = shown.variant;
    const stopShape = shown.onShape(() => {
      if (previousRole === "owner" && shown.variant === "viewer")
        ui.actions.refuse(`attach #${id}`, viewingRefusal(id));
      previousRole = shown.variant;
      requestRender();
    });
    showing = {
      conversation: shown,
      id,
      stop: () => {
        stop();
        stopShape();
      },
    };
    const refusal = attachRefusal(id, role, shown.variant);
    if (refusal !== null) ui.actions.refuse(`attach #${id}`, refusal);
    editor.setWorkspace(cwd());
    const back = drafts.get(id);
    drafts.delete(id);
    if (back !== undefined) restore(id, back);
    requestRender();
  };

  const note = (error: unknown) => {
    if (error instanceof RefusalError) ui.actions.refuse("action", error.refusal);
    else transcript.note(amber(clean(error instanceof Error ? error.message : String(error))));
  };

  const closeSwitcher = () => {
    if (overlay === null) return;
    overlay.hide();
    overlay = null;
    tui.setFocus(focusOf(mode));
    requestRender();
  };
  const openSwitcher = () => {
    if (overlay !== null) return;
    closeFullScreen?.();
    sidebar.setVisible(false);
    const view = new SwitcherOverlay(
      () => {
        const gateway = ui.gateway;
        return switcherRows(gateway.conversations.value, showing?.id ?? null, Date.now() as EpochMs);
      },
      {
        attach: (conversation, role) => {
          closeSwitcher();
          void switchTo(conversation, role).catch(note);
        },
        create: () => {
          closeSwitcher();
          void switchTo(null, "owner").catch(note);
        },
        close: closeSwitcher,
      },
    );
    overlay = tui.showOverlay(view, { width: SWITCHER_COLUMNS, maxHeight: "80%", anchor: "center" });
    requestRender();
  };

  /** Marks the message and scrolls the chat so it starts at the top of the viewport. */
  const reveal = (target: Selectable) => {
    const key = `e${target.entry}`;
    transcript.select(key);
    const offset = transcript.offsetOf(key);
    if (offset !== null) chatScroll.scrollTo(offset, { disableFollow: true });
  };
  const attachForkIfShown =
    (from: ConversationId) => async (result: Outcome<{ readonly conversation: ConversationId }>) => {
      if (!result.ok) return;
      const fork = result.value.conversation;
      if (showing?.id === from) await switchTo(fork, "owner");
      else transcript.note(muted(`Fork #${fork} of #${from} is ready; /resume ${fork} opens it.`));
    };
  const act = (choice: SelectionChoice) => {
    if (showing === null || choice.kind === "cancel") return;
    const { conversation, id } = showing;
    switch (choice.kind) {
      case "edit": {
        const position = selectables(conversation.transcript.value).findIndex(
          (candidate) => candidate.entry === choice.target.entry,
        );
        editor.setValue(editText(choice.target.item));
        setMode({
          kind: "edit",
          message: choice.target.entry,
          position: position + 1,
          of: id,
          images: editImages(choice.target.item),
        });
        return;
      }
      case "summarize_up_to":
        void ui.actions
          .observe(
            "summarize up to here",
            retryByRequestId(ui, { request: newActionRequest(), message: choice.target.entry }, (args) =>
              owner(conversation).summarizeUpTo(args),
            ),
          )
          .then(attachForkIfShown(id))
          .catch(note);
        return;
      case "summarize_from":
        void ui.actions
          .observe(
            "summarize from here",
            retryByRequestId(ui, { request: newActionRequest(), message: choice.target.entry }, (args) =>
              owner(conversation).summarizeFrom(args),
            ),
          )
          .then(attachForkIfShown(id))
          .catch(note);
        return;
    }
  };
  const enterSelection = () => {
    if (showing === null) return;
    const targets = selectables(showing.conversation.transcript.value);
    if (targets.length === 0) return;
    const selection = new MessageSelection(targets, {
      moved: reveal,
      choose: (choice) => {
        leaveToChat();
        act(choice);
      },
    });
    setMode({ kind: "select", selection });
    const target = selection.target;
    if (target !== null) reveal(target);
  };

  let menuView = null as MenuView | null;
  const app: TerminalApp = {
    ui,
    tui,
    transcript,
    renderer,
    shown: () => (showing === null ? null : { conversation: showing.conversation, id: showing.id }),
    rows,
    cwd,
    switchTo,
    restore,
    showProgress: (panel: ProgressPanel | null) => {
      progressSlot.clear();
      if (panel !== null) progressSlot.addChild(panel);
      requestRender();
    },
    openMenu: () => {
      closeSwitcher();
      const view = new MenuView({
        inventory,
        ui,
        current: showing?.id ?? null,
        attach: (conversation) => {
          closeFullScreen?.();
          void switchTo(conversation, "owner").catch(note);
        },
        close: () => closeFullScreen?.(),
        requestRender,
        height: () => device.rows - 1,
      });
      menuView = view;
      fullScreen("menu", view, false, () => {
        view.dispose();
        menuView = null;
      });
    },
    openSwitcher,
    openPanel: (index) => {
      const gallery = transcript.latestGallery;
      if (gallery === null) throw new Error("No rendering yet.");
      if (index > gallery.details.views.length) throw new Error(`Choose /panel 0–${gallery.details.views.length}.`);
      openPanel(gallery, index);
    },
    exit: (code) => resolveExit(code),
  };
  const runCommand = (value: string) => {
    const [name = "", ...rest] = value.slice(1).split(" ");
    const command = commands.find((candidate) => candidate.name === name);
    if (command === undefined) {
      note(`Unknown command /${name}; /help lists them.`);
      return;
    }
    command.run(rest.join(" "), app).catch(note).finally(requestRender);
  };

  editor.onSubmit = (value) => {
    if (mode.kind === "edit") {
      if (value.trim() === "" || showing === null) return;
      const { conversation, id } = showing;
      const { message, images: addresses } = mode;
      editor.addToHistory(value);
      leaveToChat();
      const loaded = async (): Promise<Outcome<ImageContent[]>> => {
        const images: ImageContent[] = [];
        for (const at of addresses) {
          const image = await conversation.image({ at });
          if (showing?.conversation !== conversation)
            throw new Error(`Not sent: the chat left #${id} while the message's images loaded.`);
          if (!image.ok) return image;
          images.push(image.value);
        }
        return done(images);
      };
      void loaded()
        .then((images) => {
          if (!images.ok) return ui.actions.refuse("edit and resend", images.refusal);
          return ui.actions
            .observe(
              "edit and resend",
              retryByRequestId(ui, { request: newUserRequest(), message, text: value, images: images.value }, (args) =>
                owner(conversation).edit(args),
              ),
            )
            .then(attachForkIfShown(id));
        })
        .catch(note);
      return;
    }
    if (value.startsWith("/")) {
      editor.addToHistory(value);
      editor.setValue("");
      runCommand(value);
      return;
    }
    if (value.trim() === "" || showing === null) return;
    const { conversation } = showing;
    if (conversation.variant === "owner") {
      editor.addToHistory(value);
      editor.setValue("");
    }
    const args = {
      request: newUserRequest(),
      text: value,
      images: [],
      whenBusy: running() ? ("steer" as const) : ("followUp" as const),
    };
    void ui.actions.observe(
      args.whenBusy === "steer" ? "send" : "queue a follow-up",
      retryByRequestId(ui, args, (args) => owner(conversation).send(args)),
      { action: "conversation.send", conversation: showing.id, sending: { request: args.request, text: value } },
    );
  };
  editor.onEscape = () => {
    if (mode.kind === "edit") {
      leaveToChat();
      return;
    }
    if (running()) {
      runCommand("/abort");
      return;
    }
    if (mode.kind === "chat" && editor.getValue() === "") enterSelection();
  };

  tui.addInputListener((data) => {
    // The kitty keyboard protocol and modifyOtherKeys send Ctrl-C as an escape sequence, not \x03.
    if (matchesKey(data, "ctrl+c") && !isKeyRelease(data)) {
      const editing = overlay === null && closeFullScreen === null && (mode.kind === "chat" || mode.kind === "edit");
      if (editing && editor.getValue() !== "") {
        editor.setValue("");
        requestRender();
      } else resolveExit(0);
      return { consume: true };
    }
    if (overlay !== null) return undefined;
    if (matchesKey(data, "ctrl+t")) {
      if (isKeyRelease(data) || isKeyRepeat(data)) return { consume: true };
      sidebar.setOpen(!sidebar.isOpen);
      if (!sidebar.isOpen && sidebar.focused) tui.setFocus(editor);
      return { consume: true };
    }
    if (
      !tui.hasOverlay() &&
      !isKeyRelease(data) &&
      matchesKey(data, "shift+tab") &&
      (mode.kind === "chat" || mode.kind === "edit")
    ) {
      if (sidebar.focused) tui.setFocus(editor);
      else if (sidebar.isVisible && device.columns >= CHAT_MIN_COLUMNS + SIDEBAR_COLUMNS) {
        tui.setFocus(sidebar);
      }
      requestRender();
      return { consume: true };
    }
    if (sidebar.focused && matchesKey(data, "escape")) {
      tui.setFocus(editor);
      requestRender();
      return { consume: true };
    }
    if (isKeyRelease(data)) return undefined;
    if (matchesKey(data, "ctrl+o")) {
      openSwitcher();
      return { consume: true };
    }
    if (matchesKey(data, "ctrl+b") && backgroundable()) {
      runCommand("/background");
      return { consume: true };
    }
    if (mode.kind !== "chat") return undefined;
    if (
      matchesKey(data, "down") &&
      editor.focused &&
      !editor.isShowingAutocomplete() &&
      editor.getValue() === "" &&
      !background.isEmpty
    ) {
      tui.setFocus(background);
      requestRender();
      return { consume: true };
    }
    if (!matchesKey(data, "up") || !editor.focused || editor.isShowingAutocomplete() || editor.getValue() !== "")
      return undefined;
    if (showing === null || showing.conversation.variant !== "owner") return undefined;
    const { conversation, id } = showing;
    if (userQueued(queuedRows(conversation.queue.value)).length === 0) return undefined;
    void ui.actions
      .observe(
        "bring queued messages back",
        retryByRequestId(ui, { request: newActionRequest() }, (args) => owner(conversation).withdraw(args)),
      )
      .then((back) => {
        if (back.ok && back.value.length > 0) restore(id, editorTextOf(back.value));
      });
    return { consume: true };
  });

  const stops = [
    ui.gateway.link.subscribe(configureFrames),
    listen(
      [
        ui.gateway.link,
        ui.gateway.load,
        ui.gateway.runs,
        ui.gateway.conversations,
        ui.gateway.modules,
        ui.gateway.streams,
        ui.gateway.jobs,
        ui.gateway.settings,
        ui.gateway.accounts,
        ui.actions,
      ],
      requestRender,
    ),
    ui.gateway.loginFlow.subscribe(() => {
      syncLogin();
      requestRender();
    }),
    ui.connection.subscribe(() => {
      configureFrames();
      if (ui.connection.value.state === "refused") resolveExit(1);
      requestRender();
    }),
  ];

  const ticker = setInterval(() => {
    if (!background.isEmpty) requestRender();
  }, 1_000);

  const hangUp = () => resolveExit(129);
  const terminate = () => resolveExit(143);
  // Node prints the error and exits after the monitor, so the shell gets its screen and modes back first.
  const restoreTerminal = () => tui.stop();
  let code: number;
  try {
    inventory = await ui.host.get(InventoryContract, {});
    stops.push(
      inventory.inventory.subscribe(() => {
        configureFrames();
        requestRender();
      }),
    );
    configureFrames();
    await switchTo(options.conversation, options.role);
    tui.setLayoutRoot(layout);
    syncLogin();
    tui.setFocus(focusOf(mode));
    tui.start();
    process.on("SIGHUP", hangUp);
    process.on("SIGTERM", terminate);
    process.on("uncaughtExceptionMonitor", restoreTerminal);
    requestRender();
    code = await exited;
  } finally {
    clearInterval(ticker);
    for (const stop of stops) stop();
    closeSwitcher();
    menuView?.dispose();
    login.dispose();
    transcript.dispose();
    sidebar.dispose();
    await frames.close();
    if (inventory !== null) await inventory.release();
    tui.stop();
    process.off("SIGHUP", hangUp);
    process.off("SIGTERM", terminate);
    process.off("uncaughtExceptionMonitor", restoreTerminal);
    const last = showing;
    showing = null;
    if (last !== null) {
      last.stop();
      last.conversation.release();
    }
  }
  if (code === 1) {
    process.stderr.write("The gateway disconnected.\n");
    const connection = ui.connection.value;
    if (connection.state === "refused") process.stderr.write(`${refusalText(connection.refusal)}\n`);
  }
  return code;
}
