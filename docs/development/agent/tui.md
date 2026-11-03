# Terminal application

`dimos/agents/frontends/tui/` contains the terminal, the pi-tui application that `dimos agent` opens on one conversation.
`dimos agent` starts the gateway when none runs, then [cli/commands/tui.ts](/dimos/agents/cli/commands/tui.ts) calls
`runTerminal` from [main.ts](/dimos/agents/frontends/tui/main.ts). The terminal reaches the gateway only through the [SDK](/docs/usage/agent/sdk.md). It turns served state
into rows with [view models](/docs/development/agent/view-models.md) and previews media with
[shared/render/](/docs/development/agent/render.md). [The terminal](/docs/usage/agent.md#the-terminal) in
docs/usage/agent.md describes what a user sees and lists every key.

Source: [dimos/agents/frontends/tui/](/dimos/agents/frontends/tui/). TypeScript examples use `#dimos/agents/...` package subpaths.
Save temporary example files there and run them from `dimos/agents` with `node --import=./test/web-sdk.mjs frontends/tui/demo_example.ts`.
Remove the temporary file with `rm frontends/tui/demo_example.ts` before running `npm run typecheck` or `npm test`.
Those checks treat saved examples as package code and may reject their imports.

## Built on pi-tui and pi-coding-agent

[pi-tui](https://github.com/earendil-works/pi/tree/main/packages/tui) draws the screen. A `TUI` owns the terminal. It
holds the components, sends each key to the component that has focus, and rewrites only the rows that changed. main.ts
runs a `TuiAltScreen` over a `ProcessTerminal`. That renderer draws in the alternate screen buffer and does its own
scrolling. [setup.ts](/dimos/agents/frontends/tui/setup.ts) runs the first-run screens on a `TuiMainScreen`, which draws in the normal buffer.

A `Component` is an object with `render(width)`, which returns one string per row. No row may be wider than `width`, so
components cut their lines with `truncateToWidth`. A component that takes keys has `handleInput(data)`. It gets the raw
terminal bytes while it has focus and decodes them with `matchesKey(data, "escape")`. `invalidate()` drops whatever a
component cached. `/theme` calls `tui.invalidate()` after it changes the theme.

Nothing redraws on its own. A component asks for a frame with `tui.requestRender()`, and pi-tui merges the requests into
one frame. The terminal builds on that rule. Every component reads the SDK state it draws inside `render`, and every
subscription only asks for a frame. main.ts subscribes with `listen` from [sdk/state.ts](/dimos/agents/sdk/state.ts). A new
entry, a queue change or a new stream reaches the screen on the next frame, and the terminal keeps no copy of served
state.

`VStack` and `HStack` split the screen into rows and columns, and `ScrollView` scrolls one region. `tui.setLayoutRoot`
replaces the whole screen, which is how `/menu` and an interactive `/panel` go full screen. `tui.setFocus(component)`
picks the component that gets keys. A component that wraps a text input implements `Focusable` and passes its `focused`
flag down, as `LoginPanel` does. `tui.addInputListener` sees every key before the focused component and can consume it.
main.ts handles the global keys there. `tui.showOverlay` draws a component over the screen and returns an
`OverlayHandle` that hides it again. The conversation switcher is the one overlay.

pi-tui also gives the terminal its editor and its pictures. `Editor` is the multi-line prompt, and
`CombinedAutocompleteProvider` completes slash commands and file paths in it. `Image` draws a picture with the kitty or
iTerm2 graphics protocol. `TerminalImage` in [lib/terminal-image.ts](/dimos/agents/frontends/tui/lib/terminal-image.ts) wraps it for Warp and for
terminals with no graphics protocol.

[pi-coding-agent](https://github.com/earendil-works/pi/tree/main/packages/coding-agent) is Pi's own coding agent. The
harness does not run on it. The terminal imports it for these components:

- `UserMessageComponent` and `AssistantMessageComponent` draw messages in the chat.
- `ToolExecutionComponent` draws tool calls with Pi's renderers from `createBashToolDefinition`,
  `createReadToolDefinition`, `createEditToolDefinition` and `createWriteToolDefinition`. The harness stores `bash`
  details with the `truncation` and `fullOutputPath` fields that Pi's card reads, as `DETAILS` in
  [shared/protocol/tools/details.ts](/dimos/agents/shared/protocol/tools/details.ts) declares.
- `ExtensionSelectorComponent`, `ThemeSelectorComponent`, `OAuthSelectorComponent` and `DynamicBorder` draw the setup
  screens and the login prompts.
- `initTheme`, `getMarkdownTheme` and `getSelectListTheme` set and read Pi's theme. The reused components follow
  `/theme`. The colours in [style.ts](/dimos/agents/frontends/tui/style.ts) are fixed truecolor values and do not.

This folder adds the layout, the wiring from SDK state to rows, the tool cards and media panels, the stream sidebar, the
main menu and pictures drawn in text cells.

## The API

### runTerminal

`runTerminal(options)` in main.ts is the one export a caller outside this folder uses. It resolves with the exit code
when the user detaches. Its options type, `TerminalOptions`, is not exported, so the caller passes an object literal.

| Field            | Type                     | What it sets                                                                        |
| ---------------- | ------------------------ | ----------------------------------------------------------------------------------- |
| `socketPath`     | `string`                 | The gateway's Unix socket.                                                          |
| `conversation`   | `ConversationId \| null` | The conversation to show. `null` starts a new one.                                  |
| `role`           | `Role`                   | `"owner"` takes input. `"viewer"` attaches read-only.                               |
| `cwd`            | `string \| null`         | Where a new conversation works. `null` uses the gateway's workspace setting.        |
| `renderCacheDir` | `string`                 | The render cache the terminal shares with the gateway, for `/inspect` and `/image`. |

The exit code is 0 after Ctrl-C on an empty editor or `/exit`, 1 when the gateway refuses the connection or goes
away, 129 after SIGHUP, 143 after SIGTERM, and 130 when the user cancels first-run setup. A signal or an uncaught error
still restores the terminal.

`runTerminal` runs these steps:

1. It connects with `connect(unixSocket({ path, label: "tui" }))` and holds `LifecycleContract` and `GatewayContract`
   with `host.get`. It joins the client, both references and a new `Feedback` into a `Ui`.
2. If the gateway has refused the connection, it prints the refusal and returns 1. If no default model is set and the
   user opens a new conversation as owner, it runs `runSetup`.
3. It calls `initTheme` with the saved theme and builds the layout on a `TuiAltScreen`. The chat column holds two header
   lines, the transcript in a `ScrollView` that follows the end, a progress slot, the action line, a hint line, the
   input slot, the background panel and the footer. The sidebar's viewport sits beside the chat when the terminal has room for 60 chat columns
   plus `SIDEBAR_COLUMNS`.
4. It shows the conversation with `switchTo`, starts the TUI and waits for an exit code.
5. On exit it stops every subscription, disposes the components, closes `RemoteFrames`, stops the TUI and releases the
   conversation and inventory. Then it releases the gateway and lifecycle references and closes the client.

The input slot holds one component at a time, and an `InputMode` picks it. The `PromptEditor` takes `chat` and `edit`.
The `LoginPanel` takes `login` while this terminal's login flow runs. A `MessageSelection` takes `select` after Esc on
an empty editor. Typed text goes to `send` as a steer while a run goes and as a follow-up otherwise. A line that starts
with `/` runs the slash command of that name. Each conversation keeps its own unsent text. `switchTo` takes the
editor's text with the conversation it leaves and puts back the text kept for the one it shows.

The terminal holds a reference only while its view shows. `LifecycleContract` and `GatewayContract` live for the whole
run. One `ConversationContract` lives while its conversation shows, and `switchTo` releases it before it gets the next
one. The terminal holds `InventoryContract` for its lifetime, configures `RemoteFrames` from its current relay endpoint,
and shares the same reference with `MenuView`.

### Ui and Feedback

`Ui` in [session.ts](/dimos/agents/frontends/tui/session.ts) is the SDK `Client` plus three members. `gateway` is a `Ref<typeof GatewayContract>`,
`lifecycle` is a `Ref<typeof LifecycleContract>`, and `actions` is a `Feedback`. Every component that acts on the
gateway takes a `Ui`. Before it calls a method, it narrows the reference to the `owner` variant. `owner()` from
[sdk/failures.ts](/dimos/agents/sdk/failures.ts) does that and throws a `RefusalError` for a viewer. A component wraps a
`by_request_id` action in `retryByRequestId`.

`Feedback` in [feedback.ts](/dimos/agents/frontends/tui/feedback.ts) is a `Status<readonly ActionRecord[]>` from
[shared/protocol/status.ts](/dimos/agents/shared/protocol/status.ts). It holds the newest 50 actions this terminal started. It
never calls or retries an RPC.

- `observe(text, work, context?)` records `work` as pending, waits for it or for `client.ended`, and records it as done,
  refused or unknown. A thrown error becomes a refusal through `refusalOfFailure`. It returns the `Outcome<T>`.
- `refuse(text, refusal)` records a refusal the terminal decided itself, such as an owner attach that came back as
  viewer.

The action line shows the newest record with `actionLine`. `pendingSends` keeps a sent message in the chat as `sending`
until the transcript or the queue holds its request id. The Overview tab of `/menu` lists the newest refused records.

### runSetup and LoginPanel

`runSetup(ui)` in setup.ts asks for a theme, a provider, a login unless that provider is already configured, and a
default model. It waits up to 5 s for the provider's models to reach `accounts`. Each answer is one gateway action
through `ui.actions.observe`. It resolves with `{ kind: "done" }`, or with `{ kind: "cancelled" }` after Ctrl-C or Esc.
A failed login or a provider with no models rejects.

`LoginPanel` in [dialogs.ts](/dimos/agents/frontends/tui/dialogs.ts) draws this terminal's `loginFlow` member each time it renders. A `select`
prompt is a list, `secret` and `manual_code` are a masked line, and `text` is a plain line. The provider's progress
lines stay above the prompt, so a device code stays visible. Enter answers with `gateway.answerLogin`. Esc answers
`null`, which cancels the flow. Setup and the chat both show it.

### Slash commands

[commands.ts](/dimos/agents/frontends/tui/commands.ts) exports `SlashCommand`, `TerminalApp` and `slashCommands()`. A `SlashCommand` is
`{ name, usage, run(args, app) }`. `run` gets the text after the name and a `TerminalApp`, and returns a promise.
`slashCommands()` returns every command in `/help` order, and the editor completes their names. To add a command, add it
to that array. When `run` throws, main.ts shows a `RefusalError` on the action line and any other error as a chat note.

`TerminalApp` is what main.ts hands each command.

| Member                            | What it does                                                                                    |
| --------------------------------- | ----------------------------------------------------------------------------------------------- |
| `ui`, `tui`, `renderer`           | The `Ui`, the `TUI`, and the `FileRenderer` for `/inspect` and `/image`.                        |
| `transcript`                      | The chat's `TranscriptView`. `note(text)` adds a line and `showRendering(details)` a rendering. |
| `shown()`                         | The shown conversation's reference and id, or `null`.                                           |
| `rows()`                          | The shown conversation's `TimelineRow` list, as the chat draws it.                              |
| `cwd()`                           | The shown conversation's directory.                                                             |
| `switchTo(conversation, role)`    | Shows another conversation, or a new one for `null`.                                            |
| `restore(from, text)`             | Puts text withdrawn from `from` into the editor, or holds it until `from` shows again.          |
| `showProgress(panel)`             | Shows a `ProgressPanel` above the editor, or removes it for `null`.                             |
| `openMenu()`, `openSwitcher()` | Opens `/menu` or the switcher.                                                               |
| `openPanel(index)`                | Selects view `index` of the newest rendering. 0 is the overview.                                |
| `exit(code)`                      | Ends `runTerminal` with `code`.                                                                 |

### Families

Three folders each define a family in a `base.ts`, and one consumer file lists every member.
[test/boundaries.test.ts](/dimos/agents/test/boundaries.test.ts) fails a member that does not import its `base.ts` and a member
the consumer does not import.

| Folder   | Contract       | Consumer                | Members                                   |
| -------- | -------------- | ----------------------- | ----------------------------------------- |
| cards/   | `ToolCard`     | `cardFor` in choose.ts  | `BashCard`, `PiToolCard`                  |
| views/   | `ViewPanel<K>` | `Gallery` in gallery.ts | `ImagePanel`, `SpatialPanel`, `ClipPanel` |
| menu/    | `MenuTab`   | `TABS` in view.ts       | Ten tabs, one per key from 1 to 0         |

`ToolCard` is a `Component` for one tool call. It has the `call`, `output(text)` for the output so far, `end(result)`
for the `ToolResult`, `setExpanded(expanded)` for `/expand`, and `dispose()`, which stops timers.
`cardFor(call, tui, cwd)` returns a `BashCard` for `bash` and a `PiToolCard` for every other tool.

`ViewPanel<K>` is a `Component` for one kind of `RenderedView`. It has the `view` it draws, `dispose()`, and
`handleInput` when it takes keys. A panel reads its own files and calls the `changed` callback it was built with when it
needs a frame.

`MenuTab` has a `title`, an `inventory` flag, and `rows(context)`, which returns the tab's `MenuRow` list each
time the view renders. A `MenuRow` has `text`, a `Level`, its `MenuKey` list, and `open`, which Enter calls.
`open` returns a `MenuDetail` pane with a `title`, `lines()` and `close()`, or `null` after an action that leaves
`/menu`. `TabContext` carries the `Ui`, `now`, the held inventory or `null`, the `current` conversation,
`attach(conversation)` and `requestRender()`. `TAB_KEYS` has ten keys, 1 to 0, and every one has a tab.

### What the terminal may import

test/boundaries.test.ts holds every file in frontends/ to these rules:

- It imports package files only from frontends/, sdk/, shared/protocol/, shared/view-models/ and shared/render/.
- It never imports the internals behind them: shared/protocol/schema.ts, shared/protocol/actions/methods.ts,
  shared/protocol/actions/standing.ts, or anything in sdk/lib/.
- It imports values only from chord, pi-tui, pi-coding-agent and `node:` modules. Every other package, pi-durable and
  pi-ai among them, is imported for types only. sharp and ws reach the terminal through shared/render/ and sdk/node.ts.
- Everything the terminal runs through value imports stays in those five folders, so it never loads harness or gateway
  code.
- It imports a `lib/` file only from inside that `lib/`'s parent folder.

## Examples

### A component with one key

`KeyHints` draws a heading and three rows, and Space folds the rows away. The module runs it on its own `TuiMainScreen`
until Ctrl-C, as setup.ts runs its screens. `MessageSelection` in conversation/selection.ts has the same shape, with
rows from `render` and keys from `handleInput`.

```ts
import type { Component } from "@earendil-works/pi-tui";
import { ProcessTerminal, TuiMainScreen, matchesKey, truncateToWidth } from "@earendil-works/pi-tui";
import { accent, muted } from "#dimos/agents/frontends/tui/style.ts";

const HINTS = ["Ctrl+O  switch conversations", "Ctrl+T  show or hide the sidebar", "Ctrl-C  detach"];

class KeyHints implements Component {
    private folded = false;
    private readonly changed: () => void;

    constructor(changed: () => void) {
        this.changed = changed;
    }

    handleInput(data: string): void {
        if (!matchesKey(data, "space")) return;
        this.folded = !this.folded;
        this.changed();
    }

    render(width: number): string[] {
        const heading = truncateToWidth(` ${accent(this.folded ? "▸" : "▾")} Keys`, width);
        if (this.folded) return [heading];
        return [heading, ...HINTS.map((hint) => truncateToWidth(muted(`   ${hint}`), width))];
    }

    invalidate(): void {}
}

const tui = new TuiMainScreen(new ProcessTerminal());
const hints = new KeyHints(() => tui.requestRender());
tui.addChild(hints);
tui.setFocus(hints);
tui.addInputListener((data) => {
    if (data !== "\x03") return undefined;
    tui.stop();
    return { consume: true };
});
tui.start();
```

### A tool card

`StreamGetCard` draws a `stream_get` call as the stream it reads, a spinner while the read waits, and then the result's
state and first line. `/expand` shows Pi's generic card through a `PiToolCard` it holds, as `BashCard` in
[cards/bash.ts](/dimos/agents/frontends/tui/cards/bash.ts) does for Pi's bash card. To use it, move it into cards/, import `./base.ts` from there,
and return it from `cardFor` in [cards/choose.ts](/dimos/agents/frontends/tui/cards/choose.ts).

```ts
import type { TUI } from "@earendil-works/pi-tui";
import type { ToolCall } from "@earendil-works/pi-ai";
import type { ToolCard } from "#dimos/agents/frontends/tui/cards/base.ts";
import type { ToolResult } from "#dimos/agents/shared/protocol/conversation/state.ts";
import { Loader, truncateToWidth } from "@earendil-works/pi-tui";
import { PiToolCard } from "#dimos/agents/frontends/tui/cards/pi-tool.ts";
import { accent, amber, muted } from "#dimos/agents/frontends/tui/style.ts";
import { clean } from "#dimos/agents/shared/view-models/text.ts";
import { movedToBackground } from "#dimos/agents/shared/view-models/conversation/tools.ts";

function streamOf(call: ToolCall): string {
    const { module, stream } = call.arguments;
    return typeof module === "string" && typeof stream === "string" ? `${module}.${stream}` : "stream_get";
}

function firstLine(result: ToolResult): string {
    const text = result.content.flatMap((block) => (block.type === "text" ? [block.text] : [])).join("\n");
    return text.split("\n").find((line) => line.trim() !== "") ?? "";
}

export class StreamGetCard implements ToolCard {
    readonly call: ToolCall;
    private readonly loader: Loader;
    private readonly full: PiToolCard;
    private result: ToolResult | null = null;
    private expanded = false;

    constructor(call: ToolCall, tui: TUI, cwd: string) {
        this.call = call;
        this.loader = new Loader(tui, accent, muted, "Waiting for a message");
        this.full = new PiToolCard(call, tui, cwd);
    }

    output(text: string): void {
        this.full.output(text);
    }

    end(result: ToolResult): void {
        this.result = result;
        this.loader.stop();
        this.full.end(result);
    }

    setExpanded(expanded: boolean): void {
        this.expanded = expanded;
        this.full.setExpanded(expanded);
    }

    render(width: number): string[] {
        if (this.expanded) return this.full.render(width);
        const head = truncateToWidth(`${accent("◉ ")}${clean(streamOf(this.call))}`, width);
        const result = this.result;
        if (result === null) return ["", head, ...this.loader.render(width).filter(Boolean)];
        const state = movedToBackground(result)
            ? muted("in the background")
            : result.isError
              ? amber("failed")
              : muted("done");
        return ["", head, truncateToWidth(` ${state}  ${muted(clean(firstLine(result)))}`, width)];
    }

    invalidate(): void {
        this.loader.invalidate();
        this.full.invalidate();
    }

    dispose(): void {
        this.loader.stop();
    }
}
```

### A /menu tab

`busyTab` lists every conversation with a run in progress, the oldest run first, and Enter shows the selected one in the
chat. It reads `ui.gateway.conversations` each time the view renders, as `conversationsTab` in
[menu/conversations.ts](/dimos/agents/frontends/tui/menu/conversations.ts) does. To show it, move it into menu/ and add it to `TABS` in
[menu/view.ts](/dimos/agents/frontends/tui/menu/view.ts). Add a matching key to `TAB_KEYS`.

```ts
import type { MenuRow, MenuTab } from "#dimos/agents/frontends/tui/menu/base.ts";
import { duration } from "#dimos/agents/shared/view-models/time.ts";

const LONG_RUN_MS = 10 * 60 * 1000;

export const busyTab: MenuTab = {
    title: "Busy",
    rows: (context) =>
        context.ui.gateway.conversations.value
            .flatMap((row) => (row.running === null ? [] : [{ row, running: row.running }]))
            .sort((a, b) => a.running.since - b.running.since)
            .map(({ row, running }): MenuRow => {
                const elapsed = context.now - running.since;
                const what = running.kind === "tool" ? running.name : running.kind;
                const mark = row.id === context.current ? "*" : " ";
                return {
                    text: `${mark} #${row.id}  ${what}  ${duration(elapsed)}  ${row.title}`,
                    level: elapsed > LONG_RUN_MS ? "warn" : "ok",
                    keys: [],
                    open: () => {
                        context.attach(row.id);
                        return null;
                    },
                };
            }),
};
```

## Files

The root holds the entry and what it composes. [main.ts](/dimos/agents/frontends/tui/main.ts) is `runTerminal`, the layout, the input modes, the
global keys and the full-screen views. [session.ts](/dimos/agents/frontends/tui/session.ts) is `Ui`. [commands.ts](/dimos/agents/frontends/tui/commands.ts) holds every slash
command and the key list `/help` prints. [setup.ts](/dimos/agents/frontends/tui/setup.ts) is `runSetup`. [dialogs.ts](/dimos/agents/frontends/tui/dialogs.ts) is `LoginPanel`.
[feedback.ts](/dimos/agents/frontends/tui/feedback.ts) is `Feedback`, and feedback.test.ts checks pending sends, refusals, unknown outcomes and a
closed client. [inspect.ts](/dimos/agents/frontends/tui/inspect.ts) is `inspectPath(path, cwd, renderer, progress)`, which renders a file in this
process as the gateway would for a tool result and reports clip progress to a `ProgressPanel`. It throws when the file
is not an image, a point cloud or a frame index, and it only reads the original. [style.ts](/dimos/agents/frontends/tui/style.ts) has `accent`,
`muted`, `amber` and `red`, which colour text, and `levelColor(level, text)`, which cleans text and colours it plain,
amber or red for `ok`, `warn` or `critical`. Its `StatusLine` draws one row with a left text and an optional right text,
read at each render.

[conversation/](/dimos/agents/frontends/tui/conversation/) draws the chat and its controls. [transcript.ts](/dimos/agents/frontends/tui/conversation/transcript.ts) is
`TranscriptView`, the chat. It keeps one component per `TimelineRow` from `timelineOf`, keyed by the row, so a streamed
reply updates one `Markdown` component and a new entry adds one. An input shows by its source, as the user's text,
`⚡ module: text`, `⏰ job: text`, or the end of a background call. A tool result shows its card, with a `Gallery` under
it when the result carries a rendering. A line the terminal adds itself, such as a command's answer or an `/inspect`
rendering, sits after the newest entry it saw. [input.ts](/dimos/agents/frontends/tui/conversation/input.ts) is `PromptEditor`, pi-tui's `Editor`
with command and file completion, an Esc hook and a masked mode, and `ChatInput`, the one-line input of dialogs. A
masked `ChatInput` drops its undo and kill-ring history when it leaves the mode. [switcher.ts](/dimos/agents/frontends/tui/conversation/switcher.ts)
is `SwitcherOverlay`, the Ctrl+O overlay over `switcherRows`. [selection.ts](/dimos/agents/frontends/tui/conversation/selection.ts) is
`MessageSelection` and its `SelectionChoice`, the Esc-then-arrows selection over `selectables`, which starts at the
newest message. [progress.ts](/dimos/agents/frontends/tui/conversation/progress.ts) is `ProgressPanel`, a spinner, a bar and the newest finished
frame while `/inspect` renders a clip. [background.ts](/dimos/agents/frontends/tui/conversation/background.ts) is `BackgroundPanel`, one line under the
editor for the calls running in the background; Down on an empty editor focuses it and lists them all, Enter opens a call's output full
screen, and `x` stops the call. input.test.ts checks that a paste submits whole, that a modified Enter adds a
line, and that a secret never reaches the editor history. progress.test.ts checks that the preview keeps its picture
while the next frame loads. The timeline and selection calculations live in shared/view-models/conversation/.

[cards/](/dimos/agents/frontends/tui/cards/) is the `ToolCard` family in [base.ts](/dimos/agents/frontends/tui/cards/base.ts). The transcript makes one card per call through
`cardFor` and owns it. `BashCard` in bash.ts shows `$ command`, a spinner, the last 12 output lines while the command
runs and the last 3 after. Its label names the newest uv stage line, such as `Installed 16 packages`, once that line has
left the shown lines. A nonzero exit or an error result reads `Failed`. `PiToolCard` in [pi-tool.ts](/dimos/agents/frontends/tui/cards/pi-tool.ts)
is Pi's `ToolExecutionComponent` with Pi's own renderers for `bash`, `read`, `edit` and `write`. Every other tool gets a
one-line card with its name, its state and the first line of output until `/expand`. Pi draws result images itself
except on Warp, where `needsImageSlices()` turns them off. bash.test.ts runs a real child process through a `BashCard`.

[views/](/dimos/agents/frontends/tui/views/) is the `ViewPanel` family in [base.ts](/dimos/agents/frontends/tui/views/base.ts), one panel per renderer in shared/render/.
`Gallery` in [gallery.ts](/dimos/agents/frontends/tui/views/gallery.ts) is one rendering under a card or an event line. It shows a mosaic of the
rendering's views, at most `MAX_VIEWS`, which is 6. A tab row numbers the views, and a click on a tab or `/panel N`
opens view N in its panel. `/panel 0` returns to the overview. `/panel N` opens a panel that takes keys, such as
`SpatialPanel`, full screen so it gets them. The gallery lists files past the limit as `Also:` and failed files as
`Not rendered:`. `ImagePanel` in [image.ts](/dimos/agents/frontends/tui/views/image.ts) shows the cached preview. `SpatialPanel` in
[points.ts](/dimos/agents/frontends/tui/views/points.ts) shows a point cloud. Left, Right and `r` rotate it, `+` and `-` zoom between 0.25 and 4,
`g` switches between graphics and Braille, `d` shows the source path and SHA-256, and `0` resets. It repaints with
`paintCloud` from shared/render/points.ts. `ClipPanel` in [clip.ts](/dimos/agents/frontends/tui/views/clip.ts) loops a clip's saved frame PNGs at
the source timing and never plays the GIF. clip.test.ts checks that playback never blanks between frames.

[sidebar/](/dimos/agents/frontends/tui/sidebar/) draws the stream sidebar. [sidebar.ts](/dimos/agents/frontends/tui/sidebar/sidebar.ts) is `Sidebar`, `SidebarSource`, and
`SIDEBAR_COLUMNS`, which is 40 cells with the `│ ` gutter. main.ts places `sidebar.column`, the gutter beside a
`ScrollView`, in the layout. It passes a function that returns the shown conversation's `SidebarSource` or `null`, and
one that returns the screen's rows, so the gutter reaches the bottom of the screen. The column lists the
conversation's agent modules with their streams, its jobs with their next due time and last three runs, and its
admitted notices in the input queue. Below them it lists the system's picture streams the conversation does
not show. `sidebarGroups` and `systemGroups` from shared/view-models/streams/ build the groups. Each render collects the
channels of unfolded cards and passes that set to the `FrameSource` with `want`, and a closed or hidden sidebar wants
nothing. Each SDK rate sample replaces the displayed `RateTable` through `ratesOf`. A picture frame goes to the painter one at a time per channel,
and a frame that arrives while one paints is dropped. `StreamCardView` in [card.ts](/dimos/agents/frontends/tui/sidebar/card.ts) is one stream's
title row with its rate, then a picture slot or the newest value. Each card keeps one kitty image id, so a new frame
replaces the picture in place. `FramePainter` in [painter.ts](/dimos/agents/frontends/tui/sidebar/painter.ts) turns an image, cloud or grid record
into a PNG with shared/render/frames.ts. The frames come from `RemoteFrames` in
[sdk/streams/remote.ts](/dimos/agents/sdk/streams/remote.ts). It opens a Web SDK session from the current inventory relay
endpoint and subscribes to the exact manifest channel IDs. The gateway updates the catalog from live manifests.
Preview types come from encodings; channels without module annotations appear under Relay. Closing a preview stops its SDK read.
Any other `FrameSource` can replace it.
sidebar.test.ts drives a `Sidebar` through a real `TuiAltScreen` with an in-memory terminal and frame source.

[menu/](/dimos/agents/frontends/tui/menu/) is the main menu that `/menu` opens, the `MenuTab` family in [base.ts](/dimos/agents/frontends/tui/menu/base.ts). `MenuView` in
view.ts is the full-screen view main.ts opens. Keys 1 to 9 pick a tab, Up and Down pick a row, Enter opens the row's
detail, a row's own keys run its actions, and Esc closes the detail and then the view. A tab longer than the screen
shows the rows around the selected one. A failed key action lands in `Feedback` as a refusal.

| Key | Tab           | File                                         | Rows                                                                                                  |
| --- | ------------- | -------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| 1   | Overview      | [overview.ts](/dimos/agents/frontends/tui/menu/overview.ts)           | This terminal's link, the dimOS link, load and run, the five newest refused actions. |
| 2   | Conversations | conversations.ts                             | Every conversation in switcher order. Enter shows it in the chat.                                     |
| 3   | Agent modules | [modules.ts](/dimos/agents/frontends/tui/menu/modules.ts)             | Every agent module and its owner. `r` restarts, `u` unloads, `d` deletes.   |
| 4   | Jobs          | [jobs.ts](/dimos/agents/frontends/tui/menu/jobs.ts)                   | Every job as a lane over the last and next 12 hours. `r` runs it now, `d` deletes, Enter lists runs.  |
| 5   | Streams       | [streams.ts](/dimos/agents/frontends/tui/menu/streams.ts)             | Every dimOS stream, grouped by module instance.                                                       |
| 6   | Env           | [inventory.ts](/dimos/agents/frontends/tui/menu/inventory.ts)         | The dimOS checkout, Python, transport and system size.                                                |
| 7   | Modules       | inventory.ts                                 | Every module instance. Enter lists its streams and RPCs.                                              |
| 8   | RPCs          | inventory.ts                                 | Every RPC with its signature and doc.                                                                 |
| 9   | Stream types  | inventory.ts                                 | Every stream with its direction, type and channel.                                                    |

Tabs 6 to 9 read the inventory held by the terminal. [menu/lib/](/dimos/agents/frontends/tui/menu/lib/) is private to menu/.
`plain(text, level)` in [rows.ts](/dimos/agents/frontends/tui/menu/lib/rows.ts) is a row with no keys and no Enter.
[lane.ts](/dimos/agents/frontends/tui/menu/lib/lane.ts) has `laneText`, which draws a job's runs, due times and now on a ruler of a given number
of cells, and `axisText`, the clock labels above it. The Jobs tab uses 48 cells, one per 30 minutes. lane.test.ts checks
both strings.

[lib/](/dimos/agents/frontends/tui/lib/) is private to the terminal. `TerminalImage(image, size, changed, imageId?)` in terminal-image.ts is a
component for one picture. It takes a PNG or any format sharp reads, such as the JPEG the model was sent. A terminal
with no graphics protocol gets the picture in text cells. They are half blocks in truecolor, 256 colours or 8 colours,
or an ASCII ramp under `NO_COLOR`, `TERM=dumb` or an unknown `TERM`. Warp with kitty graphics gets one image per cell
row, because Warp ignores source-rectangle cropping and whole-row images let a `ScrollView` clip rows. Every other
terminal gets pi-tui's `Image`. `setPng(png)` swaps the picture and keeps its image id, and `close()` stops pending
work. `needsImageSlices(env?)` says whether the terminal is Warp. The progress panel, the view panels, the sidebar cards
and `PiToolCard` use it. terminal-image.test.ts covers each path. Image preparation shared with the harness lives in
shared/render/.

## See also

- The [client SDK](/docs/usage/agent/sdk.md) covers `connect`, `listen`, `owner`, `retryByRequestId` and the frame sources.
- The [view models](/docs/development/agent/view-models.md) build the rows the terminal draws.
- The [protocol](/docs/development/agent/protocol.md) defines the contracts and actions the terminal calls.
- [Media rendering](/docs/development/agent/render.md) prepares every preview the panels show.
- The [CLI](/docs/usage/agent/cli.md) explains how `dimos agent` reaches `runTerminal`.
