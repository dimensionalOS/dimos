# Frontend kit for the Pi session viewer

Research date: 2026-07-23. Sources are limited to first-party documentation,
repositories, and npm metadata.

## Recommendation

Build a small repo-owned React viewer with:

- shadcn/ui primitives (Base UI flavor) for the shell, sidebar, badges,
  collapsibles, tooltips, and accessible interactions;
- selectively adapted AI Elements source for the visual treatment of messages,
  reasoning, and tool calls;
- Streamdown for assistant Markdown; and
- a purpose-built, immutable `PiSessionViewModel` produced from native Pi v3
  JSONL.

Do **not** adopt assistant-ui's runtime or the Vercel AI SDK runtime. They solve
live chat lifecycle problems that this viewer intentionally does not have. Do
not install AI Elements wholesale either: its supported setup assumes Next.js,
the AI SDK, React 19, Tailwind 4, and shadcn/ui
([AI Elements setup](https://elements.ai-sdk.dev/docs/setup)). Its copied-source
components are still the best design reference because they already cover
messages, reasoning, tools, and branch selectors, and their Apache-2.0 license
permits adaptation
([component catalog](https://elements.ai-sdk.dev/),
[license](https://github.com/vercel/ai-elements/blob/main/LICENSE)).

This is a customized solution, but not a from-scratch design system. It keeps
the session semantics and read-only guarantee under our control while reusing
polished, accessible presentation building blocks.

## Comparison

| Candidate | Custom JSONL without an agent runtime | Pi content fit | Strictly read-only | Offline/bundled | Maintenance and maturity | Verdict |
|---|---|---|---|---|---|---|
| **assistant-ui 0.14.27** | Possible through `ExternalStoreRuntime`, where the application owns the messages; however, the runtime still models composers, run lifecycle, editing, regeneration, and branches ([runtime architecture](https://www.assistant-ui.com/docs/runtimes/concepts/architecture)). | First-class message parts include reasoning and tool calls, and it has branch navigation ([message parts](https://www.assistant-ui.com/docs/primitives/message), [branching](https://www.assistant-ui.com/docs/guides/branching)). Pi usage and timestamps would remain custom. | A custom composition can omit composer/action primitives, and callback-driven features only turn on when supplied. The package itself is not a read-only viewer: its public runtime also exposes editing, reloading, copying, rating, speaking, and branching actions ([runtime API](https://www.assistant-ui.com/docs/api-reference/runtimes)). | Bundler-friendly React package; no cloud service is required for the core MIT-licensed library ([npm metadata](https://www.npmjs.com/package/%40assistant-ui/react), [license](https://github.com/assistant-ui/assistant-ui/blob/main/LICENSE)). | Mature and active: 0.14.27, 426 published versions and about 1.2M weekly downloads when checked ([npm metadata](https://www.npmjs.com/package/%40assistant-ui/react)). Its runtime and 19 direct dependencies are substantial for a static transcript ([package metadata](https://raw.githubusercontent.com/assistant-ui/assistant-ui/main/packages/react/package.json)). | **Reject for this viewer.** Strongest full chat framework, but mapping Pi's immutable tree into a mutable chat runtime adds complexity and risk without product value. |
| **Vercel AI Elements 1.9.0** | Individual copied-source components can receive ordinary props/children, but the officially supported setup requires a Next.js project with AI SDK and shadcn/ui ([setup](https://elements.ai-sdk.dev/docs/setup)). | Excellent visual coverage: messages have response and branch components; reasoning is collapsible; tools show input, output, errors, and status ([message](https://elements.ai-sdk.dev/components/message), [reasoning](https://elements.ai-sdk.dev/components/reasoning), [tool](https://elements.ai-sdk.dev/components/tool)). Usage and timestamps remain custom. | Easy to make read-only by copying only display components and never installing prompt, action, checkpoint-restore, or download controls. Components are composable source copied into the app ([project README](https://github.com/vercel/ai-elements)). | Once copied and built, assets are local. Installation itself fetches registry source and dependencies; runtime services and keys are unnecessary if no AI SDK hooks are used. This is an inference from its copied-source installation model ([installation docs](https://elements.ai-sdk.dev/docs)). | Active but younger: CLI 1.9.0, 29 versions and about 36K weekly downloads when checked; Apache-2.0 ([npm metadata](https://www.npmjs.com/package/ai-elements), [license](https://github.com/vercel/ai-elements/blob/main/LICENSE)). | **Use as a selective design donor, not as the application architecture.** Adapt `message`, `reasoning`, and `tool`; replace AI SDK types with our view-model types. |
| **shadcn/ui + Base UI primitives** | No data model or runtime is imposed. shadcn copies component source into the repository, where it can be freely customized ([shadcn introduction](https://ui.shadcn.com/docs), [installation model](https://ui.shadcn.com/docs/new)). | Provides the structural pieces, not agent semantics. We implement message rows, the branch rail, usage cards, and timestamps; Collapsible, Sidebar, Tooltip, Badge, Scroll Area, and Tabs supply the interaction foundation ([component catalog](https://ui.shadcn.com/docs/components)). | Best fit: only the controls we add exist. Base UI is an unstyled component library with control over CSS and accessibility behavior ([Base UI npm metadata](https://www.npmjs.com/package/%40base-ui/react)). | Copied source and npm dependencies bundle into local static assets. No CDN or hosted runtime is part of the documented model ([shadcn introduction](https://ui.shadcn.com/docs)). | shadcn CLI 4.14.1 and Base UI 1.6.0 were current when checked; both are MIT licensed ([shadcn package](https://raw.githubusercontent.com/shadcn-ui/ui/main/packages/shadcn/package.json), [Base UI npm metadata](https://www.npmjs.com/package/%40base-ui/react), [shadcn license](https://github.com/shadcn-ui/ui/blob/main/LICENSE.md)). shadcn now defaults new projects to Base UI while retaining Radix support ([July 2026 announcement](https://ui.shadcn.com/docs/changelog/2026-07-base-ui-default)). | **Choose as the foundation.** Lowest semantic coupling and highest control over the read-only contract. |
| **Streamdown 2.5.0** | Accepts a Markdown string; it has no chat or agent runtime ([README](https://github.com/vercel/streamdown)). | Handles assistant prose, GFM, code highlighting, math, and optional diagrams, but not messages, tools, usage, timestamps, or branches ([features](https://github.com/vercel/streamdown#features)). | Display-only except optional local UI controls such as copy/render; these can be disabled or omitted. It uses hardened Markdown rendering ([README](https://github.com/vercel/streamdown)). | ESM package and CSS are bundled locally. Tailwind must scan its distributed source and shadcn-style CSS variables must be defined ([installation](https://github.com/vercel/streamdown#installation)). | 2.5.0, about 4.5M weekly downloads when checked, Apache-2.0 ([npm metadata](https://www.npmjs.com/package/streamdown), [package metadata](https://raw.githubusercontent.com/vercel/streamdown/main/packages/streamdown/package.json), [license](https://github.com/vercel/streamdown/blob/main/LICENSE)). | **Choose for Markdown only.** It gives polished code and rich text without contaminating the session model. |
| **react-markdown 10.1.0** | Accepts a Markdown string and custom React component mappings; no runtime ([README](https://github.com/remarkjs/react-markdown)). | Safe CommonMark is built in and GFM is available through `remark-gfm`; syntax highlighting requires a separate renderer ([usage and examples](https://github.com/remarkjs/react-markdown#use)). It does not model agent events. | Display-only and secure by default, provided unsafe URL transforms/plugins are not added ([security](https://github.com/remarkjs/react-markdown#security)). | ESM works with modern bundlers and browsers ([compatibility](https://github.com/remarkjs/react-markdown#compatibility)). | Very mature: 10.1.0 and about 20.7M weekly downloads when checked; MIT ([npm metadata](https://www.npmjs.com/package/react-markdown), [license](https://github.com/remarkjs/react-markdown/blob/main/license)). | **Fallback.** Prefer it if bundle size or Streamdown's Tailwind coupling becomes a problem; otherwise Streamdown produces the nicer transcript with less custom renderer work. |

The new `@assistant-ui/react-pi` is not a shortcut for this requirement. Version
0.0.7 is explicitly a **live** Pi runtime: it drives sessions, messages,
steering/follow-ups, model and thinking controls, approvals, archive/delete, and
a server-side Pi supervisor. Its documented HTTP contract contains POST, PATCH,
and DELETE routes and requires model credentials for live sessions
([official npm documentation](https://www.npmjs.com/package/%40assistant-ui/react-pi)).
That is almost the inverse of a credential-free evidence viewer, so it should
not sit behind a nominally read-only wrapper.

## Proposed UI

The viewer should be a document inspector, not a disabled chat application:

1. A compact top bar shows session ID, complete/partial status, model, elapsed
   time, token totals, and the explicit **read-only** state.
2. A collapsible left rail shows the Pi parent/child entry tree. Selecting a
   branch changes the visible ancestry path but never alters the session.
3. The center transcript renders user and assistant turns with restrained role
   treatment. Thinking blocks are visually quieter and collapsed by default.
4. Tool calls appear as accessible collapsibles with name/status in the header,
   formatted JSON arguments, result/error content, and duration when available.
5. A details drawer shows exact timestamps, per-message usage, IDs, parent IDs,
   and raw JSON for forensic inspection.
6. No composer, regenerate/edit buttons, approvals, restore controls, network
   status, or session-management actions exist in the component tree.

AI Elements already demonstrates the desired message, branch, reasoning, and
tool presentation patterns
([message](https://elements.ai-sdk.dev/components/message),
[reasoning](https://elements.ai-sdk.dev/components/reasoning),
[tool](https://elements.ai-sdk.dev/components/tool)). The Pi branch rail and
usage/timestamp details should remain custom because neither AI Elements nor
assistant-ui represents Pi's entry-level parent graph and evidence metadata
directly.

## Integration shape

Create a small Vite/React build next to the Pi adapter, not inside the existing
robot-facing Svelte UI. The adapter already requires Node 22 and TypeScript but
has no React dependency or browser build
([Pi adapter package](/packages/pi-spatial-adapter/package.json)); the
existing web interface is a separate Svelte/Vite application
([web interface package](/dimos/web/dimos_interface/package.json)).

Keep the boundary deliberately narrow:

```text
native Pi v3 JSONL
        |
        v
validated parser -> immutable PiSessionViewModel
        |
        +-- session summary
        +-- entry graph / selected ancestry
        +-- message parts (text, thinking, tool call/result)
        +-- usage and timestamps
        |
        v
React display components -> prebuilt local JS/CSS assets
```

The frontend should receive one already-admitted session document from the
loopback launcher. It should have no write endpoint and no client code capable
of changing the source. Bundle fonts, icons, JS, and CSS; do not use remote
fonts, CDNs, analytics, gateway keys, or model APIs. For rendered Markdown,
restrict or neutralize remote images and unsafe links even though Streamdown
uses hardened rendering, because an offline evidence viewer should not produce
incidental network requests
([Streamdown security feature](https://github.com/vercel/streamdown#features)).

## Suggested implementation pins

Run a build spike before committing the lockfile, then pin the exact compatible
set. The source versions observed during this research were:

- shadcn CLI `4.14.1` (development-time source installer only)
  ([package metadata](https://raw.githubusercontent.com/shadcn-ui/ui/main/packages/shadcn/package.json));
- Base UI `1.6.0`
  ([npm metadata](https://www.npmjs.com/package/%40base-ui/react));
- AI Elements `1.9.0` as the source/tag from which selected display components
  are adapted ([npm metadata](https://www.npmjs.com/package/ai-elements)); and
- Streamdown `2.5.0`
  ([package metadata](https://raw.githubusercontent.com/vercel/streamdown/main/packages/streamdown/package.json)).

Record copied/adapted component provenance and retain the required MIT and
Apache-2.0 notices. Pinning copied component source is naturally achieved by
committing it; future upgrades should be explicit diffs, not automatic registry
refreshes. In particular, pinning `ai-elements@1.9.0` does **not** pin generated
output: that CLI invokes `shadcn@latest` and fetches component JSON from the live
AI Elements registry
([CLI source](https://raw.githubusercontent.com/vercel/ai-elements/main/packages/cli/index.js)).
Commit the selected generated/adapted files and record their upstream tag or
commit; do not run the generator during ordinary builds.
