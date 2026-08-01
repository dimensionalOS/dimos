## Context

The benchmark retains native Pi v3 JSONL as canonical conversation evidence and uses the pinned Pi CLI HTML export to verify inspectability. That export remains a machine gate, but it is cumbersome for human review. The viewer must improve presentation without changing the evidence boundary established by ADR 0003.

An initial Pi Web integration was rejected. Pi Web is a live agent workspace with mutation routes, and its supported extension points cannot remove core controls or add server-side read-only enforcement. Frontend research instead recommends a small display-only application: shadcn/Base UI for accessible structure, selected AI Elements presentation patterns for messages, reasoning, and tools, and Streamdown for Markdown. The full comparison and primary-source evidence are recorded in `docs/research/pi-session-viewer-frontend-kit.md`.

## Goals / Non-Goals

**Goals:**

- Open one retained Pi session from an attempt directory in a polished local browser UI.
- Preserve Pi-native messages, thinking, tool activity, usage metadata, state changes, timestamps, and branches.
- Keep canonical session bytes immutable and private.
- Make the viewer structurally read-only: no mutation components, client actions, or server routes exist.
- Ship all browser assets locally with exact dependency pins and recorded component provenance.
- Preserve Pi `0.80.10` and the existing pinned CLI export as the publication compatibility gate.

**Non-Goals:**

- No experiment dashboard, session search, multi-session navigation, or persistent service.
- No benchmark sidecars, prompts, predictions, scores, logs, or policy audits in the UI.
- No session continuation, prompting, editing, forking, renaming, archiving, deletion, or restoration.
- No public or remote viewer, shared URL, retained viewer database, or canonical HTML.
- No live agent runtime, Vercel AI SDK runtime, assistant-ui runtime, Pi Web, CDN, analytics, or model-provider integration.

## Decisions

### Adopt a display kit without adopting a chat runtime

Create a private Vite/React frontend package next to the Pi adapter. Use shadcn's Base UI flavor for the shell, sidebar, badges, collapsibles, tooltips, and accessible interactions. Adapt only the display portions of AI Elements' message, reasoning, and tool components. Use Streamdown for assistant Markdown.

The build spike starts from the researched versions: shadcn CLI `4.14.1` for development-time source installation, Base UI `1.6.0`, AI Elements `1.9.0` as source provenance, and Streamdown `2.5.0`. It shall select and lock an exact compatible React/Vite/Tailwind set before implementation proceeds.

AI Elements' CLI is not reproducible by version alone because it invokes `shadcn@latest` and reads a live component registry. Therefore, adapted component files become repo-owned source. Record their upstream tag or commit and licenses, pin all runtime dependencies exactly, and never run a live registry generator during ordinary builds.

**Alternatives considered:** assistant-ui offers a mature external-store runtime and branch primitives, but it still models live chat actions and adds substantial state machinery. The existing Svelte interface avoids a second framework but is an older terminal UI without agent transcript components. Plain shadcn alone is viable but requires recreating established message, reasoning, and tool presentation.

### Translate native JSONL into an immutable view model

The Python boundary admits and parses one native session through existing validation. It creates an immutable `PiSessionViewModel` containing:

- session identity, status, model, timestamps, totals, and partial-state metadata;
- the entry parent/child graph and selected ancestry path;
- typed message parts for text, thinking, tool calls, tool results, and state changes;
- per-entry usage, timing, IDs, and safe raw-entry detail.

The browser receives this model as a single local document. It never reads the attempt directory and does not import Pi or an agent runtime. Selecting a branch changes browser-local presentation state only.

**Alternative considered:** pass raw JSONL to the browser and duplicate validation in TypeScript. That widens the trusted parsing surface and risks disagreement with the canonical Python validator.

### Present the session as a document inspector

The UI uses three responsive regions:

1. A compact header shows complete/partial status, model, elapsed time, token totals, and an explicit read-only badge.
2. A collapsible branch rail shows the Pi parent/child entry tree and changes the visible ancestry path.
3. The transcript shows restrained user and assistant turns, collapsed thinking, structured tool cards, and an optional details drawer for timestamps, usage, IDs, and raw entry JSON.

No prompt input, response action, approval, checkpoint, download, or session-management component is included. Remote images and unsafe links in Markdown are blocked or neutralized. Fonts, icons, JavaScript, and CSS are bundled locally.

**Alternative considered:** render branches as a node-canvas graph. A tree rail preserves the native structure with less interaction complexity and better keyboard and narrow-screen behavior.

### Serve a capability-scoped, GET-only local application

`pi-baseline session view <attempt-directory>` creates a mode-`0700` temporary review root and copies only the byte-identical native JSONL. It derives the immutable view-model document there, records the canonical source digest, and serves prebuilt assets plus that document from a repo-owned Python server.

The server binds `127.0.0.1` on an ephemeral port and places an unguessable capability token in the URL path. It allowlists only the token-scoped application shell, static assets, and session document. Every unsupported path and every non-GET/HEAD method fails closed. Responses use a restrictive Content Security Policy, `no-store`, no-referrer, and content-type hardening. The server has no mutation handler, WebSocket, upload, proxy, or outbound-network feature.

**Alternative considered:** run a Node development or application server. Prebuilt assets and a small Python server remove a production Node process, reduce routes, and keep lifecycle management inside the existing CLI.

### Preserve evidence and export authority

The viewer never receives writable access to the canonical attempt directory. The command verifies the canonical digest before launch and after shutdown, remains attached until interrupted, shuts down within a bounded interval, and removes temporary state in a `finally` path.

The existing pinned `pi --export INPUT.jsonl OUTPUT.html` path remains part of session-gated publication. Viewer startup or rendering never changes attempt status or authorizes prediction and score publication.

## Risks / Trade-offs

- **[Risk] Adapted component source drifts from upstream or loses required notices.** → Record source tag/commit per adapted file, retain MIT and Apache-2.0 notices, and update only through explicit reviewed diffs.
- **[Risk] Markdown causes external requests or unsafe markup.** → Deny remote images, restrict links, omit unsafe plugins, apply a restrictive CSP, and test hostile content.
- **[Risk] The view model loses a Pi entry type or branch relationship.** → Use exhaustive conversion, representative complete and partial fixtures, raw-entry details, and graph invariants.
- **[Risk] Committed frontend assets become stale relative to source.** → Rebuild in CI and compare output or manifest digests; never fetch registry content during builds.
- **[Risk] The staged session or view model outlives the command.** → Use a private temporary root, bounded shutdown, unconditional cleanup, and exit-path tests.
- **[Trade-off] A repo-owned viewer requires modest frontend maintenance.** → Keep it display-only, adapt a small component subset, and avoid a general chat runtime.

## Migration Plan

1. Run a frontend fit spike with complete and partial Pi fixtures, then lock exact runtime dependencies and component provenance.
2. Add the private viewer package, immutable view model, and deterministic local asset build.
3. Add the GET-only loopback server, admission, staging, lifecycle, and CLI command.
4. Verify rendering, accessibility, offline behavior, mutation absence, source hashes, export behavior, and cleanup.
5. Roll back by removing the viewer command, frontend package, and packaged assets. The unchanged Pi pin and retained v3 JSONL remain canonical throughout.

## Open Questions

No product decisions remain. The implementation spike may choose exact React, Vite, and Tailwind patch versions, but it must preserve the source-owned, offline, display-only boundary above.
