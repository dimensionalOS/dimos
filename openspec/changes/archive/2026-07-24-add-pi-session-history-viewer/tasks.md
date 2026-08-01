## 1. Prove the Frontend Fit

- [x] 1.1 Create an isolated Vite/React spike with exact candidate dependencies, shadcn/Base UI primitives, AI Elements `1.9.0` display-source provenance, and Streamdown `2.5.0`; record licenses, upstream tags or commits, and all copied adaptations.
- [x] 1.2 Map representative complete and valid partial Pi v3 fixtures into an immutable view model and prove messages, thinking, tool activity, usage data, state changes, timestamps, and branches render without a Pi or chat runtime.
- [x] 1.3 Prove the built frontend contains no composer, mutation controls, live-agent hooks, model credentials, external assets, analytics, outbound requests, or dynamic registry dependency.
- [x] 1.4 Run the ADR 0003 compatibility checklist against unchanged Pi `0.80.10`, including create, reopen, dispose, complete/partial validation, receipt, prompt binding, and pinned CLI export behavior; stop and report any go/no-go failure.

## 2. Build the Source-Owned Viewer

- [x] 2.1 Add the private viewer package with exact React, Vite, Tailwind, Base UI, Streamdown, icon, and test dependencies; commit a reproducible lockfile and deterministic asset build.
- [x] 2.2 Implement exhaustive native-session-to-`PiSessionViewModel` conversion with session summary, partial status, entry graph, selected ancestry, typed content parts, usage, timestamps, IDs, and safe raw-entry detail.
- [x] 2.3 Build the responsive document-inspector shell with a read-only status header, collapsible branch rail, transcript, and details drawer.
- [x] 2.4 Adapt the selected AI Elements message, reasoning, and tool display components to local view-model types, retaining required provenance and license notices.
- [x] 2.5 Implement native branch selection, complete/partial indicators, tool status and results, usage and timing summaries, and forensic entry details without mutation actions.
- [x] 2.6 Apply accessible keyboard behavior, focus states, semantic labels, responsive layouts, light/dark styling, and reduced-motion support.
- [x] 2.7 Bundle all fonts, icons, JavaScript, and CSS locally; harden Markdown links and images; add a restrictive CSP; and verify builds perform no live registry or network fetch beyond locked dependency installation.

## 3. Build the Private Read-Only Boundary

- [x] 3.1 Implement attempt-directory admission that resolves the native session and receipt through existing safe-path checks, accepts complete or valid partial evidence, and rejects unavailable, malformed, unsafe, or mismatched evidence before launch.
- [x] 3.2 Implement mode-`0700` disposable staging that copies only byte-identical native JSONL, derives the immutable view-model document, records the canonical digest, never follows symlinks, and never copies sidecars.
- [x] 3.3 Implement the capability-tokened Python server on `127.0.0.1` and an ephemeral port with a strict GET/HEAD route allowlist, local assets, hardened response headers, and rejection of every non-read method and unknown route.
- [x] 3.4 Implement attached lifecycle management, printed local URL, best-effort browser launch, bounded shutdown, post-review canonical digest verification, and unconditional temporary-root cleanup.

## 4. Add the Operator Command

- [x] 4.1 Add `pi-baseline session view <attempt-directory>` to the CLI without requiring experiment execution, scorer, oracle, public-root, or authentication arguments.
- [x] 4.2 Add bounded, content-free operator errors for invalid evidence, incompatible or missing viewer assets, bind/startup failure, server failure, cleanup failure, and canonical source mutation.
- [x] 4.3 Document the single-session workflow, UI layout, read-only and loopback guarantees, partial-session behavior, shutdown procedure, privacy boundary, frontend provenance, and continued authority of pinned HTML export verification.

## 5. Test the Review Contract

- [x] 5.1 Add admission and view-model tests for complete, partial, unavailable, malformed, receipt-mismatched, path-escaping, symlinked, oversized, sidecar-bearing, branched, and every supported native entry type.
- [x] 5.2 Add component and accessibility tests proving native messages, thinking, tools, usage, state changes, timestamps, branches, and details render while composers, mutation controls, sidecars, unsafe links, and remote images remain absent.
- [x] 5.3 Add HTTP and asset tests proving capability-token enforcement, loopback-only ephemeral binding, GET/HEAD-only routes, hardened headers, exact asset provenance, offline operation, and absent credentials or outbound requests.
- [x] 5.4 Add lifecycle tests for normal exit, interruption, startup failure, server failure, bounded shutdown, staged-copy mutation, canonical-source mutation, and cleanup of every temporary path.
- [x] 5.5 Run end-to-end complete and partial reviews plus pinned export, session-gated publication, privacy, CLI, frontend build, TypeScript, Python typing, and targeted pytest suites; confirm viewer success or failure cannot affect canonical evidence or publication.
