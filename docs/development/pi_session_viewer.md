# Pi session history viewer

The Pi session history viewer is a private, read-only browser view of one
retained native Pi session. It improves human review without replacing native
JSONL or the pinned Pi HTML export verification.

## Open one session

Run the command against either an attempt directory containing `private/` or
the terminal private mode directory containing the native-session receipt:

```sh
pi-baseline session view <attempt-directory>
```

The command prints a capability-bearing `http://127.0.0.1:<port>/<token>/`
URL and makes a best-effort attempt to open it in the default browser. Open the
printed URL manually if browser launch is unavailable. Press `Ctrl-C` in the
terminal to stop the server and remove the temporary review files.

The command requires no experiment execution, scorer, oracle, public-root, or
model authentication arguments.

## What the UI shows

The header identifies complete or partial evidence and summarizes model,
tokens, branches, and start time. The branch rail selects any native entry and
shows its ancestry. The transcript presents messages, collapsed thinking,
tool calls and results, state changes, timestamps, and usage. “Details” opens
IDs, metadata, usage, and a scrubbed raw native entry.

Partial evidence is clearly labeled and includes only the valid retained
prefix. An incomplete final JSONL fragment is never sent to the browser.
Embedded image bytes and provider reasoning signatures are omitted. Remote
Markdown links and images are disabled.

## Read-only and privacy boundary

Before launch, the command:

1. pins the attempt and private directories without following symlinks;
2. admits the native session through its existing safe-path, v3 validation,
   prompt-binding, receipt, size, and SHA-256 checks;
3. copies only byte-identical native JSONL into a mode-`0700` temporary
   directory; and
4. derives one immutable browser document without copying prompt sidecars,
   scores, predictions, logs, credentials, or policy evidence.

The attached server binds only `127.0.0.1` on an ephemeral port. An unguessable
path token scopes every route. Only the application shell, packaged local
assets, and immutable session document accept `GET` or `HEAD`; other methods
and paths fail closed. Responses disable caching and referrers and apply a
restrictive Content Security Policy.

The browser application has no composer, mutation action, upload, WebSocket,
agent runtime, model credential, analytics, proxy, or outbound-network
feature. Branch selection and detail expansion change browser-local
presentation state only.

On shutdown the command rechecks the canonical session digest and removes all
temporary files. A viewer failure never changes attempt state or publication
eligibility.

## Frontend provenance

The source-owned frontend is in `packages/pi-session-viewer`. Exact React,
Vite, Tailwind, Base UI, Streamdown, Lucide, TypeScript, and test versions are
recorded in its `package.json` and lockfile. The message, reasoning, and tool
presentation patterns are locally adapted from Vercel AI Elements `1.9.0`
(Apache-2.0). Base UI `1.6.0` is MIT licensed, Streamdown `2.5.0` is
Apache-2.0, and Lucide `1.26.0` is ISC licensed. `THIRD_PARTY.md` records the
source details.

Production serves prebuilt assets packaged under
`dimos/benchmark/spatial/pi_baseline/viewer_assets`; it does not run Node,
download fonts, contact a component registry, or fetch a CDN at review time.

## Compatibility and export authority

The viewer keeps `@earendil-works/pi-coding-agent` pinned at `0.80.10` and
native session format v3. The ADR 0003 compatibility suite remains the upgrade
gate for create, reopen, dispose, complete and partial validation, receipt and
prompt binding, and pinned CLI export behavior.

The existing verified command remains authoritative:

```sh
pi --export INPUT.jsonl OUTPUT.html
```

That pinned export still gates session-backed publication. Viewer availability,
startup, or successful rendering cannot replace export verification, publish a
prediction or score, or repair invalid evidence.
