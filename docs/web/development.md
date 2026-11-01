# Development

The web code is one Deno workspace under `web/` with four packages, plus the Python side under `dimos/web/`. Everything runs on Deno 2.6.10, pinned in [`dimos/utils/deno.py`](/dimos/utils/deno.py) (CI reads the pin from there). There is no node and no npm: vite, vitest and tsc run as npm packages under Deno (`nodeModulesDir: auto`), and `dimos run --local-relay` downloads Deno by itself when none is on `PATH`.

| Directory | What it is |
|---|---|
| `web/shared/` | the wire protocol and the manifest (`protocol.ts`, `manifest.ts`) with their golden fixtures |
| `web/relay/` | the relay: `main.ts` (the CLI), `server.ts` (the HTTP and WebTransport listeners), `session.ts`, `registry.ts`, `forward.ts`, `carrier.ts`, `auth.ts`, `cert.ts` |
| `web/sdk/` | `@dimos/sdk`: transport, session, stores, decoders and React hooks, plus `fixture/`, a Vite consumer |
| `web/cockpit/` | the cockpit app (Vite, React, TypeScript), built on the SDK |
| `web/examples/` | three zero-build SDK pages |
| `dimos/web/cockpit.py`, `dimos/web/codecs.py`, `dimos/web/lcm_codec.py` | the Python authoring API and the codec registry |
| `dimos/web/relay_bridge/` | the bridge module, the Python mirrors of the protocol and the manifest, the WebTransport client, the local relay process, and the fixture generators |

`dimos/web/` also holds two older web servers that have nothing to do with this stack: `websocket_vis/` (the socket.io visualizer on port 7779) and `dimos_interface/` with `robot_web_interface.py` (the FastAPI page on port 5555 used by `WebInput` and the phone and WebXR teleops).

## Tasks

From `web/`:

```bash
deno task dev            # relay on http://127.0.0.1:7780, a server (flags on the relay page)
deno task test           # relay + shared tests (unit + loopback e2e)
deno task check          # type-check relay + shared
deno fmt && deno lint    # style, all of web/
deno task -r build       # build the SDK bundle and the cockpit
```

The servers (`dev`, `fixture`) keep their terminal, so run them in a terminal of their own.

From `web/sdk/`:

```bash
deno task test           # vitest
deno task check          # tsc --noEmit
deno task build          # dist/sdk.js, the bundle the relay serves at /sdk.js
deno task fixture        # demo consumer on http://localhost:5174, a server (run a relay first)
```

From `web/cockpit/`:

```bash
deno task dev            # vite dev server on http://localhost:5173 with HMR, a server
deno task test           # vitest
deno task check          # tsc --noEmit
deno task build          # dist/, what the relay serves at /
```

## Dev loop

Run a relay (`deno task dev` in `web/`, or `dimos run <bp> --local-relay`) and the vite server side by side. `localhost:5173` is a secure context. Vite proxies `/api` to the relay on 7780, and the WebTransport connection goes straight to the advertised address. The SDK fixture does the same on 5174.

After changing cockpit or SDK dependencies, run `deno install` in `web/` and commit the `deno.lock` update. CI validates it with `deno install --frozen`. The cockpit's fonts (Inter, JetBrains Mono) are npm packages bundled into `dist/` by vite, so nothing is fetched at runtime. If vitest misbehaves under a new Deno, the fallback ladder is `--no-file-parallelism`, then `--pool=threads`, then pinning a different vitest minor.

## Built bundles and wheels

Without vite, `--local-relay` serves the built `cockpit/dist` at `/` and `sdk/dist/sdk.js` at `/sdk.js`. In a checkout, `ensure_web_dist` in [`relay_process.py`](/dimos/web/relay_bridge/relay_process.py#L76) builds both first when either is missing or older than the sources. It stamps the shared, SDK and cockpit sources, the workspace config and the lockfile, builds under a cross-process lock into temporary directories, and swaps both products in together.

Release wheels ship both bundles prebuilt inside `dimos/web/relay_bridge/_relay_dist` (built by the release workflow, see [`setup.py`](/setup.py#L116)), so a pip-installed dimos never builds or downloads npm packages.

Two overrides: `--no-web-build` skips the staleness check and the build, and `DIMOS_WEB_DIR` points the bridge at another web tree.

## Tests

- `deno task test` in `web/` covers the relay and the shared protocol. The SDK and the cockpit have their own vitest suites (`deno task test` in each).
- `uv run pytest dimos/web` covers the Python side: the authoring API, the codecs, the manifest and protocol mirrors against the golden fixtures, the bridge module with fakes, and relay tests that spawn a real relay.
- The browser tests, `uv run pytest -m web_browser dimos/e2e_tests`, drive the whole stack against the Go2 replay dataset in Playwright Chromium and Firefox (their WebTransport stacks differ). [`test_cockpit_browser.py`](/dimos/e2e_tests/test_cockpit_browser.py) covers the cockpit (live data, a stable session, a kill and restart of dimos), [`test_sdk_browser.py`](/dimos/e2e_tests/test_sdk_browser.py#L24) the zero-build, cross-origin and `file:` SDK pages, and the other files custom channels, LCM channels, publishing, voice, stats, the robot picker, map clicks and auth. The pinned browsers install on first run. The default pytest suite excludes the marker.

The CI `web` job runs deno fmt, lint, check and test, the SDK and cockpit vitest suites and builds, the browser tests, and a build of the relay Docker image.

## Golden fixtures

The wire is pinned by the fixtures in `web/shared/fixtures/`. The [Wire protocol](/docs/web/protocol.md#golden-fixtures) page has the three regeneration commands. A wire change regenerates them on both sides in the same commit.
