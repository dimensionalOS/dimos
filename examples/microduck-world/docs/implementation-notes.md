# Implementation boundaries

Historical implementation notes. For the current six-duck football application,
start with [the README](../README.md) and [cockpit behavior](cockpit-ui.md).
This package is now included at `examples/microduck-world` for review; it remains
independently installed and pins its own tested DimOS dependency.

## Application versus DimOS

The hosted world is an external Python package with a dimos.blueprints entry point.
Its composition, agent prompt, room/landmark metadata, scene XML, gateway, and
operational files belong to this project. The built-in blueprint registry is untouched.

The cumulative shared patch changes reusable behavior, including:
- Camera canvases fill their available area; paired feeds adapt to panel shape.
- Control strips use content height.
- NavMap optionally fits scene metadata, sharing one transform for imagery, overlays and clicks.
- Chat has an additive read-only display option, preserving the existing manifest format.
- The upstream integration includes a simulated upright-reset helper. This project disables automatic recovery and exposes a human-only Respawn button; see README simulation fidelity.
- Responses-based MCP agents receive camera images in the current tool result,
  fixing delayed observations and unnecessary retries.
- Reliable channel subscriptions request cached state for late viewers. Replay
  requests survive subscription coalescing; chat retains stable message numbers
  so existing viewers do not duplicate the history.

This checkpoint also binds MCP calls to deployed instance names, adds a
configurable per-runtime background tool topic, and exposes existing command
shaping and camera-ray helpers for simulation adapters.

Corresponding shared tests are included. No new hosted-world application logic
was added under DimOS demos. The Chat read-only option is a UI affordance;
it does not authorize or secure transport commands.

Scene metadata now rejects ambiguous names/aliases, reversed bounds, room targets
outside their room, and non-finite coordinates. Existing ScenePackage and RoomSpec
types are reused. The ball can already be disabled through MicroduckSimModule.ball_body;
there is no need for another project-specific simulation implementation.

## Reproducibility

`dimos-revision.txt` pins the base; `patches/dimos-hosted-world.patch` records the
shared changes. Setup checks the pin and accepts an already-applied patch.
Project dependencies are actually installed, and `uv pip check` verifies compatibility.
Test dependencies are a separate optional extra.

The pinned commit is available in `dimensionalOS/dimos`. To provision its source
from this application directory:

```bash
mkdir -p vendor
git clone https://github.com/dimensionalOS/dimos.git vendor/dimos
git -C vendor/dimos checkout "$(cat dimos-revision.txt)"
```

Install Python 3.12, uv, Deno and the Linux native dependencies described by the
pinned framework before running `./setup --test`. Fetch robot assets through the
pinned Microduck asset downloader. The HTTPS gateway additionally needs a local
`config/tailnet.json` matching `GatewayConfig`, certificates and private keys;
public hosting needs the resources described in [public launch](public-launch.md).
The live server's private backups and configuration are intentionally excluded.

## Remaining boundaries

One shared physics world hosts three independently built robot blueprints.
Duck 1 retains the host annotations; Duck 2 and Duck 3 learn from their own
sensors and keep private maps, contexts, tool topics and knowledge. See
[multiplayer](multiplayer.md) for lifecycle and isolation details.

The tailnet gateway preserves certificate pinning and adds trusted HTTPS, but the
underlying relay still trusts reachable peers. Public authorization must cover
robot registration and every command channel, not only keyboard teleop.
See public-access.md before exposing an internet-facing endpoint.

All service source and logs live in this directory. User systemd only maintains
registration symlinks outside it. Physics state is transient; named places persist.
TLS certificate renewal remains manual.

## Client world panel

The project now owns a Three.js panel under web/ and exports the compiled MuJoCo
model through WorldSimModule. It reuses the cockpit panel registry and existing SDK
without extending the shared DimOS patch. See [client rendering](client-rendering.md)
for the protocol, assets, setup and browser checkpoint.
