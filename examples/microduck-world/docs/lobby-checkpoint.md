# Character selection checkpoint — 2026-09-05

The tailnet home page now offers Duck 1, Duck 2, Duck 3 and spectator entry.
Availability is live. Duck 1 is an exclusive slot, including the existing `?host`
shortcut. A conflicting claim returns HTTP 409 without replacing the owner.

The real robot meshes supply the portraits; no duplicate model downloads or three
extra live camera feeds were added. Portraits are rendered once and their temporary
GPU resources are disposed. Gold, teal and violet shells come from project scene
metadata and appear in native MuJoCo images and the exported Three.js model.
Duck 1 advertises four supplied room annotations and five known objects. Visitors
start fresh. Spectators see room labels and active duck positions on an overview.

All code and assets remain under `/home/tule/projects/microduck-world`. This
checkpoint changes no DimOS vendor files, patches, built-in registry or demos.
The existing cockpit's agent, mapping, policy and control panels remain in use.
Chakra Petch is hosted locally with its OFL license in the distributed client assets.

## Validation

- 65 project Python tests, including a real-process regression test proving that
  worker descendants are removed when their parent has already exited.
- A deliberately blocked test startup exited in 62.7 seconds with its stack
  recorded, proving the 60-second startup watchdog terminates a stalled build.
- 12 lobby relay tests, including three simultaneous identities, exact duck
  selection, host conflicts, disconnect reservations and cross-robot denial.
- Six frontend tests, TypeScript checking, production build, project Ruff and
  strict mypy for all 19 production Python files.
- Desktop 1440×1000 and mobile 390×844 screenshots visually reviewed, with all
  portrait images loaded and no horizontal document overflow.
- Four independent Chromium contexts reached Duck 2, Duck 3, Duck 1 and spectator
  views simultaneously. Both Three.js panels rendered for every duck. A fourth
  player could not take any occupied slot, and the host alias returned 409.
- Duck 2 moved about 29 cm while Ducks 1 and 3 remained stationary. Reload kept
  Duck 2. Leaving Duck 3 returned to the lobby, and the spectator took that released
  slot as a new visitor. Test sessions were released; only the resident runtime
  remained afterward.
- The exact configured OpenAI key is absent from reachable project history,
  project source and built browser assets. `config/agent.env` remains ignored,
  untracked and mode 0600. Detailed results are in the private audit log.

Evidence lives under ignored `logs/`: `multiplayer-checkpoint.json`,
`multiplayer-lobby.png`, `multiplayer-lobby-mobile.png`, `multiplayer-world.png`,
`key-exposure-audit.json` and `startup-timeout-check.json`.

## Issues found and practical limits

The Mac browser CLI stalled during screenshot capture; server Chromium supplied
the reviewed screenshots. Four simultaneous software WebGL views also overloaded
an early test run. The final test verified each Three.js view, then switched prior
contexts to native feeds before opening the next, keeping control timers responsive.
The spectator screenshot's low FPS reflects headless software rendering, not a
measurement of normal client hardware. This was a functional check, not a public
load test or an AI-provider billing test.

One repeated visitor startup stalled during module wiring. The project runtime now
bounds startup to 60 seconds, writes the blocked stack to its private log and exits
for supervisor replacement. Cleanup always removes that runtime's entire process
group, including workers left by an exited parent. The final simultaneous-login
run completed normally. The underlying intermittent RPC discovery stall is not
claimed resolved; bounded recovery and long-duration launch testing remain relevant
before public release.

`sim.tule.world` is not live. DNS/ingress, public transport, Duck 1 access rules and
anonymous agent limits are still pending; see [public-access.md](public-access.md).
Rotate the key that appeared in chat before public launch.
