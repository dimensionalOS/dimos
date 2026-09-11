# Microduck World — checkpoint report, 5 September 2026

Project: /home/tule/projects/microduck-world on Omarchy.
Browser URL: https://omarchy.tailca0707.ts.net:8443

## Completed checkpoints

### 1. Cockpit functionality

Restored Control, mode and policy streams, Agent transcript, ObserveSkill, and
McpServer. The external blueprint conditionally includes McpClient when an OpenAI
key is configured. The scene-specific agent prompt uses project room metadata.

Browser checks exercised stand, sit/stand in both directions, left/right kicks,
roulade, and ground pick. All supported actions completed without a reported
policy error or fallen state. Roller policies correctly explain that this robot
variant does not support them.

The running MCP server exposes 18 tools on loopback. Direct list_places returned
the four rooms and five landmarks from project assets. Direct observe returned
a valid 640 × 360 JPEG from the robot camera. These checks used no LLM.

The user authorized the local OpenAI key on 5 September. It was verified against
OpenAI, transferred over SSH stdin, stored in ignored config/agent.env with mode
600, and loaded by the supervised world. The live blueprint now includes McpClient.

Browser chat tests passed for scene discovery, camera observation, kitchen
navigation, sit/stand, and operator cancellation. Kitchen navigation reached
(1.06, 0.88), confirmed independently from odometry. Cancelling a later living-room
trip cleared the goal and the agent correctly reported that it stopped early.

The first live camera test exposed a shared MCP bug: the image was queued for a
later user turn, so the agent initially reported no image and retried. The client
now returns multimodal content directly in the Responses tool result. The retest
produced one observe call, an image tool result, and one accurate answer in about
three seconds. Fourteen MCP client tests and strict mypy passed. Regression tests
also verify the actual request serialization without contacting a provider.

A second-viewer check exposed missing chat history while another browser stayed
connected. The shared relay now requests cached reliable-channel state for late
viewers; the robot preserves those requests when subscription bursts coalesce.
Requests are bounded by the current channel set, and stable chat message numbers
prevent duplicates. After the final deploy, a real list_places conversation
replayed completely into a reloaded second Mac browser while the first retained
exactly one copy of each message. Both 640 × 360 feeds remained live. A Teleop/Agent mode round trip retained the history; Teleop correctly hides
the composer behind the mode notice. The duck was left standing in Teleop mode.

### 2. Panels and navigation

Camera canvases fill their available surface with aspect ratio preserved.
Tall panels stack the chase and head feeds at full width; wide panels use an
inset. The head feed now renders at 640 × 360. The Control strip no longer
reserves unused vertical space.

NavMap optionally fits room/object metadata, using the same transform for the
costmap, labels, robot and click coordinates. There are no apartment coordinates
hardcoded into this option.

Browser checks covered 1440 × 900 and 1280 × 720 layouts, maximize/restore,
live feeds, map goals, cancellation, and keyboard release. A kitchen click sent
(1.2, 1.0), entered following_path, and cancellation cleared the goal. Keyboard
drive produced vx=0.15 and measurable movement; release returned vx to zero.
An initial key press sent immediately with the asynchronous arm request was
ignored; the check passed after waiting for the armed state.

### 3. Headless operation and recovery

Project-owned systemd user services supervise the world and HTTPS/UDP gateway.
Both are enabled, and tule has Linger=yes, allowing operation without a desktop
login. The relay's existing --open-browser false option disables desktop browser
launch on this headless host. Wrapper commands route restarts to systemd and prevent duplicate worlds.

Deliberately killing each service's main process verified automatic recovery.
The browser resumed after both world and gateway failures without manual reload.
A real server reboot has NOT been tested.

Added /healthz and a browser startup page that retries while the simulation
loads. The startup page recovered automatically in the browser. Readiness means
a robot is registered, not that every sensor or the LLM is healthy.

### 4. Project organization and setup

Scene XML, room/landmark metadata, cockpit composition, prompt, gateway,
deployment scripts and operational documentation live in the external project.
No hosted application logic was added to DimOS demos or the built-in registry.

The shared patch contains reusable camera layout, optional read-only Chat,
optional scene-fitting NavMap, spawn-relative fall recovery, immediate MCP image
results, and late-viewer history replay, with tests.
Scene validation rejects ambiguous aliases, invalid bounds/targets and non-finite
coordinates.

Setup pins the existing DimOS revision, applies the patch idempotently, installs
project dependencies and checks dependency compatibility. A full setup --test
run passed with 264 compatible packages. The vendor checkout lacks older Git
objects, so a full-history bundle was impossible. A project-local Git-directory
backup can restore the pinned source tree; restoring and applying the final
patch was verified. It is not a full-history or off-machine backup.

### 5. Public access preparation

docs/public-access.md records the relay audit and required authorization design.
The current relay trusts reachable peers; generic goal, policy and agent
commands are not all protected by the keyboard controller lease. An HTTP-only
tunnel does not cover the UDP transport.

The application remains tailnet-only. Public authorization, internet exposure,
the later sim.tule.world domain, visitor-owned ducks and scene redesign have
NOT been implemented.

## Validation

- 147 cockpit frontend tests passed; TypeScript check passed.
- 94 Python cockpit tests passed.
- 29 external-project tests passed.
- 5 targeted simulation fall-recovery tests passed.
- 14 MCP client tests passed, including immediate camera delivery and wire-format checks.
- 296 Python relay protocol, bridge and transport-session tests passed with asyncio enabled.
- 76 TypeScript relay registry and protocol tests passed.
- Strict mypy passed for gateway, UDP forwarder, scene, agent, MCP client and
  the changed relay protocol, bridge and transport session.
- Project Ruff checks and dependency compatibility checks passed.
- Browser regressions ran from the Mac over tailnet and real headless Chromium
  on Omarchy through a loopback-only SSH CDP tunnel. Final two-viewer replay
  verification and the final screenshot used separate Mac browser sessions.

The six-hour browser soak finished with 73/73 checkpoints passing, from
04:39:24 to 10:39:24 UTC on 5 September. It checked trusted HTTPS readiness and
actual browser chase/head camera, odometry and policy sequence advancement every
five minutes, with hourly screenshots. Both services recorded zero restarts during
the run. This run preceded agent enablement and the later shared fixes; it is
not a six-hour soak of the final version. The live agent and two-viewer replay
checks above cover those changes. The final source deploy was at 15:58 UTC.

Evidence is in docs/stability-report.md and
logs/browser-soak-20260905T043853Z/. Shared test logs, live runtime logs and
additional browser captures are under logs/. The final desktop screenshot is
logs/final-cockpit-20260905.png. After adding test-only HTTP type
stubs, all 265 installed packages passed the dependency compatibility check.

## Remaining work and limits

1. Implement and test public transport authorization before exposing the domain.
   The current public-access deliverable is the audit and permission design.
2. Test an intentional full reboot when convenient.
3. Renew the Tailscale certificate before 4 December 2026; renewal is manual.
4. The existing Zenoh memlock warning remains; local transport falls back, while
   browser chase video measured approximately 18–19 fps.
5. Domain publication, visitor-owned ducks and scene redesign remain deferred.

The earlier credential-transfer block was resolved by explicit user authorization.
Live agent validation used the authorized OpenAI account. No API key is included
in the source repository or this report.

## Movement handoff correction — 5 September, 16:13 UTC

The final handoff had left the duck on stand. The user then reported that neither
WASD nor room clicks moved it. Live state confirmed Teleop mode and the stand
policy, with no fallen/locked state. Selecting walk restored movement: a two-second
W hold changed odometry from about (0.008, 0.004) to (0.182, -0.021), and clicking
the kitchen label reached (1.045, 0.902). The duck is now left on walk with no
movement command held. The Control strip explains that stand holds position and
walk is needed to drive or navigate.

The browser automation CLI's letter-key command emitted an empty physical code;
the movement retest used a DOM keyboard event with code KeyW and a guaranteed
keyup. The real room-label test used mouse input. Both were verified against
robot odometry, not only UI command values.
