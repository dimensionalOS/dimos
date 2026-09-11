# Public access: sim.tule.world

Historical preparation notes. The current implementation and remaining launch
checks are in [public-launch.md](public-launch.md). That implementation uses six
equal player slots, GitHub identity, fresh private context for every duck, and a
Cloudflare WSS relay. The three-slot and anonymous-access discussion below
describes an earlier deployment.

The running deployment remains private to the tailnet at
`https://omarchy.tailca0707.ts.net:8443/`. The new character-select lobby is not a
public launch. DNS, public ingress and domain TLS have not been configured.

## Intended experience

Visitors choose an available Duck 1, Duck 2 or Duck 3, or watch without occupying a
slot. Each duck has one controller and its own agent tools, camera and knowledge.
Duck 1 retains the supplied apartment annotations; visitor ducks start fresh.
The host is a robot identity, not an administrator role.

The current tailnet deployment allows any tailnet user to claim any available duck.
`?host` remains an alias for Duck 1 and respects the same exclusive ownership check.
Whether public Duck 1 should require host authentication is an open product decision.
If it is public, its persistent conversation and knowledge must be treated as shared
public data; private host conversation must be cleared or separated before launch.

## Existing boundaries

- Opaque participant tickets bind a viewer to one runtime. The relay rejects
  spectator commands, private spectator subscriptions and cross-robot watch/control.
- All three duck slots are exclusive. Reload retains ownership; leaving releases
  immediately; a disconnected owner has 60 seconds to reconnect.
- Visitor generations isolate commands, sensor streams, conversations and maps
  from previous occupants. The shared world stream is for human visualization.
- Robot registration requires a rotating server-only credential. Its discovery
  endpoint, assignments and all MCP servers are loopback-only.
- The HTTPS gateway validates Host and Origin and exposes an allowlist of page,
  asset and session routes. Configuration, logs, state databases, source files and
  `/internal/` are not served. UDP peer storage and participant storage are bounded.

These are application boundaries, not a sandbox for running untrusted code on the
server. No visitor code uploads or execution are offered.

## API key

The OpenAI key is loaded only by server processes from ignored `config/agent.env`
(mode 0600). It is not a frontend environment variable. The key audit scans every
reachable project revision and built browser file for the exact configured value
without printing it. Results are recorded in ignored `logs/key-exposure-audit.json`.

The key previously appeared in the operator's chat, so replace it before a public
launch. Keeping it out of JavaScript does not prevent API charges: an anonymous
visitor can still ask their server-side agent to perform work.

## Remaining release work

1. Confirm DNS management and the public network route for `sim.tule.world`.
2. Choose public Duck 1 ownership and conversation persistence rules, and implement
   authentication if that identity is private.
3. Bound anonymous agent work: admission and command rates, concurrent turns,
   per-session limits and a global usage ceiling with an operator shutoff. Bound
   spectator load and long-lived idle sessions as part of the same admission policy.
4. Configure domain TLS and the chosen ingress; retain the private gateway's bind
   guard until a separate public configuration and its tests are ready.
5. Test from outside the tailnet: all four entry paths, ownership conflicts,
   revoked/forged sessions, denied cross-duck actions, reload, disconnect, server
   restart, slow viewers and enforcement of agent limits.

## Transport requirement

The browser currently uses HTTPS over TCP plus WebTransport/QUIC over UDP. An
HTTP-only proxy or tunnel may serve the lobby while leaving the feeds disconnected.
A public route must carry the required UDP traffic with trusted HTTPS discovery, or
we must implement and test an alternate browser transport before choosing an
HTTP-only ingress. A DNS record alone is not enough.

Project admission policy belongs in this repository. Generic transport support
belongs in DimOS. Scene metadata and colors remain in `assets/scenes/apartment/`;
public hosting does not require adding scene-specific logic to DimOS demos.
