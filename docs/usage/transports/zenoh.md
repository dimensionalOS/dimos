---
title: "Zenoh Session Modes"
---

Zenoh sessions come in three modes, and the choice decides **which sessions can
reach each other** and **how many copies of a stream cross a link**. Everything
below was measured against zenoh 1.9, the version dimos pins.

## The modes

| mode | links it holds | forwards between its links? |
|---|---|---|
| `peer` | meshes with every peer it finds or dials | **no** |
| `client` | exactly one, to its router | n/a |
| `router` | any number | **yes, to clients** |

Set it with `--zenoh-mode`, `ZENOH_MODE`, or `zenoh_mode` in `.env`. Native
(rust) modules read `DIMOS_ZENOH_MODE`, which the launcher derives from the same
config — but a **baked** host binary is started by systemd and never goes
through the launcher, so its mode comes from the unit's `Environment=` block and
nothing else.

## What actually reaches what

The one rule worth memorising: **a router forwards to clients, not to peers.**

| topology | delivery |
|---|---|
| router hub, client spokes | yes |
| router hub, **peer** spokes | **no** |
| peer hub, peer spokes | no |
| client ↔ peer, through a router | **yes**, both directions |

Two consequences:

- Standing up a router changes nothing until the sessions behind it are
  **clients**. Peers hung off a router still cannot see each other.
- A peer on the far side of the router is fine. A laptop dialling a robot's
  router needs no reconfiguration.

`routing/peer/mode = "linkstate"` — the documented way to make peers route — is
**not a config key zenoh 1.9 accepts**. Client mode is the only option.

## Why it matters: one wifi copy, not N

Zenoh delivers one copy per **link** with a matching subscription, not per
subscriber. So two processes on a robot that each hold their own link to a
laptop cost two copies of every stream they both want — over the one link that
is actually scarce.

Put a router on the robot and make the local sessions clients, and the laptop
pushes one copy to the router, which fans it out locally.

```
before                          after
laptop ──┬── proc A (peer)      laptop ── router ──┬── proc A (client)
         └── proc B (peer)                         └── proc B (client)
2 wifi copies                   1 wifi copy
```

Subscriptions within a **single** session are already free — the pool shares one
session per `(mode, connect, listen, multicast_interface, multicast, gossip)`
tuple, so several modules in one process cost one link between them.

## Discovery: multicast and gossip

Two independent mechanisms, both on by default:

- **Multicast scouting** finds peers on an interface. `zenoh_interface` /
  `zenoh_scouting` set how far it reaches (loopback by default); `zenoh_multicast`
  turns it off entirely.
- **Gossip** propagates peer locators along **established links** — it is what
  turns one dialled endpoint into a whole mesh, so a peer listening on an
  ephemeral port is still reachable. `zenoh_gossip` turns it off.

Gossip is not multicast: pinning multicast to loopback does nothing to gossip.

Measured: a **client stays single-linked whether gossip is on or off** — it does
not dial peers it learns about. So `mode=client` alone is enough to keep a
session off the wifi mesh, and gossip can stay on.

## Naming what to dial

Connect endpoints normally come from `--robot-ip`, and only when the stack's
transport is zenoh — a session belonging to *this process's* robot. Tools that
merely want to watch someone else's robot have no such robot, so
`--zenoh-connect` (`ZENOH_CONNECT`, `zenoh_connect` in `.env`) names endpoints
directly and is honoured whatever the transport. Comma separated; a bare host
gets `tcp/` and port 7447.

Watching a Go2, whose `dimos-helper` runs a **router**, from a laptop:

```bash
dimos --zenoh-connect go2 --zenoh-mode client spy --transport zenoh
```

Client mode is the load-bearing half — see the delivery table above. Note the
filter goes *after* `spy`: a root `--transport` would set the stack backend, and
the spy watches every transport regardless, so it rejects that placement.

## Gotchas

- **`client` + multicast off + no connect endpoints fails at `open()`** —
  `No peer specified and multicast scouting deactivated!`. There is no discovery
  fallback, so always give a client its router's address. Peer mode only warns.
- **`_await_connect()` is a no-op when `connect` is empty**, so a session
  relying on discovery alone gets no startup wait and can publish into the void.
- **Turning gossip off** makes zenoh warn that it cannot de-duplicate data
  traversing region gateways. Harmless in a strict star, but multiply-connected
  peers can then double-deliver.
- **The python knob and the native knob are different env vars** —
  `ZENOH_MODE` for `GlobalConfig`, `DIMOS_ZENOH_MODE` for rust children. Setting
  one by hand on a robot leaves the other at its default.

## Related

- [Transports](/docs/usage/transports/index.md) — what the transport layer guarantees.
- [Installing DDS transport libs](/docs/usage/transports/dds.md) — the other backend.
