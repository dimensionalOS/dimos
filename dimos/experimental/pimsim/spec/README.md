# PimSim spec

A small, dependency-light contract layer for PimSim, in the same shape as
`dimos/manipulation/planning/spec/`: the **protocols** new code should be read
and reasoned about through, separate from the sprawling concrete modules that
implement them.

PimSim today is large and protocol-free — physics, scene, rendering, sensing,
and an LCM/WebSocket bridge are spread across `module.py` (~1200 lines),
`entity.py`, `client.py`, `headless.py`, and the `scene_assets/` cooker. This
package does **not** add behaviour. It *names* the contracts that were only
implicit, so the architecture is legible without reading all of it.

## The real contribution, in one sentence

> Physics authority is a **pluggable role**; everything else is decoupled from
> it through three shared contracts, so any authority can drive any consumer.

That is the whole thesis (`../DESIGN.md` argues it from first principles). The
spec encodes it as **three data/scene contracts** + **two roles**.

## Three contracts (the nouns)

| Contract | What it is | Where it lives |
|---|---|---|
| **Scene package** | what geometry *exists* — cooked offline, portable (visual GLB, decimated collision GLB, per-entity GLBs, CoACD hulls, `objects.json`, MuJoCo wrapper MJCF) | `dimos/simulation/scene_assets/spec.py` → `ScenePackage` (concrete dataclass) |
| **Entity stream** | where everything *is now* — `EntityDescriptor` (what a thing *is*) + `EntityStateBatch` (a per-tick `(descriptor, pose)` snapshot), versioned length-prefixed JSON over LCM so the Rust lidar and the browser can decode it | `../entity.py` (concrete) |
| **LCM bus** | the transport, plus an LCM-over-WebSocket bridge so a browser tab is a first-class bus participant | dimos LCM; bridge in `../module.py` |

`models.py` re-exports the first two; `enums.py` holds `EntityKind`,
`ShapeHint`, and the new `AuthorityMode`.

### What `EntityDescriptor` is (the part that keeps being unclear)

`EntityDescriptor` is **identity + how to instantiate**, nothing temporal:
a stable `entity_id`, a `kind` (`dynamic` / `kinematic` / `static`), a
`mesh_ref` (GLB) *or* a `shape_hint` + `extents` for primitives, `mass`, and an
optional `rgba`. It answers "what is this object and how do I create it once."
**Pose is deliberately not in it** — that lives in `EntityStateBatch`, which
pairs each descriptor with its current `Pose` and is restreamed every tick.
The split is what lets a consumer cache identity/geometry once (by
`entity_id`) and then just apply pose updates.

## Two roles (the protocols — `protocols.py`)

| Protocol | Role | Style | Implemented by |
|---|---|---|---|
| **`EntityAuthority`** | owns a scene's physics; broadcasts an `EntityStateBatch` every tick | pub/sub port (`Out[EntityStateBatch]`) + one `@rpc` | `BabylonSceneViewerModule` (Havok) · `MujocoSimModule` (headless) |
| **`EntityConsumer`** | authority-blind reader of the stream; never touches a physics engine | pub/sub port (`In[EntityStateBatch]`) | `SceneLidarModule` · `SplatCameraModule` · `MujocoWorld.sync_entity_poses` · reachability builder |

**Why two interface styles in one spec.** PimSim's entity flow is *streaming*:
an authority publishes on its own clock and nobody calls it, so its contract
is the dimos **port** (`Out`/`In`), declared as a Protocol attribute — not a
method. Surfaces that are genuinely *called* synchronously use **method
stubs** (the `WorldSpec` style): the `@rpc spawn_entity`, and the proposed
`SceneObjectWorld` below. *Streaming = ports, synchronous = methods* — and
knowing which a thing is, is half of understanding PimSim.

`EntityAuthority` also carries `authority_mode` (`OWNS` vs `MIRROR` — today the
Babylon `entity_authority` `"browser"`/`"external"` string). The
`entity_state_batch` it publishes each tick *is* the contract every consumer
depends on; everything else exists to produce it.

## Proposed: one scene noun, two verbs (`DESIGN.md` §7-A)

Three types describe "a shaped thing at a pose": `Obstacle` (planning),
`EntityDescriptor` (PimSim), perception `Object` (`Detection3D`). The proposal
(**not implemented**) merges the first two into one `SceneObject` (`models.py`)
and gives the planning world two verbs (`SceneObjectWorld` in `protocols.py`):

- `add_object(obj, pose)` — inject **new** geometry (what `add_obstacle` and a
  perception detection do; mutates the body set), and
- `update_object_pose(id, pose)` — reposition **known** geometry (what
  `sync_entity_poses` does each tick from the stream; writes a pose only).

Perception `Object` stays separate (it pulls open3d + cv2, is detector output)
and is *converted into* a `SceneObject`. The payoff: `EntityStateBatch` becomes
the streaming form of `(SceneObject, pose)` — the entity stream and the
planning world share one vocabulary.

## How to read the diagram

```
   AUTHORITY (EntityAuthority)        CONSUMERS (EntityConsumer)
   ┌──────────────────┐ Out[Entity   ┌────────────────────────────┐
   │ Babylon + Havok  │  StateBatch]  │ scene_lidar (Rust raycast)  │
   │  (OWNS / MIRROR) │ ──over the───►│ splat / camera views        │
   │ MuJoCo (OWNS)    │   LCM bus     │ planning world (collision)  │  In[EntityStateBatch]
   └──────────────────┘               │ reachability builder        │
        ▲ ingests once                └────────────────────────────┘
        └── ScenePackage (cooked geometry)   no consumer knows the authority
```

## Status

The protocols describe the **intended** interface; the concrete modules
satisfy them in spirit (the ports already exist on the Modules) but don't yet
*declare* them. Making `BabylonSceneViewerModule` / `MujocoSimModule` and the
consumers conform explicitly — and landing the `SceneObject` convergence
(§7-A), which is agreed jointly because it touches every world backend — is the
follow-up this spec exists to anchor.
