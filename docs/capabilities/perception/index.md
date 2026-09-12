# Perception

## Detections

## Re-identification

A tracker associates across adjacent frames, so it breaks on every occlusion, turn
away, or detector miss, and each break invents an object. `EmbeddingIDSystem`
(`dimos/perception/detection/reid/`) assigns long-term ids from appearance and is what
`ReidModule` runs.

[Tracklet re-identification](tracklet_reid.md) adds position and time as vetoes ahead
of appearance — two chairs of one model are identical to a similarity model, and only
where and when they were separates them. That page records what was measured, what the
numbers rule out, and which of them are withdrawn.

## Experimental WorldBelief

The experimental xArm6 WorldBelief stack records RGB-D observations and processes
them on demand to maintain object identities across scans and process restarts.
Its implementation lives in
`dimos/experimental/world_belief/` while the shared detector,
embedding, and 3D object primitives remain in their standard packages.

Run the hardware blueprint:

```bash
dimos --xarm6-ip <ROBOT_IP> run xarm6-worldbelief
```

Then request a scan or recall from another terminal:

```bash
dimos mcp call scan -a prompt='["mug", "coke can"]'
dimos mcp call recall -a text="mug"
```

This stack is experimental and may change without compatibility guarantees.
