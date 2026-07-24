# Make obstacle replacement an atomic scene operation

Obstacle replacement accepts a complete `Obstacle` identified only by its immutable name, while a separate first-class pose update preserves all non-pose properties. All planning-world obstacle operations require a finalized world. The world snapshots accepted values and returns copies so callers cannot bypass atomic mutation by retaining a mutable object reference. Each scene query or mutation observes either the complete scene before an obstacle update or the complete scene after it, while a multi-step planning run may span scene revisions so frequent perception updates do not starve planning. Unexpected native update failure is treated as an invariant violation: the exception propagates, the planning world becomes unusable, and recovery requires reconstruction rather than rollback. The planning world is authoritative; visualization is a best-effort projection whose failure neither rolls back the update nor invalidates the planning world.

Opaque backend planners count as one atomic scene operation because their internal collision queries cannot be synchronized individually. Consequently, RoboPlan-native planning excludes obstacle replacement for the full native call; rapidly changing worlds should use planners whose collision-query boundaries are exposed through `WorldSpec`.

Update methods return `False` only when the named obstacle does not exist. Lifecycle violations and invalid inputs raise before mutation; unexpected native failures propagate and invalidate the world. Visualization commands return no world result and their failures remain isolated from the authoritative planning world.

Version one does not retry or resynchronize a failed visualization update. Each visualization implementation absorbs its own renderer failures, records them in logs, and exposes a persistent warning when it has a frontend. Renderer error handling remains internal to the visualization boundary and does not require an orchestration-level warning API.

Each world backend owns one reentrant lock that serializes native scene operations, including queries, obstacle mutations, and opaque planning calls. This intentionally favors simple, auditable exclusion over a custom read/write lock; any later concurrency optimization must preserve the same atomic scene-operation contract.

Incoming pose-only messages use the dedicated pose update. A message carrying any structural or appearance change must provide a complete obstacle and uses full replacement; missing fields are never merged from stored state. An unknown pose-only target is not implicitly added, while a complete replacement message may retain the existing update-as-add ingestion behavior.

The existing `WorldObstacleMonitor` callback contract is intentionally unchanged:
an `"update"` callback continues to receive `None` for its obstacle payload,
including after a complete replacement. The currently unused callback extension
point should be discussed separately with repository owners before its contract
is expanded.

Atomicity is per obstacle operation, not per perception frame. Collision queries may run between updates to different obstacles from one incoming frame; version one does not introduce a batch obstacle transaction.

Stored-plan invalidation and execution-time obstacle handling are outside this decision. A successful obstacle update does not invalidate an existing generated plan or add execution-time collision guarantees; those require a separate policy that accounts for continuously changing worlds.

## PR review note

`WorldObstacleMonitor` exposes add/remove callback registration even though no in-tree production consumer registers it. Callback compatibility does not gate this implementation and no callback redesign is in scope; any incidental behavior change should be called out for repository-owner discussion during PR review.
