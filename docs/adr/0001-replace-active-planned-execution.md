# Replace possibly active planned execution before dispatch

A newly requested planned execution replaces the previously accepted plan rather
than overlapping with it. Because execution acceptance does not establish physical
completion, every task that might still belong to the previous plan must first be
confirmed safe through cancellation; if any task's safety is uncertain, the new plan
is not dispatched. This avoids requiring coordinator-side execution observation or
atomic multi-plan arbitration while preserving a single-writer safety model.
Concurrent execution requests fail fast rather than entering a hidden queue.
One condition arbiter serializes execution and cancellation. A waiting cancellation
immediately gates new dispatches, waits for the current coordinator call to
resolve, and then cancels every task that might have accepted the plan. An
in-flight multi-robot dispatch stops before sending the next robot when
cancellation is waiting.
The generated plan is the atomic execution unit: a multi-robot plan is dispatched
in full or rejected, never filtered to a single robot at execution time.
The execution manager is conventionally the single writer for its configured
coordinator trajectory tasks, but the coordinator does not enforce that ownership.
Manual or diagnostic calls that execute the same tasks concurrently violate this
operating assumption.
