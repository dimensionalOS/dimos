---
status: accepted
---

# Coordinator loops are attachable

A `ModuleCoordinator` that enters `loop()` starts its coordinator RPC service and is therefore attachable by debugging clients. Connection attempts probe that service directly rather than requiring a local CLI run-registry entry, so CLI-launched and direct Python-launched systems share the same attachment model.

The RPC start is idempotent because daemonized CLI runs must still start it only after daemonization. This choice makes entering the run loop expose coordinator RPC on the configured transport bus; callers that build a coordinator without entering `loop()` do not expose it automatically.
