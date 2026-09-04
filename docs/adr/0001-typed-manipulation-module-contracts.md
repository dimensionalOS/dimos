# Typed manipulation contracts on deployed Modules

Python applications and agent tools use the same manipulation operations on
`ManipulationModule` and `PickAndPlaceModule`. Public Specs and domain result
types define that contract; `Dimos.get_module(Spec)` selects a deployed provider
by its advertised RPCs and compatible signatures, failing on missing or ambiguous
matches. This keeps robot composition and operation implementations in the
runtime. The client-only `dimos.sdk.manipulation.Arm` binds one group, converts
numeric targets, sequences common blocking calls, and raises with the original
failure result. It borrows the connection and exposes its typed RPC proxy for
advanced operations; it does not change motion behavior or own plans. Agent
formatting uses `agent_encode()` at the MCP boundary; Python callers receive
typed results.
