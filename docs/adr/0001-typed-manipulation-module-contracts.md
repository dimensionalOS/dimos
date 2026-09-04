# Typed manipulation contracts on deployed Modules

Python applications and agent tools use the same manipulation operations on
`ManipulationModule` and `PickAndPlaceModule`. Public Specs and domain result
types define that contract; `Dimos.get_module(Spec)` selects a deployed provider
by its advertised RPCs and compatible signatures, failing on missing or ambiguous
matches. This keeps robot composition in Blueprints and avoids a second client
wrapper with its own operation implementations or connection lifecycle. Agent
formatting uses `agent_encode()` at the MCP boundary; Python callers receive the
typed result directly.
