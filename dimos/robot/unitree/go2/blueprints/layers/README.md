# Go2 Architecture Layers

This directory names the Go2 blueprint composition boundaries that map to
layers 3 through 6 of the project architecture diagram. These files do not move
implementation modules; they only group existing blueprints so the runtime
structure can evolve without breaking current import paths.

## Layer Mapping

- Layer 3, Agent Brain: MCP server/client and the LLM agent loop.
- Layer 4, World State: spatial memory today, with a lazy temporal-memory
  factory for the existing temporal-memory blueprint.
- Layer 5, Skill Interface: MCP-callable skills and skill containers that
  bridge the agent to navigation, perception, speech, and Unitree commands.
- Layer 6, Robot Body / Local Policy: the existing Go2 robot stack, including
  perception, mapping, navigation, movement management, and hardware connection.

All blueprint variables in this directory are intentionally prefixed with `_`.
The registry generator skips those names, so these internal layer nodes do not
become new CLI blueprints.
