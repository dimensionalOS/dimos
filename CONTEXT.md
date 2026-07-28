# DimOS

DimOS composes robotics capabilities into running systems that developers and agents can inspect and control.

## Language

**DimOS shell**:
An interactive Python environment connected to a coordinator on the configured transport bus for inspecting module instances and invoking their RPCs.
_Avoid_: REPL, console, attach

**RPC**:
An operation a module exposes for invocation from another process.
_Avoid_: Command, endpoint

**Skill**:
An RPC intended for agent and tool discovery, with additional descriptive and execution metadata.
_Avoid_: Separate skill call

**Module instance**:
A deployed occurrence of a module class, identified by its exact instance name. Multiple instances of the same module class may coexist under distinct names.
_Avoid_: Module class
