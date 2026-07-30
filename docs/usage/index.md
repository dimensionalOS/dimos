(doc-usage-index-concepts)=

# Concepts

(doc-usage-index-usage)=

# Usage

Learn how DimOS modules communicate, compose, and run.

(doc-usage-index-table-of-contents)=

## Table of Contents

- [Modules](modules.md): The primary units of deployment in DimOS, modules run in parallel and are python classes.
- [Streams](sensor_streams/index.md): How modules communicate, a Pub / Sub system.
- [Blueprints](blueprints.md): a way to group modules together and define their connections to each other.
- [RPC](blueprints.md#calling-the-methods-of-other-modules): how one module can call a method on another module (arguments get serialized to JSON-like binary data).
- [Skills](blueprints.md#defining-skills): An RPC function, except it can be called by an AI agent (a tool for an AI).
- Agents: AI that has an objective, access to stream data, and is capable of calling skills as tools.

```{toctree}
:hidden: true
:maxdepth: 2

modules
blueprints
configuration
cli
lcm
transforms
visualization
python-api
camera_calibration
native_modules
tool_streams
transports/index
data_streams/index
sensor_streams/index
```
