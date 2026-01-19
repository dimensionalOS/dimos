# Concepts

If you are trying to learn a new API or know what tools you should use, then you are in the right place. In contrast, if you already know the API you want to use and just need details on how to use it, see [the API reference](/docs/API/README.md).

## Table of Contents

- [Modules](/docs/concepts/modules.md): The building blocks of DimOS, modules run in parallel and are singleton python classes.
- [Streams](/docs/api/sensor_streams/index.md): How modules communicate, a Pub / Sub system.
- [Blueprints](/docs/concepts/blueprints.md): a way to group modules together and define their connections to each other.
- [RPC](/docs/concepts/blueprints.md#calling-the-methods-of-other-modules): how one module can call a method on another module (arguments get serialized to JSON-like binary data).
- [Skills](/docs/concepts/blueprints.md#defining-skills): An RPC function, except it can be called by an AI agent (a tool for an AI).
- Agents: AI that has an objective, access to stream data, and is capable of calling skills as tools.
