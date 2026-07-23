---
title: "Concepts"
---

This page explains general concepts.

## Table of Contents

- [Modules](/docs-old/usage/modules.md): The primary units of deployment in DimOS, modules run in parallel and are python classes.
- [Streams](/docs-old/usage/sensor_streams/index.md): How modules communicate, a Pub / Sub system.
- [Blueprints](/docs-old/usage/blueprints.md): a way to group modules together and define their connections to each other.
- [RPC](/docs-old/usage/blueprints.md#calling-the-methods-of-other-modules): how one module can call a method on another module (arguments get serialized to JSON-like binary data).
- [Skills](/docs-old/usage/blueprints.md#defining-skills): An RPC function, except it can be called by an AI agent (a tool for an AI).
- Agents: AI that has an objective, access to stream data, and is capable of calling skills as tools.
