---
title: "Concepts"
---

This page explains general concepts.

## Table of Contents

- [Modules](/usage/modules): The primary units of deployment in DimOS, modules run in parallel and are python classes.
- [Streams](/usage/sensor_streams/index): How modules communicate, a Pub / Sub system.
- [Blueprints](/usage/blueprints): a way to group modules together and define their connections to each other.
- [RPC](/usage/blueprints#calling-the-methods-of-other-modules): how one module can call a method on another module (arguments get serialized to JSON-like binary data).
- [Skills](/usage/blueprints#defining-skills): An RPC function, except it can be called by an AI agent (a tool for an AI).
- Agents: AI that has an objective, access to stream data, and is capable of calling skills as tools.
