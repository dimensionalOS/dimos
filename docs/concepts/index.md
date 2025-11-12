# Concepts

This section explains the key concepts and abstractions that make up DimOS.

## [Agent](agent.md)

LLM-based reasoning systems that orchestrate robot behavior by processing natural language commands and intelligently executing skills

## [Skills](skills.md)

Skills are how you give agents control over robot capabilities.

## [Modules](modules.md)

Every DimOS component -- from hardware drivers to AI agents -- is a Module: a distributed actor communicating through typed streams and RPC

## [Blueprints](blueprints.md)

Declarative specifications for wiring modules together

## [Transport](transport.md)

Abstraction layer for message passing between modules -- same code works across different backends like LCM for network or shared memory for local IPC
