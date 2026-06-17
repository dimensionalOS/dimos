# DimOS

DimOS composes robot software into named runnable stacks that users can launch from the command line or Python APIs.

## Language

**Blueprint**:
A composition of modules that represents a runnable robot stack.
_Avoid_: stack definition, launch file

**DimOS Module**:
An autonomous subsystem that can participate in a blueprint and communicate with other modules through typed streams or RPC.
_Avoid_: Python module, component, node

**Runnable Blueprint Name**:
The public name a user invokes to select a blueprint for execution.
_Avoid_: registry key, CLI alias, package name

**Namespaced Blueprint Name**:
A runnable blueprint name formed from an external blueprint namespace, a dot separator, and a local entry name.
_Avoid_: fully qualified import path, plugin route

**External Blueprint Namespace**:
A required prefix, derived by default from the installed distribution name, that identifies the source of runnable blueprint names provided outside DimOS itself.
_Avoid_: plugin prefix, discovery trigger, package alias

**Canonical Distribution Namespace**:
The normalized form of an installed distribution name used as an external blueprint namespace.
_Avoid_: raw package name, import package name

**External Blueprint Discovery**:
The recognition of externally provided runnable blueprint names from installed package metadata without loading the blueprint itself.
_Avoid_: validation, import scan, registry generation

**External Blueprint Entry Point**:
An installed package metadata declaration that provides a runnable blueprint name by pointing to either a blueprint object or a DimOS Module class.
_Avoid_: factory hook, plugin loader, filesystem scan

**External Local Blueprint Name**:
The package-defined suffix of a namespaced blueprint name, using DimOS-style lowercase kebab-case.
_Avoid_: Python symbol name, import name, nested path, runnable entry

**Bare Blueprint Name**:
A runnable blueprint name without an external namespace; bare names refer only to DimOS-provided blueprints.
_Avoid_: unqualified plugin name, default external name

**Blueprint Composition Command**:
A `dimos run` invocation that names one or more blueprints or modules, each resolved independently and composed into one runnable stack.
_Avoid_: launch command, multi-blueprint mode
