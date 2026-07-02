# DimOS Runtime Environments

This context defines the language for isolating module-specific dependency environments while preserving DimOS blueprint orchestration.

## Language

**Runtime Reconciliation**:
The deployment-time act of ensuring every required module runtime environment exists and is current before modules are launched.
_Avoid_: setup stage, prepare step

**Runtime Project**:
A package-local Python project that provides the dependency environment for one or more placed modules.
_Avoid_: worker package, env package

**Locked Runtime Project**:
A runtime project with committed lockfile state that deployment can install from without rewriting project files.
_Avoid_: auto-generated runtime env, mutable runtime project

**Runtime Environment Registration**:
The act of adding named runtime environments to a blueprint before deployment resolves placements and launches modules.
_Avoid_: setup registration, env setup

**Python Runtime Worker**:
A Python worker process launched from a registered runtime environment while preserving normal DimOS Python module semantics.
_Avoid_: native module, subprocess module

**Deployment Slice**:
The set of modules being deployed by one coordinator entrypoint, such as an initial build or a later blueprint load.
_Avoid_: whole app, reload batch

**Runtime Placement**:
The blueprint-level binding of a Module Contract to a named runtime environment and Runtime Implementation.
_Avoid_: instance placement, module-name placement

**Module Contract**:
A dependency-light module base class imported by the coordinator to declare the module's DimOS-facing surface.
_Avoid_: stub module, fake module

**Runtime Implementation**:
A runtime-project-local subclass of a Module Contract selected for execution inside a Python Runtime Worker.
_Avoid_: coordinator module, native module

**Contract Descriptor**:
A future portable description of a Module Contract's DimOS-facing surface for runtime validation without importing the contract class.
_Avoid_: current contract mechanism, runtime implementation
