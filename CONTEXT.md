# DimOS installation language

Terms used to describe a DimOS installation and the evidence that it is ready.

## Language

**Managed DimOS project**:
A developer-owned Python package or DimOS contributor checkout with a recorded installation profile and its own environment.
_Avoid_: Global install, arbitrary existing project

**Installation profile**:
A named selection of dependencies for a reference DimOS use case.
_Avoid_: Everything install, all robots

**Verified setup**:
A particular project and platform whose required installation and dependency checks passed.
It does not establish that a robot workflow has been tested.
_Avoid_: Robot certified, all workflows working
