# DimOS

DimOS composes planning and control capabilities into runnable robot systems.

## Language

**Trajectory task**:
A coordinator's sole planned-motion activity, responsible for executing timed joint trajectories across its execution domain.
_Avoid_: JTT, JTC, joint trajectory controller

**Execution domain**:
The complete set of robots governed by one coordinator and one planned-execution authority.
_Avoid_: Independent arms, task group

**Subset plan**:
A coordinated plan that commands only some robots or joints in an execution domain. The trajectory task emits only those commanded joints.
_Avoid_: Partial execution, independent execution

**Execution target**:
A robot in an execution domain whose planned joints may contribute to coordinated execution. It describes joint-name mapping, not a coordinator task destination.
_Avoid_: Controller, task
