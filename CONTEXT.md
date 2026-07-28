# Manipulation Planning

This context defines the language used to describe geometric robot-motion planning
independently from orchestration and caller-facing convenience APIs.

## Language

**Planner interface**:
The internal contract that receives a fully resolved planning problem and returns a
geometric planning result. It does not resolve live state, caller-friendly selectors,
execution, or presentation concerns.
_Avoid_: Public planning API, user-facing planner API

**Planning facade**:
The caller-facing orchestration boundary that resolves planning context and delegates a
fully formed problem to the planner interface.
_Avoid_: Planner interface, planning backend

**Linear Cartesian path**:
A TCP path whose position is linearly interpolated and whose orientation is spherically
interpolated between explicit start and target poses.
_Avoid_: Linear joint path, constrained path

**Cartesian target**:
Either an absolute stamped TCP pose or a Cartesian delta resolved from the explicit
planning start state. Each target identifies the endpoint of one planning-group TCP
track.
_Avoid_: Cartesian path, goal configuration

**Cartesian delta**:
A world-frame translation and rotation applied to the TCP pose calculated from the
explicit planning start state. Translation follows world axes and rotation is
pre-multiplied (`R_target = R_delta @ R_start`); a zero rotation preserves the starting
TCP orientation.
_Avoid_: Tool-frame command, path constraint
