# BEHAVIOR uses explicit whole-robot control ownership

R1 direct commands, native action vectors, and upstream primitives share one
continuous simulator with one explicit control owner. Takeover cancels the previous
operation, clears commands, and holds measured state; primitive completion keeps
primitive ownership. This prevents a stale planner or teleop stream from moving the
robot immediately after a manipulation action.

Physical and symbolic primitives require an explicit execution kind, recorded in
each episode. They never substitute for each other: task success from direct state
changes must remain distinguishable from success through physical manipulation.
