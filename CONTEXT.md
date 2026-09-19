# DimOS simulation language

Terms used to connect robot stacks with task-oriented simulation.

## Language

**Task instance**: A particular activity definition and initial arrangement in a scene.
_Avoid_: Episode, world snapshot

**Episode**: One execution of a task instance from initialization or reset until termination.
_Avoid_: Task definition

**Control owner**: The sole command source currently allowed to control the whole robot.
_Avoid_: Command priority

**Physical primitive**: A named action realized through robot control and simulated physics.
_Avoid_: Symbolic action

**Symbolic primitive**: A named action that may directly change simulator object poses or states.
_Avoid_: Physical manipulation

**Ground truth**: Simulator-provided object and task state, distinct from sensor-derived estimates.
_Avoid_: Perception result

**Task success**: Satisfaction of the task evaluator's goal conditions.
_Avoid_: Primitive completion
