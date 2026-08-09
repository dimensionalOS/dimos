# Pick And Place

`PickAndPlaceModule` is a robot-independent high-level workflow. It composes
object-scene, grasp-provider, candidate-filter, and execution capabilities;
`ManipulationModule` supplies the execution and planning capabilities in robot
blueprints.

The canonical API is `scan_objects(prompts)`, `get_object(object_id)`,
`select_grasp(object_id, rank=0)`, `get_grasp_candidates()`,
`pick_selected(robot_name=None)`, and `place_at(..., robot_name=None)`.

The module deliberately does not estimate tabletops, install scene geometry,
measure containers, derive box placement targets, or publish visualization.
