# Run VLN-CE as a live-agent condition

The first navigation benchmark uses official VLN-CE R2R task binding and native
scoring under the named `dimos_geometry_training_scene_development` condition.
The public boundary exposes RGB, depth-derived geometry, odometry, the complete
navigability map, and ordinary DimOS navigation RPCs. The private container
retains the reference path, privileged metrics, and terminal result.

The evaluator prepares Habitat, DimOS, Memory2, and the live Pi session before
crossing one start barrier. Habitat and Pi then share the case's 300-second
wall-clock horizon. Pi may make repeated `python_exec` calls in one persistent
workspace. Only `submit_route()` represents VLN-CE STOP; reaching an internal
waypoint does not end the route. Timeout also invokes official STOP scoring.

The runtime always attempts native video recording. Video is presentation
evidence, so recording failure produces diagnostic metadata while the official
result remains valid. The host validates the container's result against a
pinned JSON schema and exact attempt identities; it does not reimplement the
official metrics.

This public episode proves the integration end to end. Its complete geometry
and training-scene inputs make it unsuitable for comparison with standard
VLN-CE validation, test, or leaderboard results.
