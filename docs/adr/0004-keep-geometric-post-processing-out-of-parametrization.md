# Keep geometric post-processing out of trajectory parametrization

Trajectory parametrization converts an accepted geometric path into a timed trajectory under motion limits. It may perform bounded interpolation or curve fitting needed to define continuous motion between the supplied waypoints, but it does not rewrite the source path through shortcutting, waypoint simplification, or path-class-specific resampling; those operations belong to plan generation or its post-processing stage.
