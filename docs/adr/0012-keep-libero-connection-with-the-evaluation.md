# Keep LiberoConnection with the evaluation

`LiberoConnection` is an ordinary DimOS `Module`, but its domain role is exclusively to adapt the LIBERO container's Policy Interface into the LIBERO-PRO evaluation blueprint. It therefore lives under the in-repo LIBERO-PRO evaluation rather than the general robot/manipulator connection hierarchy. We do not create or support a speculative standalone use; the module remains composable and testable through normal DimOS interfaces inside the fresh debug and measured blueprints that need it.
