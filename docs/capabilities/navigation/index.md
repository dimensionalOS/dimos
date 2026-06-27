
# GO2 Navigation

![output](assets/noros_nav.gif)

The Go2 navigation stack uses simple **column-carving voxel map** strategy: each new LiDAR frame replaces the corresponding region of the global map entirely, ensuring the map always reflects the latest observations.

[Navigation Deep Dive](/docs/capabilities/navigation/deep_dive.md)

We also have a simple relocalization system on previously stored and reconstructed maps.

[Relocalization](/docs/capabilities/navigation/relocalization.md)
