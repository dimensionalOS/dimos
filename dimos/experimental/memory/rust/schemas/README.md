# Recording schemas

These ROS 2 Jazzy message definitions are bundled as MCAP `ros2msg` schemas,
including every transitive dependency separated by `MSG: package/Type`.
Dependency block names match the `package/Type` field references in upstream
`.msg` files; the MCAP Schema record name remains `package/msg/Type`.
Foxglove resolves these references literally, so mixing the two spellings inside
a definition leaves its dependencies unresolved.
They are compile-time assets; building the recorder requires no ROS installation
or schema download, and the wire structs in `src/ros.rs` follow their field order.

Upstream snapshots (Jazzy branches, retrieved 2026-09-20):

- [common_interfaces](https://github.com/ros2/common_interfaces/tree/a941f14bb318d8d904505ed935ccbb97f24a70a4) (`a941f14bb318d8d904505ed935ccbb97f24a70a4`).
- [rcl_interfaces](https://github.com/ros2/rcl_interfaces/tree/7aa3caf43377ea6ad615bc1040832e2c7566bfbe) (`7aa3caf43377ea6ad615bc1040832e2c7566bfbe`).
- [geometry2](https://github.com/ros2/geometry2/tree/f702874b1c8535d6a038230ab2cda0ba5d521ebd) (`f702874b1c8535d6a038230ab2cda0ba5d521ebd`).

`sensor_msgs`, `geometry_msgs`, `nav_msgs`, and `std_msgs` come from
`common_interfaces`; `builtin_interfaces/Time` comes from `rcl_interfaces`;
`tf2_msgs/TFMessage` comes from `geometry2`.
Definitions and upstream comments are preserved, with concatenation added and
trailing whitespace normalized.
The adjacent `LICENSE-ros-*` files carry their upstream licenses (Apache 2.0 for
common/rcl interfaces, BSD 3-Clause for geometry2).

`LineSegments3D.json` is the DimOS JSON Schema for weighted line segments.
Its JSON payload contains `ts`, `frame_id`, paired XYZ endpoints in `segments`,
and one entry in `weights` for each segment.
