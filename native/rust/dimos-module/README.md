# Native module SDK

`#[tf]` binds standard `tf2_msgs/TFMessage` streams named `tf` and `tf_static`.
Both ports must appear in the native module registry and Python wrapper.
The SDK uses generated `TransformStamped` and `Time` values directly. TF
fields accept no codec parameters; CDR is the default wire format.

`lookup_transform(target, source, time)` maps source coordinates into the
target frame. ROS zero time selects the latest common time. `lookup().at()`
also takes a ROS `Time`; `within()` waits for missing data. Lookups interpolate
and reject extrapolation. The `transforms` library owns the frame tree,
history, composition and interpolation. Its current registry pins each child's
parent and static/dynamic kind; changing either requires rebuilding the buffer.

`publish()` sends dynamic transforms and `publish_static()` sends static
transforms. Static edges never expire from a receiving buffer and do not
constrain the common time of a dynamic chain. Invalid batches do not change
the graph or enqueue a packet. A failed publication queue leaves the graph
unchanged.

The SDK's LCM and Zenoh transports do not provide ROS DDS transient-local
history. A static publisher must remain available and reannounce its complete
mount set for late subscribers. Existing Python static-mount publishers still
reannounce on dynamic `tf` for compatibility with the Python query layer; they
have not been migrated to `tf_static` in this SDK change.

Older modules pinned to SDK 0.1.0 retain their explicit legacy wire path. SDK
0.2.0 does not fall back to legacy TF decoding.

Message definitions come from the existing built-in message pipeline in
`packages/dimos-generated/schema-sources.json`. Before building the SDK from
source, install the project to generate `build/message-source/rust`. The SDK
and native consumers depend on that same `dimos-generated-messages` crate;
TF messages have no separate definitions or generation command.

### Generated message boundary

SDK 0.2 uses `dimos-generated-messages` from the prepared
`build/message-source/rust` crate. Point-cloud helpers accept its
`sensor_msgs::msg::point_cloud2::PointCloud2` directly. The SDK no longer depends
on `lcm-msgs`; the LCM network transport still carries the same standalone CDR
payloads as Zenoh.

For a standalone module, stage the module, SDK, and generated crate together
with `native/rust/module-source.nix` or
`dimos.message_codegen.native_build.stage_rust_module_source`. Both preserve
repository-relative Cargo paths. Prepare message sources explicitly before
staging; staging does not download dependencies or install tools.
