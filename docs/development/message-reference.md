# Message contracts

This reference applies to both built-in and external messages in the CDR
proposal. Definitions are `.msg` files, with identities such as
`dimos_msgs/msg/DeviceReading` or `story_msgs/msg/DeviceReading`. Generation
produces Python pybind11 values, C++ declarations/codecs and Rust values/codecs;
language builds compile those outputs. There is no import-time compilation and
no ROS installation requirement. CDR/schema compatibility does not mean that
this package builds a ROS node.

## One owner for a dependency type

A field declared `std_msgs/Header header` uses the built-in package's Header in
all languages. External packages import the same Python class, C++ declaration
and Rust dependency type instead of creating another identity. Constructors and
field assignment preserve that type but copy the Header value; object identity
is not promised. Exact dependency versions, definitions and binding ABI must
agree. Conflicting owners or dependencies fail the build. Runtime schema-hash
routing enforcement is not implemented; do not rely on it to detect every
same-name schema mismatch.

External project defaults discover `interfaces/<package>/msg/*.msg` and derive
the Python module name from the distribution name in `pyproject.toml`.
`dimos-message-build` provides the PEP517/660 backend shared with `dimos build`.
`pip install .` in that project builds only Python; `dimos build` builds its
configured language outputs. Build isolation installs declared Python build
dependencies, not OS toolchains or the dimOS runtime.

## Generated values and runtime modules

Generated fields follow the `.msg` layout: `header.stamp.sec`/`nanosec`, nested
poses and explicit image dimensions/encoding/data. Python values expose
`encode()`, `decode()`, `msg_name` and full `schema`; these are useful for codec
checks. Normal modules publish and receive typed values through SDK ports rather
than manually encoding bytes. Math, image/cloud conversion and visualization
helpers live outside the generated value classes.

The runtime SDK supplies Python `Module` with `In`/`Out`, C++
`Module`/`Builder`/`Output<T>` and Rust `Module`/`Input<T>`/`Output<T>` with CDR
adapters. These runtime APIs belong to the runtime-cutover layer; standalone
message builds do not install the complete native SDK. LCM and Zenoh remain
transports: CDR replaces typed encoding.

## Distribution and recordings

Built-ins are built in CI and distributed as prebuilt language artifacts.
External authors distribute their wheel/sdist, CMake archive and Cargo source
bundle with matching dependency versions. Current proposal packages are review
artifacts, not an assumed public-index release. Pip does not honor a checkout's
`[tool.uv.sources]`; provide matching distributions explicitly.

Providers export owned types and the complete dependency schema closure. MCAP
embeds CDR bytes and full `ros2msg` schemas even when dependency value types are
imported from another package. Viewers need no installed custom package; typed
replay needs matching installed types. Known native stamped layouts preserve
source time. Unknown external native layouts currently use reception time while
retaining the complete payload Header.

Changing a wire layout requires matching rebuilt packages. The proposal is a
deliberate old-message API/wire break; transparent legacy-recording decoding and
mixed old/new typed deployments are not supported.
