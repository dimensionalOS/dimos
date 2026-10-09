# Message contracts

This reference applies to both built-in and external messages in the CDR
proposal. Definitions are `.msg` files, with identities such as
`dimos_msgs/msg/DeviceReading` or `story_msgs/msg/DeviceReading`. Generation
packages Python imports/stubs and schemas, C++ ROSIDL source projects, and Rust
crate/build-script inputs. Rosbags initializes Python classes/codecs in memory
from installed schemas. CMake and Cargo generate and compile native declarations;
Python installation needs no native compiler. Import performs no native
compilation, filesystem code generation or downloads. No ROS installation is required. CDR/schema compatibility does not mean that
this package builds a ROS node.

## One owner for a dependency type

A field declared `std_msgs/Header header` uses the built-in package's Header in
all languages. External packages import the same Python class, C++ declaration
and Rust dependency type instead of creating another identity. Constructors and
field assignment preserve that type. Python dataclasses retain the supplied
Header object; use `copy.deepcopy` for independently mutable nested values.
C++ and Rust retain their native copy/move semantics. Exact dependency versions, definitions and binding ABI must
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
`__msgtype__`; registry functions supply codecs and complete schemas:

```python
from dimos_generated.geometry_msgs.msg import Vector3
from dimos_message_build.registry import encode, decode, schema

message = Vector3(x=1.0, y=2.0, z=3.0)
assert decode(encode(message), Vector3) == message
assert "float64 x" in schema(message.__msgtype__)
```

Python and Rust constructors require all fields explicitly, including fields
with `.msg` defaults. Python numeric array fields require the declared NumPy dtype. Normal modules publish and receive typed values through SDK ports rather
than manually encoding bytes. Math, image/cloud conversion and visualization
helpers live outside the generated value classes.

The runtime SDK supplies Python `Module` with `In`/`Out`, C++
`Module`/`Builder`/`Output<T>` and Rust `Module`/`Input<T>`/`Output<T>` with CDR
adapters. These runtime APIs belong to the runtime-cutover layer; standalone
message builds do not install the complete native SDK. LCM and Zenoh remain
transports: CDR replaces typed encoding.

## Distribution and recordings

CI distributes built-in Python wheels/sdists, C++ source packages and Rust crates.
Python users install matching wheels without building dimOS. C++ source consumers
prepare the pinned upstream support libraries explicitly; prebuilt native SDK
products are out of scope. External authors distribute their wheel/sdist, native
source archive and Cargo crate with matching dependency versions. Current proposal packages are review
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

CDR input must be trusted and schema-matched. Native libraries do not uniformly
reject malformed bytes or enforce every bound; unsupported bounded Python schemas
raise `NotImplementedError`. The [accepted limitations](/docs/development/message-limitations.md)
list precise strict expected failures. Do not use this decoder as a hostile-input
validation boundary.

Rust packages contain owned `.msg` inputs, a small `build.rs` adapter and the
codec/schema contracts. Normal `cargo build` invokes pinned `ros2msg` to generate
the declarations in Cargo's `OUT_DIR`; `src/lib.rs` includes that result. This
also applies to the Rust part of `dimos build`. Cargo and its dependency cache
are native build prerequisites, not Python installation prerequisites. No helper
binary, ROS installation or runtime download is required. Dependency types are
reexported from their owner crate, preserving exact cross-package identity.
