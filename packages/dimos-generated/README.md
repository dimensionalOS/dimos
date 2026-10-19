# Built-in message packages

Git keeps the 14 DimOS definitions under `custom/dimos_msgs`, the supported
`catalog.json`, and `schema-sources.json` with pinned revisions, archive hashes,
selected-file hashes and license paths. Bulk upstream `.msg` files are not Git
inputs. Explicit preparation puts their selected closure in ignored `interfaces/`.
Python aliases/stubs and native source projects are also ignored build outputs.

## Supported types

`catalog.json` lists explicit roots and a reason for every root. Builds
resolve their complete dependency closure. Adding another upstream `.msg` to the
source tree does not silently add an installed type.

The default package exports 73 types. The starting set is the 63 static consumer
types audited at published runtime commit
`59ce3b8ddb2aedcecd8ca0c592b64e2824899874`. The additional roots preserve:

- The two remaining project-owned bounding-box array messages. This layout
  change does not retire DimOS's own message contracts.
- `Empty` and `Int64` used by the dynamic topic/CDR fixtures.
- `UInt32MultiArray` and its layout dependencies used by strict array/truncation
  acceptance.
- `SolidPrimitive`, `Polygon` and `Point32` used by the existing strict bounds
  acceptance. Python construction remains available, but encoding/decoding this
  bounded type remains unsupported. CDR-L07/L08 assertions remain strict xfails.

All 14 project-owned types remain exported. The pinned source lock selects the upstream definitions needed by this package.
Other upstream types are not part of the default installed API. This is a deliberate proposal API reduction from
153 exports, not a claim that static imports prove every other type unused.
Dynamic topic lookup and typed replay accept installed, supported type names.
External packages can reuse the exported dependency-owned types, including the
same Header identity. We do not promise an arbitrary omitted standard type or
split ROS-package ownership across Rust crates. There is no core/extras scheme.

## Install matching review artifacts

The proposal packages are not assumed to exist on PyPI. Put matching backend and
built-in wheels plus their Python dependencies in a wheelhouse:

```sh
pip install --no-index --find-links review/wheels dimos-generated==0.1.0
```

Wheel installation needs no ROS, CMake, Cargo or C/C++ compiler. Import performs
no download or native compilation. Python uses rosbags classes and CDR codecs:

```python
from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.std_msgs.msg import Header
from dimos_cdrgen.registry import encode, decode

header = Header(Time(17, 23), "map")
assert decode(encode(header), Header) == header
```

## Explicit source preparation

Prepare the backend parser first, then prepare builtin inputs as a separate action:

```sh
python -m scripts.prepare_builtin_sources
# Reuse cached, hash-checked archives without network:
python -m scripts.prepare_builtin_sources --offline
```

The preparation script delegates archive/file validation to PR1's generic
`source_inputs.prepare_sources`. It adds the local custom inputs and licenses.
It never runs inside pip hooks or runtime imports. An unprepared checkout fails
with the missing-input path instead of fetching data. Offline development needs
these archives and Python dependency wheels cached beforehand. Release preparation
performs the same checks before producing self-contained source/wheel artifacts.
The generic backend retains only the first layer's small conformance fixtures,
not the full builtin catalog. Product data do not select backend behavior.

## Source and editable builds

PEP 517/660 calls the same `generate_builtin()` implementation used by native
source packaging. It emits Python files and native source projects, but never
invokes a native compiler. A source build needs the matching
`dimos-cdrgen` wheel, setuptools/wheel and their dependencies. An offline
build needs these in its wheelhouse:

```sh
pip install --no-index --find-links review/wheels ./packages/dimos-generated
pip install --no-index --find-links review/wheels -e ./packages/dimos-generated
```

After editing a custom message or catalog, repeat source preparation and the editable install. Restart
Python processes. There is no import-time source build. A clean checkout requires explicit source preparation before a pip build. A
release sdist carries its own `interfaces/` inputs, catalog, pins and licenses. It can rebuild a wheel outside the
checkout without GitHub access. The checkout-only uv dependency path is removed
from release sdists.

The generator backend itself retains the explicit upstream parser preparation
contract from layer 1. Preparing that backend is a developer/release step, not
an implicit download inside this package's pip hook.

## Maintainer artifacts and checks

```sh
python -m scripts.generate_builtin_messages
python -m scripts.generate_builtin_messages --check
```

Outputs live in `packages/dimos-generated/build/message-source/python`. The check
compares independent regeneration with those build artifacts and does not repair
them. CI mutation tests retain field/schema/version/deletion/rename and missing
resource checks. Undeclared source additions require a catalog decision.

`bash scripts/package_messages.sh` creates the explicit C++ source archive and
Cargo crate using the same roots/closure. It invokes Cargo as a native packaging
step. Consumers still supply their native toolchains and prepared dependencies.
MCAP writers can embed the complete schema closure of a supported type even when
its nested values come from another package.
