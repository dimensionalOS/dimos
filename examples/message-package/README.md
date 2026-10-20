# Verify messages in the installed DimOS wheel

Build the layer-2 DimOS wheel, install it into a clean environment,
then run outside the checkout. This example constructs an upstream `Image` and
the existing custom `LineSegments3D`, checks their CDR roundtrips, and inspects
the packaged definitions and native build inputs.

Use Python 3.12 and a C++ compiler (for DimOS's existing A* extension).
From this PR's checkout, install its commit into your active environment and run:

```sh
python -m pip install "dimos @ git+https://github.com/dimensionalOS/dimos.git@$(git rev-parse HEAD)"
python -I examples/message-package/demo_installed_wheel.py
```

This uses standard isolated source installation: the backend prepares hash-verified
message definitions, generates Python modules, and builds missing Web bundles
using the declared Deno dependency. You do not need to publish DimOS, run a
preparation script, or install ROS/CMake/Rust. Network access is needed on a cold
source build. Installing a prebuilt wheel performs none of these build steps.

Expected output, with your temporary directory in the first line:

```text
Installed package: <temporary-directory>/venv/lib/python3.12/site-packages/dimos/messages/__init__.py
Catalog: 73 types
PASS: sensor_msgs/msg/Image pixels, encoding and source stamp
PASS: dimos_msgs/msg/LineSegments3D geometry, weight and nested type identity
PASS: packaged definitions and C++/Rust build inputs; no compiled message libraries
```

`-I` ignores `PYTHONPATH` and the script directory during imports. The script
also verifies that the imported package lives inside the clean environment
and that the owning DimOS distribution is installed. It uses the installed
wheel rather than an editable checkout.

## What the package contains

Git stores 14 custom definitions, `catalog.json`, upstream pins, hashes, and
thin build entry points. Preparation copies the selected upstream definitions
and custom inputs into ignored `packages/dimos-generated/interfaces/`.
The root `setup.py` calls `dimos_cdrgen.source_package.setup_arguments` to generate and
package Python imports and stubs, complete schemas, C++ CMake projects, and
Rust Cargo inputs. Those generated files are shipped in the wheel, not checked
into Git. There is no separately published built-in Python message distribution.
A source distribution also carries the prepared `.msg` inputs.

At this layer's head, 69 catalog roots resolve to 73 types: 59 upstream and
14 custom. This is an explicit supported selection, not proof that every
message used elsewhere in the repository is covered. Later stack layers add
further types.

The installed DimOS wheel includes these messages alongside the main runtime.
Messages use `dimos-cdrgen`, rosbags, and NumPy; an ordinary DimOS installation
also installs its other runtime dependencies. Running this example does not fetch `.msg` files or compile
native bindings. For offline installation, make those dependency wheels
available locally and pass `--no-index --find-links <wheelhouse>` to
`uv pip install`. Offline source builds use `DIMOS_OFFLINE=1` and require cached archives and
locked Deno dependencies unless prepared inputs/bundles are already present.

MCAP recording is a separate layer. The existing
[layer-3 example](https://github.com/dimensionalOS/dimos/blob/3cb96b780433dcdac605f69453721aac2df1d800/examples/message-codegen/demo_mcap.py)
writes messages and their schemas into a file for external readers. It is not
required for this wheel check.
