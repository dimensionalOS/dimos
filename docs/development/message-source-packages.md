# CDR generation in dimOS

> CDR migration is still in progress. The generator is integrated on main.
> This unit adds built-in messages in [#4366](https://github.com/dimensionalOS/dimos/pull/4366).
> Runtime defaults, recording, viewers, and legacy cleanup remain separate units.

dimOS uses the published `dimos-cdrgen==0.1.1` package directly. Its parser,
generators, ownership checks, codecs and native build recipes live in the
[independent library](https://github.com/TomCC7/dimos-cdrgen).
See its [Python, C++ and Rust guides](https://tomcc7.github.io/dimos-cdrgen/)
for message package configuration and supported schema features.

Install `dimos-cdrgen==0.1.1` in both the build and runtime dependencies of
message packages. Import its APIs directly:

```python
from dimos_cdrgen.registry import decode, encode
from dimos_cdrgen.source_package import setup_arguments
```

There is no dimOS compatibility distribution. Existing consumers of
`dimos_message_build` or generic `dimos.message_codegen` APIs must change their
imports and dependency declarations to `dimos_cdrgen` and `dimos-cdrgen`.

dimOS retains two adapters:

- `dimos.message_codegen.native_build.write_cmake_toolchain` adds SDK discovery
  for installed `dimos/_native/cpp` and source-checkout `native/cpp` layouts.
  Import other build functions directly from `dimos_cdrgen.native_build`.
- `dimos/native/cdr_codec.hpp` exposes the library codec under `dimos::native`
  for dimOS C++ consumers.

This first stack layer does not switch runtime transports or supply built-in
message catalogs. Those remain separate PRs.

## Verify the integration

Use a Python environment with the published generator and test tools:

```sh
uv pip install 'dimos-cdrgen[native]==0.1.1' numpy build pytest pytest-asyncio pytest-env
python -m pytest --noconftest -o addopts='' -q dimos/message_codegen/test_integration.py
```

The tests check SDK discovery, an installed message wheel built from a source
distribution, Python type identity and the dimOS C++ header against literal CDR
bytes. Native verification needs CMake and a C++17 compiler. It prepares pinned
native dependencies in a temporary cache; use `DIMOS_NATIVE_TEST_CACHE` to reuse
a writable cache, or `DIMOS_NATIVE_SOURCE_DIRS` with `DIMOS_OFFLINE=1` for
pre-fetched source directories. Generic generator regression tests stay in the
library repository.
