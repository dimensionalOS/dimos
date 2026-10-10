# Keep built-in definitions and generated packages aligned

Built-in message definitions live in `dimos/message_codegen/schemas/<package>/msg/*.msg`.
The distribution version comes from `packages/dimos-generated/pyproject.toml`.
These are the inputs; installed packages and generated output are never used as
the definition source for the alignment check.

Python value classes, C++ headers/CMake sources, Rust crate sources, and complete
schema closures are **checked in** under `packages/dimos-generated/src/`:

```text
dimos_generated/                         # Python classes and pure CDR runtime
dimos_generated_schemas/schemas/         # canonical .msg closure
dimos_generated_schemas/package/cpp/     # C++ header codec and CMake exports
dimos_generated_schemas/package/rust/    # Cargo manifest and Rust sources
```

CI packages those sources as Python wheels/sdists and native source artifacts.
C++ and Rust consumers compile the distributed sources with their normal
toolchains; they do not regenerate standard messages. Python wheel installation
does not require a message compiler.

## Check locally

Prepare the pinned formatter explicitly and install only the locked generation
dependencies, without the runtime or native message toolchains:

```sh skip
rustup toolchain install 1.92.0 --component rustfmt
uv sync --only-group message-codegen --frozen
.venv/bin/python -m scripts.generate_builtin_messages --check
```

Use a prepared uv cache and `--offline` for an offline dependency install. The
check invokes the already installed Rust 1.92.0 formatter; it does not install
system packages or download a toolchain. Ruff and Python dependencies resolve
from `uv.lock`.

Each check generates fresh Python/C++/Rust/schema outputs in a temporary directory
and compares every relative filename and byte against the checked-in output tree.
Changed content, missing files, stale deleted types and extra **untracked** files
all fail. Only Python cache/egg-info metadata is excluded. No incremental cache or
comparison against another copy of generated output can make stale source pass.

After editing a definition, generator/template, or distribution version, repair
the outputs explicitly and review both changed and newly generated files:

```sh skip
.venv/bin/python -m scripts.generate_builtin_messages
.venv/bin/python -m scripts.generate_builtin_messages --check
git diff -- packages/dimos-generated/src
git status --short -- packages/dimos-generated/src
```

Commit definitions and generated changes together. Restart Python processes
after regeneration; an already imported class does not update in place.

## CI enforcement

The main `ci` workflow runs **Built-in message alignment** on every pull request,
main push, merge-group run and manual dispatch. Its failed result fails the
existing `ci-complete` aggregate. It is not an allowed skip. No branch protection
or ruleset changes are made by this proposal.

The `message-codegen` workflow also runs **Reject checked-in generated source
drift**. Its relevant input paths include schemas, generator/templates, package
metadata/output, root `pyproject.toml`, `uv.lock`, and the check/workflow itself.
Its existing three-language and independent ROS2 wire/schema tests complement
source alignment; a byte-for-byte check alone does not prove codec correctness.

Regression tests deliberately alter a field, schema text, add/remove a type,
change generator output and package version, and remove Python/C++/Rust/schema
outputs or add an untracked output. Each must fail before explicit regeneration
and pass after repair. Version repair is checked in Python, CMake and Cargo.
The dedicated CI job requires the pinned formatter and executes every case;
ordinary runtime test environments without that toolchain report an explicit
prerequisite skip. The mandatory drift command never skips missing tools.

CI has read-only repository permissions for this check. It fails with the repair
command; it does not commit generated changes, use a persistent write token, or
silently replace definitions during packaging.
