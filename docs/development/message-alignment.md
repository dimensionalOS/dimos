# Verify prepared built-in inputs and artifacts

PR1 owns the generic source-package hooks, dependency ownership and pinned source
preparation. PR2 owns `packages/dimos-generated/custom`, `catalog.json`,
`schema-sources.json`, package metadata and thin preparation/release entrypoints.
There is no committed generated snapshot or bulk upstream catalog.

```sh skip
python scripts/prepare_message_parser.py
python -m scripts.prepare_builtin_sources
python -m scripts.generate_builtin_messages
python -m scripts.generate_builtin_messages --check
```

Source preparation checks archive and selected-file hashes. Use `--offline` with
cached archives. It is an explicit developer/release step, never a pip hook.
Only the selected message closure plus licenses is materialized into ignored
`packages/dimos-generated/interfaces`. The release sdist includes those inputs.

The artifact check regenerates into temporary storage and compares file names and
bytes with `packages/dimos-generated/build/message-source/python`. It does not
repair changed, missing or stale artifacts. CI prepares inputs first, then runs
both regeneration and mutation checks. Python dependencies and native tools are
prepared separately; these checks do not install system tools or change hosts.

Regression tests cover fields, schemas, package version, explicit root addition,
deletion/rename, extra files and missing Python/C++/Rust/schema outputs. Cold offline
source/wheel/sdist/editable tests block native tools and verify type identity,
complete dependency schemas, field edits and renames. Codec assertions and precise
strict expected failures remain separate from artifact alignment.

The checks retain read-only repository permissions. They do not commit outputs,
query CI status or prove complete native/runtime/hardware acceptance.
