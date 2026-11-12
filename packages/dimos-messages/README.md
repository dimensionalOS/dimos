# Built-in message sources

The built-in ROS message definitions live in this directory and are vendored
into Git, together with their upstream licenses:

- `interfaces/`: message definitions and license files. `interfaces/dimos_msgs`
  is owned by DimOS; the other packages are vendored upstream definitions.
- `schema-sources.json`: upstream repository, revision, archive hash and
  per-file hashes for every vendored package.

Builds read these committed files only. There is one explicit maintainer
operation to refresh them from the pinned upstream revisions:

```sh
python -m dimos.message_codegen.sources --update
```

`dimos.message_codegen.sources` verifies every downloaded archive and file
against `schema-sources.json` before writing. Builds never call it.

`setup.py` generates the Python messages from `interfaces/` through
`dimos-cdrgen==0.1.2`. The wheel contains the generated `dimos_messages`
package, the `dimos_messages_schemas` resources (schemas, C++/Rust build
inputs, `schemas.json` and `message-package.json`), and the upstream licenses.
Wheel installation does not generate code or compile native examples. There is
no separate `dimos-generated` distribution: the main `dimos` distribution owns
the built-in messages.

See [`examples/message-package/README.md`](/examples/message-package/README.md)
for install commands, example commands and output directories, and
[`docs/development/message-source-packages.md`](/docs/development/message-source-packages.md)
for the generator integration. To add a message to the built-in package, add a
`.msg` file under `interfaces/dimos_msgs/msg` and reinstall; `dimos_msgs` is the
package DimOS owns, and every other directory under `interfaces` is vendored.

After you change a definition, reinstall the package. Keep the upstream license
files; generated code and wire formats do not change when only comments change.
Encoding of bounded types remains unsupported.
