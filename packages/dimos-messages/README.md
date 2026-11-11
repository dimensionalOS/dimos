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

See the [built-in example tutorial](/docs/development/message-source-packages.md)
for install commands, example commands, and output directories. To add a message
to the built-in package, follow [Add a message inside the DimOS repository](/docs/development/adding-internal-message.md).

After you change a definition, reinstall the package. Keep the upstream license
files; generated code and wire formats do not change when only comments change.
Encoding of bounded types remains unsupported.
