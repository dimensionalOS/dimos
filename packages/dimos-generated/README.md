# Built-in message sources

These definitions generate the `dimos.messages` classes used by typed transports
and recordings. Numerical helpers live in `dimos.message_utils`.

The source files are:

- `custom/<package>/msg/`: local message definitions, including `dimos_msgs` and `control_msgs`.
- `custom/<package>/LICENSE`: the license copied with each local package.
- `schema-sources.json`: upstream revisions, file hashes, and license paths.

cdrgen discovers all `.msg` files in the prepared source directory.

For source and editable installs, `setup.py` checks the definitions and calls
`dimos-cdrgen`. The wheel includes Python messages, schemas, native source code,
and licenses. Wheel installation does not generate code or compile native examples.
There is no separate `dimos-generated` distribution.

See the [built-in example tutorial](/docs/development/message-source-packages.md)
for install commands, example commands, and output directories.

After you change a definition, install the package again.
Then restart Python. Keep the upstream license files.
External projects can define and register their own message packages. See the
[external message guide](/docs/development/messages-external-project.md).
Use the generator's supported-schema diagnostics for encoding limitations.
