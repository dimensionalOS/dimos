# Built-in message sources

**WIP:** These messages work in standalone examples. CDR transport and application
support are incomplete. Existing code continues to use `dimos.msgs`.

The source files are:

- `custom/dimos_msgs/`: message definitions owned by DimOS.
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
Encoding of bounded types remains unsupported.
