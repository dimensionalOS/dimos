# Standalone CDR example

The [generator walkthrough](../../dimos/message_codegen/README.md) contains the
message definition, generation command, Python usage and native consumer contract.
The executable three-language acceptance lives in
`dimos/message_codegen/test_native_consumer.py`; run:

```sh skip
bash scripts/test_message_codegen.sh
```

It writes `input.cdr`, `cpp.cdr`, `rust.cdr` and the independent Jazzy reference
project under `build/message-codegen/demo/evidence/`. Values progress from 20.5
to 21.5 to 22.5 with the same Header fields. Each language imports its dependency's
Header type. No dimOS runtime, transport or MCAP implementation is required.

The original bounded Telemetry fixture remains unchanged as an explicit native
limitation test; it is not the supported exchange example. Prior wrapper-based
example code has been removed from this layer.
