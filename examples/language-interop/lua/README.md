# Lua message support

The previous Lua example depended on generated LCM message codecs and has been
retired with the deliberate CDR wire/API break. This proposal does not provide
a Lua message generator. Raw LCM remains a supported transport.

Use the tested [Python, C++ and Rust examples](/examples/message-codegen/README.md)
for generated CDR values, schemas and recording. Lua CDR generation is deferred;
there is no fallback decoding of the old example's typed messages.
