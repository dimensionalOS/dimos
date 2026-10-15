# Language interoperability with CDR

Generated `.msg` value types use encapsulated CDR over raw LCM or Zenoh.

- [Python, C++ and Rust relay, recording and replay](/examples/message-codegen/README.md)
- [Add and use a message from each language](/docs/development/messages.md)
- [C++ raw-LCM virtual robot controller](cpp/README.md)
- [Browser CDR messages](ts/README.md)
- [Lua support status](lua/README.md)

The previous generated LCM-message examples are retired. This proposal changes
the typed wire format and API; it does not remove either transport or promise
compatibility with historical typed recordings.
