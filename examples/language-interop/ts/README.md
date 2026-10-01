# TypeScript and browser messages

The previous `@dimos/msgs` LCM-codec CLI and web examples have been retired.
The supported browser path uses advertised ROS message schemas and CDR through
the [web SDK](/docs/web/web_sdk.md) and [bridge](/docs/web/bridge.md).

For a bounded three-language relay over raw LCM or Zenoh, use the
[Python, C++ and Rust examples](/examples/message-codegen/README.md). This change
is an intentional wire/API break; the old typed LCM payloads are not accepted.
