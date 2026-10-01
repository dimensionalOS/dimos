# Define and use messages

Choose the workflow that owns your definitions:

- [Develop a built-in message in this checkout](/docs/development/messages-in-repository.md): edit canonical `.msg` inputs, rebuild the independent message package, test consumers, and let CI distribute artifacts.
- Installed dimOS users author a separate external message project. Its complete Python/C++/Rust tutorial accompanies the runtime-cutover layer; ordinary built-in users only install matching prebuilt packages.

Both workflows share [message contracts](/docs/development/message-reference.md):
type ownership, language builds, schema distribution and generated values versus
runtime helpers. These APIs currently belong to the CDR proposal, not a public
release. `dimos build` targets external projects; it is not the built-in rebuild
command.
