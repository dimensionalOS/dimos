# Forward Python-native RPC through the host contract

Python-native modules use a host Contract and an isolated Runtime Subclass. The Contract keeps the module's public RPC identity and forwards its declared RPCs and skills to a uniquely named internal DimOS RPC endpoint in the runtime. This design preserves contract inheritance, normal module references, RPC serialization, and native-style process management without adding an `external-python` deployment manager or a second RPC protocol.

The alternatives were to expose the child directly through a dedicated worker manager or to add a private socket protocol. Direct exposure split lifecycle and application RPCs across proxies and required coordinator-specific deployment behavior. A socket protocol duplicated DimOS serialization, errors, timeouts, and async behavior. The internal endpoint reuses the RPC system's existing nested-call support.
