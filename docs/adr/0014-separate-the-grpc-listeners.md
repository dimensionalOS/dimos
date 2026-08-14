# Separate the gRPC listeners

The LIBERO container binds the Policy Interface and Evaluation Control Interface to separate gRPC endpoints. `LiberoConnection` receives only the policy endpoint, while the Evaluation receives the control endpoint and its per-trial capability. This makes the privilege split structural today and allows the future policy sandbox to reach only the policy listener without redesigning the protobuf services or depending on method-level filtering at one shared address.
