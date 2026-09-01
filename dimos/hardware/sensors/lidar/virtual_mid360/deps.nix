# A workspace member, so cargo needs it present, but it builds through its own
# flake rather than the shared derivation.
_: {
  sources."dimos/hardware/sensors/lidar/virtual_mid360" = ./.;
}
