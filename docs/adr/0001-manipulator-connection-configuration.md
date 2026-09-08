# Manipulator connections belong to the robot assembly

Manipulator connection settings belong to robot-specific coordinator configs. Single-arm configs share an optional `address` field exposed through the existing blueprint configuration system as `--address` when unambiguous. Assemblies requiring multiple endpoints expose explicit local fields and own their validation. Robot hardware factories resolve these settings after module configuration has been parsed; the base `ControlCoordinatorConfig` does not gain a universal address field.

Outside simulation, no addresses selects mock hardware, all required addresses selects physical hardware, and a partial set is a configuration error. This all-or-none rule applies to both coupled robots and existing assemblies of independent arms. Mixed physical/mock operation is not part of this change. It keeps the assembly's operating mode predictable without requiring separate mock blueprint names.

A supplied address explicitly requests physical hardware. Empty or invalid addresses are configuration errors; an unreachable device or other connection failure stops startup without falling back to mock. If an assembly fails partway through connection, startup cleanup must disconnect devices already connected and release any resources acquired by the failed connection attempt.

The existing global `--simulation` interface remains in scope as a preserved runtime behavior. Device-specific `xarm6_ip`, `xarm7_ip`, `can_port`, and `device_path` fields are to be removed from `GlobalConfig`, with no compatibility aliases. Shared single-arm configuration should remain small; assembly-specific selection rules stay in robot factories rather than a process-wide resolver.
