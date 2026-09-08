# DimOS

DimOS composes modules into runnable robot stacks whose deployment-wide behavior and device-local configuration have distinct ownership.

## Language

**Device address**:
A module-local endpoint used to connect one configured hardware adapter to its physical device, such as an IP address, CAN interface, or serial path.
_Avoid_: Global robot IP, global CAN port, global device path

**Connection selection**:
The robot assembly's selection of mock, physical, or simulated hardware according to its connection settings and the runtime simulation mode.
_Avoid_: Adapter fallback, implicit hardware mode

**Robot assembly**:
A configured robot or group of robots operated together, whose hardware connections are selected and validated as a unit. An assembly may contain one arm, independent arms, or coupled arms sharing one adapter.

**Simulation mode**:
A process-wide runtime mode that allows a blueprint to select simulated modules and adapters in place of physical hardware while preserving the same runnable blueprint interface.
_Avoid_: Manipulator simulation config, adapter mode
