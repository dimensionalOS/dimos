---
title: "Piper Integration"
description: "Connect and run a Piper arm with DimOS manipulation and teleoperation blueprints."
---

## Optional SLCAN setup

Use this separate path only with a serial-CAN adapter, such as `/dev/ttyACM0`;

```bash
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
```

## Bring up the Piper CAN interface

Piper uses SocketCAN at 1,000,000 bit/s. For the default vendor setup, use
DimOS's vendored copy of the upstream activation helper:

```bash
bash dimos/robot/manipulators/piper/scripts/can_activate.sh can0 1000000
```

If the device already exposes `can0`, run the vendored helper directly. Verify
the interface before starting a blueprint:

```bash
ip link show can0
```

## Run a Piper blueprint

Use the coordinator for the basic manipulation composition:

```bash
dimos --can-port can0 run coordinator-piper
```

For keyboard Cartesian teleoperation, use:

```bash
dimos --can-port can0 run keyboard-teleop-piper
```

The Quest teleoperation composition is available as:

```bash
dimos --can-port can0 run keyboard-teleop-piper
```

Note that ommitting the `--can-port` argument will fallback the control coordinator to use fake hardware adapter. This is good for testing.
