# Piper Integration

## Optional SLCAN setup

Use this separate path only with a serial-CAN adapter, such as `/dev/ttyACM0`;

```bash
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
```

This is a separate prerequisite for serial-CAN adapters. It is not needed when
the Piper adapter already exposes a native SocketCAN interface.

## Bring up a native Piper CAN interface

Piper uses SocketCAN at 1,000,000 bit/s. For the default vendor setup, use
the dimOS CLI to configure an existing CAN interface and bring it up:

```bash
dimos hardware can setup can0
```

For a non-default bitrate, pass `--bitrate` explicitly:

```bash
dimos hardware can setup can0 --bitrate 500000
```

The command prints each privileged operation before requesting sudo. Verify the
interface before starting a blueprint:

```bash
dimos hardware can status can0
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
dimos --can-port can0 run teleop-quest-piper
```

Note that ommitting the `--can-port` argument will fallback the control coordinator to use fake hardware adapter. This is good for testing.
