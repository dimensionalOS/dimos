---
title: "Galaxea A1Z"
---

The A1Z integration uses the vendor's 250 Hz position-control loop, the G1Z
gravity model, and the G1Z gripper.

Install DimOS before setting up the A1Z. Then run one setup command as your
normal user:

```bash
dimos a1z setup
```

From a DimOS source checkout with its development environment installed, run:

```bash
uv run --no-sync dimos a1z setup
```

The command displays every dependency installation command before it asks for
confirmation. Pass `--yes` to skip the prompt. Setup installs the
`manipulation` dependencies, the pinned gripper-capable A1Z SDK, and the Python
packages required by the host platform. In a source checkout, it uses a locked,
inexact sync so that it preserves other installed extras.

## Linux setup

On Ubuntu, setup installs `can-utils` when `cansend` is missing. On other Linux
distributions, setup installs and verifies the Python dependencies, then tells
you to install the package that provides `cansend`. Rerun setup after installing
it.

After dependency checks pass, setup uses `sudo` to:

1. Load the Linux `gs_usb` kernel driver.
2. Bind the HHS USB-CANFD adapter.
3. Create the stable `a1zcan` SocketCAN interface at 1 Mbit/s.
4. Send a safe probe and verify that the adapter transmitted it.

Do not start the arm unless setup prints `A1Z CAN setup passed`. To install and
verify only the Python dependencies, run:

```bash
dimos a1z setup --sdk-only
```

After rebooting or reconnecting the HHS adapter, configure and test SocketCAN
again:

```bash
dimos a1z can-setup
```

## macOS setup

macOS uses the HHS adapter directly through libusb because it does not provide
SocketCAN. Setup installs `pyusb` and `gs-usb`. When libusb is missing and
Homebrew is available, setup also runs `brew install libusb`. If Homebrew is
unavailable, install libusb yourself and rerun setup.

The setup command verifies the gripper-capable SDK, finds the attached HHS
USB-CANFD adapter, opens it in listen-only mode, and closes it without
transmitting or enabling the arm. To verify only the Python SDK, run:

```bash
dimos a1z setup --sdk-only
```

## Run

The A1Z has no brakes or hardware e-stop button; the PSU switch is the hardware
kill switch. Support the arm and clear its workspace before starting or
stopping DimOS. Disabling the motors makes the arm fall.

```bash
dimos run keyboard-teleop-a1z
```

This launches keyboard teleoperation, the control coordinator, trajectory
execution, and `ManipulationModule`. Startup waits for feedback from all six arm
motors, validates the measured state, holds the measured pose, and then ramps
gravity compensation.

On Linux, the blueprint uses `a1zcan` by default. If you configured another
verified SocketCAN interface, pass it explicitly:

```bash
dimos run keyboard-teleop-a1z --can-port can0
```

On macOS, the adapter selects the userspace USB transport automatically; omit
`--can-port`.

## Troubleshooting

- **The interface is UP, but the arm does not respond.** Some Linux `gs_usb`
  drivers create an interface but drop every transmission through the HHS
  adapter. Update to a kernel with the endpoint-discovery fix. On Jetson or
  another pinned-kernel system, follow the
  [Galaxea driver guide](https://galaxea-ai.feishu.cn/docx/XF2ed4pmhoervNxODlfc11Gvnbb)
  and build the driver for the exact running kernel. Do not install a desktop
  kernel or copy a kernel module from another machine.
- **The bus behaves strangely after a crash.** Replug the HHS adapter and rerun
  `dimos a1z can-setup` on Linux or `dimos a1z setup` on macOS.
- **macOS cannot find or open the adapter.** Check that libusb is installed,
  reconnect the adapter, and rerun `dimos a1z setup`.
- **The gripper remains stiff after shutdown.** The adapter retries the disable
  command, but a degraded bus can still lose it. Support the arm and turn off
  the PSU.
