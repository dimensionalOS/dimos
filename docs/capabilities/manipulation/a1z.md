# Galaxea A1Z

The A1Z integration uses the vendor's 250 Hz position-control loop, the G1Z
gravity model, and the G1Z gripper. The SDK is not published on PyPI, so install
the pinned Git revision until the vendor publishes a compatible release.

## Install from a source checkout

The repository setup script shows its complete plan and asks for confirmation
before it changes the checkout environment or installs system packages:

```bash
bin/hardware/a1z/setup
```

The script runs a locked, inexact sync so it preserves other extras in the
checkout. It installs `can-utils` on Ubuntu and `libusb` through Homebrew on
macOS when needed. On other Linux distributions, it prints the missing system
package instead of selecting a package manager for you.

## Install into an existing environment

Install these requirements with the package manager that owns the environment:

```bash
python -m pip install 'dimos[manipulation]'
python -m pip install 'a1z @ git+https://github.com/userguide-galaxea/GALAXEA-A1Z.git@e931ecd0e25ad35df251097ba42921b3d2fa7224'
```

On macOS, also install the userspace USB dependencies:

```bash
python -m pip install 'gs-usb==0.3.1' 'pyusb==1.3.1'
brew install libusb
```

On Linux, install the package that provides `cansend`. Ubuntu calls this package
`can-utils`:

```bash
sudo apt-get install can-utils
```

Check the installation without changing the host:

```bash
dimos hardware a1z doctor --software-only
```

## Linux setup

Connect and power the arm, then configure its CAN interface:

```bash
dimos hardware a1z configure-can
```

The command asks for confirmation, then uses `sudo` to:

1. Load the Linux `gs_usb` kernel driver.
2. Bind the HHS USB-CANFD adapter.
3. Create the stable `a1zcan` SocketCAN interface at 1 Mbit/s.
4. Send a safe probe and verify that the adapter transmitted it.

Do not start the arm unless configuration reports a successful transmission.
Run the full read-only diagnostic afterward:

```bash
dimos hardware a1z doctor
```

After rebooting or reconnecting the HHS adapter, run `configure-can` again.

## macOS setup

macOS uses the HHS adapter directly through libusb because it does not provide
SocketCAN. The doctor verifies the gripper-capable SDK, finds the attached HHS
USB-CANFD adapter, opens it in listen-only mode, and closes it without
transmitting or enabling the arm:

```bash
dimos hardware a1z doctor
```

## Run

The A1Z has no brakes or hardware e-stop button; the PSU switch is the hardware
kill switch. Support the arm and clear its workspace before starting or
stopping dimOS. Disabling the motors makes the arm fall.

```bash
dimos run keyboard-teleop-a1z
```

This launches keyboard teleoperation with mock hardware, the control coordinator,
trajectory execution, and `ManipulationModule`. Select a CAN interface explicitly
to use the real arm. Real-hardware startup waits for feedback from all six arm
motors, validates the measured state, holds the measured pose, and then ramps
gravity compensation:

```bash
dimos --can-port a1zcan run keyboard-teleop-a1z
```

On Linux, pass another verified SocketCAN interface instead if needed:

```bash
dimos --can-port can0 run keyboard-teleop-a1z
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
  `dimos hardware a1z configure-can` on Linux or
  `dimos hardware a1z doctor` on macOS.
- **macOS cannot find or open the adapter.** Check that libusb is installed,
  reconnect the adapter, and rerun `dimos hardware a1z doctor`.
- **The gripper remains stiff after shutdown.** The adapter retries the disable
  command, but a degraded bus can still lose it. Support the arm and turn off
  the PSU.
