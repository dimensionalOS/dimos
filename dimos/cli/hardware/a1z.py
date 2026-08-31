# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Diagnostics and host configuration for the Galaxea A1Z."""

from __future__ import annotations

from collections.abc import Callable
import ctypes.util
from datetime import datetime
import importlib
import inspect
from pathlib import Path
import platform
import shutil
import subprocess
import time
from typing import Any

import typer

from dimos.constants import STATE_DIR

app = typer.Typer(help="Galaxea A1Z robot commands")

_USB_VENDOR_ID = "a8fa"
_USB_PRODUCT_ID = "8598"
_DEFAULT_CAN_INTERFACE = "a1zcan"
_DEFAULT_CAN_BITRATE = 1_000_000
_CAN_PROBE_FRAME = "1FFFFFFF#"
_CAN_PROBE_ATTEMPTS = 20
_CAN_PROBE_POLL_SECONDS = 0.05
_SYS_USB_DEVICES = Path("/sys/bus/usb/devices")
_SYS_CLASS_NET = Path("/sys/class/net")
_GS_USB_NEW_ID = Path("/sys/bus/usb/drivers/gs_usb/new_id")
_A1Z_GUIDE = (
    "https://github.com/dimensionalOS/dimos/blob/main/docs/capabilities/manipulation/a1z.md"
)
_Check = tuple[str, Callable[[], str]]
_TEACH_HARDWARE_ID = "arm"
_GRIPPER_OPEN_M = 0.1
_GRIPPER_CLOSED_M = 0.0


def _abort(message: str) -> None:
    typer.echo(f"ERROR: {message}", err=True)
    raise typer.Exit(1)


def _verify_sdk() -> str:
    """Return the installed SDK path or raise with actionable instructions."""
    try:
        a1z = importlib.import_module("a1z")
        get_robot_module = importlib.import_module("a1z.robots.get_robot")
        get_a1z_robot = get_robot_module.get_a1z_robot
        parameters = inspect.signature(get_a1z_robot).parameters
    except Exception as exc:
        raise RuntimeError(
            f"A1Z SDK unavailable: {exc}. Install the pinned SDK listed in {_A1Z_GUIDE}."
        ) from exc
    if "with_gripper" not in parameters:
        raise RuntimeError(
            "installed A1Z SDK lacks get_a1z_robot(with_gripper=...). Install the pinned "
            f"gripper-capable SDK listed in {_A1Z_GUIDE}."
        )
    return str(a1z.__file__)


def _verify_adapter_import() -> str:
    try:
        adapter = importlib.import_module("dimos.hardware.manipulators.galaxea_a1z.adapter")
    except Exception as exc:
        raise RuntimeError(
            f"A1Z adapter unavailable: {exc}. Install DimOS manipulation dependencies as "
            f"described in {_A1Z_GUIDE}."
        ) from exc
    return str(adapter.__file__)


def _verify_linux_dependencies() -> str:
    cansend = shutil.which("cansend")
    if cansend is None:
        raise RuntimeError(
            "`cansend` is missing. Install your distribution's can-utils package "
            "(`sudo apt-get install can-utils` on Ubuntu)."
        )
    return cansend


def _macos_usb_modules() -> tuple[Any, Any, Any]:
    try:
        usb_backend = importlib.import_module("usb.backend.libusb1")
        usb_core = importlib.import_module("usb.core")
        importlib.import_module("gs_usb.gs_usb")
        gs_usb_module = importlib.import_module(
            "dimos.hardware.manipulators.galaxea_a1z.gs_usb_bus"
        )
    except Exception as exc:
        raise RuntimeError(
            f"macOS USB-CAN dependencies unavailable: {exc}. Install pyusb and gs-usb as "
            f"described in {_A1Z_GUIDE}."
        ) from exc
    return usb_backend, usb_core, gs_usb_module


def _verify_macos_dependencies() -> str:
    if ctypes.util.find_library("usb-1.0") is None:
        raise RuntimeError("libusb is missing. Install it with `brew install libusb`.")
    usb_backend, _, _ = _macos_usb_modules()
    if usb_backend.get_backend() is None:
        raise RuntimeError("PyUSB could not load libusb. Install it with `brew install libusb`.")
    return "pyusb, gs-usb, and libusb"


def _find_hhs_usb_device() -> Path | None:
    for device in sorted(_SYS_USB_DEVICES.glob("*")):
        try:
            vendor = (device / "idVendor").read_text().strip().lower()
            product = (device / "idProduct").read_text().strip().lower()
        except OSError:
            continue
        if vendor == _USB_VENDOR_ID and product == _USB_PRODUCT_ID:
            return device
    return None


def _find_can_interface(usb_device: Path) -> str | None:
    interfaces = sorted(usb_device.parent.glob(f"{usb_device.name}:*/net/*"))
    return interfaces[0].name if interfaces else None


def _run_privileged(command: list[str], **kwargs: Any) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["sudo", *command], check=True, text=True, **kwargs)


def _read_can_counter(interface: str, counter: str) -> int:
    return int((_SYS_CLASS_NET / interface / "statistics" / counter).read_text())


def _usb_bulk_out_endpoint(usb_device: Path) -> str:
    for endpoint in sorted(usb_device.parent.glob(f"{usb_device.name}:*/ep_*")):
        try:
            if (endpoint / "direction").read_text().strip() == "out" and (
                endpoint / "type"
            ).read_text().strip() == "Bulk":
                return f"0x{(endpoint / 'bEndpointAddress').read_text().strip()}"
        except OSError:
            continue
    return "unknown"


def _verify_can_transmit(interface: str, usb_device: Path) -> None:
    if shutil.which("cansend") is None:
        raise RuntimeError(
            "`cansend` is required to prove that the HHS adapter can transmit. "
            "Install your distribution's can-utils package."
        )

    tx_before = _read_can_counter(interface, "tx_packets")
    dropped_before = _read_can_counter(interface, "tx_dropped")
    result = subprocess.run(
        ["cansend", interface, _CAN_PROBE_FRAME],
        check=False,
        capture_output=True,
        text=True,
    )
    tx_after = tx_before
    dropped_after = dropped_before
    for _ in range(_CAN_PROBE_ATTEMPTS):
        tx_after = _read_can_counter(interface, "tx_packets")
        dropped_after = _read_can_counter(interface, "tx_dropped")
        if tx_after > tx_before or dropped_after > dropped_before:
            break
        time.sleep(_CAN_PROBE_POLL_SECONDS)

    diagnostics = (
        f"interface={interface}, kernel={platform.release()}, "
        f"USB OUT={_usb_bulk_out_endpoint(usb_device)}, "
        f"tx_packets={tx_before}->{tx_after}, "
        f"tx_dropped={dropped_before}->{dropped_after}, "
        f"cansend={result.stderr.strip() or result.stdout.strip() or result.returncode}"
    )
    if dropped_after > dropped_before:
        raise RuntimeError(
            "the Linux gs_usb driver rejected transmission through the HHS adapter "
            f"({diagnostics}). See the kernel and Jetson remediation guide in {_A1Z_GUIDE}."
        )
    if tx_after <= tx_before:
        raise RuntimeError(
            "the CAN interface did not complete a transmission "
            f"({diagnostics}). Check arm power, cabling, termination, and competing processes."
        )


def _configure_linux_can(interface: str, bitrate: int) -> None:
    _run_privileged(["modprobe", "gs_usb"])
    usb_device = _find_hhs_usb_device()
    if usb_device is None:
        raise RuntimeError(
            f"HHS USB-CANFD adapter {_USB_VENDOR_ID}:{_USB_PRODUCT_ID} was not found"
        )

    current_interface = _find_can_interface(usb_device)
    if current_interface is None:
        _run_privileged(
            ["tee", str(_GS_USB_NEW_ID)],
            input=f"{_USB_VENDOR_ID} {_USB_PRODUCT_ID}\n",
            capture_output=True,
        )
        subprocess.run(["udevadm", "settle", "--timeout=3"], check=True)
        for _ in range(30):
            current_interface = _find_can_interface(usb_device)
            if current_interface:
                break
            time.sleep(0.1)
    if current_interface is None:
        raise RuntimeError("the gs_usb driver did not create a CAN interface")

    _run_privileged(["ip", "link", "set", current_interface, "down"])
    if current_interface != interface:
        if (_SYS_CLASS_NET / interface).exists():
            raise RuntimeError(f"target CAN interface {interface!r} already exists")
        _run_privileged(["ip", "link", "set", current_interface, "name", interface])
    _run_privileged(["ip", "link", "set", interface, "type", "can", "bitrate", str(bitrate)])
    _run_privileged(["ip", "link", "set", interface, "up"])
    _verify_can_transmit(interface, usb_device)


def _verify_linux_can(interface: str) -> str:
    usb_device = _find_hhs_usb_device()
    if usb_device is None:
        raise RuntimeError(
            f"HHS USB-CANFD adapter {_USB_VENDOR_ID}:{_USB_PRODUCT_ID} was not found."
        )
    detected_interface = _find_can_interface(usb_device)
    if detected_interface != interface:
        detected = detected_interface or "no SocketCAN interface"
        raise RuntimeError(
            f"expected interface {interface!r}, found {detected!r}. Run "
            "`dimos hardware a1z configure-can`."
        )

    interface_path = _SYS_CLASS_NET / interface
    try:
        flags = int((interface_path / "flags").read_text(), 16)
        driver = (interface_path / "device" / "driver").resolve(strict=True).name
    except OSError as exc:
        raise RuntimeError(f"cannot inspect SocketCAN interface {interface!r}: {exc}") from exc
    if driver != "gs_usb":
        raise RuntimeError(f"interface {interface!r} uses driver {driver!r}, not 'gs_usb'.")
    if not flags & 0x1:
        raise RuntimeError(
            f"interface {interface!r} is DOWN. Run `dimos hardware a1z configure-can`."
        )
    return f"{interface} is UP on gs_usb"


def _verify_macos_can() -> str:
    usb_backend, usb_core, gs_usb_module = _macos_usb_modules()
    backend = usb_backend.get_backend()
    if backend is None:
        raise RuntimeError("PyUSB could not load libusb. Install it with `brew install libusb`.")
    device = usb_core.find(
        idVendor=int(_USB_VENDOR_ID, 16),
        idProduct=int(_USB_PRODUCT_ID, 16),
        backend=backend,
    )
    if device is None:
        raise RuntimeError(
            f"HHS USB-CANFD adapter {_USB_VENDOR_ID}:{_USB_PRODUCT_ID} was not found."
        )
    bus = gs_usb_module.GsUsbMacBus(listen_only=True)
    try:
        return "HHS adapter opened in listen-only mode"
    finally:
        bus.shutdown()


def _doctor_checks(system: str, software_only: bool, interface: str) -> list[_Check]:
    checks: list[_Check] = [
        ("A1Z SDK", _verify_sdk),
        ("DimOS A1Z adapter", _verify_adapter_import),
    ]
    if system == "Linux":
        checks.append(("can-utils", _verify_linux_dependencies))
        if not software_only:
            checks.append(("Linux USB-CAN", lambda: _verify_linux_can(interface)))
    elif system == "Darwin":
        checks.append(("macOS USB-CAN dependencies", _verify_macos_dependencies))
        if not software_only:
            checks.append(("macOS USB-CAN", _verify_macos_can))
    return checks


def _run_doctor_checks(checks: list[_Check]) -> int:
    failures = 0
    for name, check in checks:
        try:
            detail = check()
        except (OSError, RuntimeError, ValueError) as exc:
            failures += 1
            typer.echo(f"FAIL  {name}: {exc}", err=True)
        else:
            typer.echo(f"PASS  {name}: {detail}")
    return failures


@app.command()
def doctor(
    software_only: bool = typer.Option(
        False,
        "--software-only",
        help="Check dependencies without requiring attached hardware",
    ),
    interface: str = typer.Option(
        _DEFAULT_CAN_INTERFACE,
        "--interface",
        help="Expected Linux SocketCAN interface",
    ),
) -> None:
    """Check A1Z dependencies and hardware without changing the host."""
    system = platform.system()
    if system not in {"Linux", "Darwin"}:
        _abort("A1Z diagnostics support Linux and macOS only")

    failures = _run_doctor_checks(_doctor_checks(system, software_only, interface))
    if failures:
        typer.echo(
            f"A1Z doctor found {failures} problem(s). See {_A1Z_GUIDE}.",
            err=True,
        )
        raise typer.Exit(1)
    typer.echo("A1Z doctor passed.")


@app.command("configure-can")
def configure_can(
    interface: str = typer.Option(
        _DEFAULT_CAN_INTERFACE,
        "--interface",
        help="Stable SocketCAN interface name",
    ),
    bitrate: int = typer.Option(
        _DEFAULT_CAN_BITRATE,
        "--bitrate",
        help="CAN bitrate",
    ),
    yes: bool = typer.Option(
        False,
        "--yes",
        "-y",
        help="Skip the confirmation prompt",
    ),
) -> None:
    """Configure and transmission-test the Linux HHS USB-CANFD adapter."""
    if platform.system() != "Linux":
        _abort("`dimos hardware a1z configure-can` is Linux-only; macOS uses userspace USB-CAN")
    if not yes and not typer.confirm(
        "This will request sudo to configure the A1Z CAN interface. Continue?",
        default=False,
    ):
        typer.echo("Aborted.")
        raise typer.Exit(1)
    try:
        _configure_linux_can(interface, bitrate)
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        _abort(str(exc))
    typer.echo(f"A1Z CAN configuration passed: {interface!r} transmitted at {bitrate} bit/s.")


def _default_recording_path() -> Path:
    return STATE_DIR / "recordings" / f"a1z_teach_{datetime.now():%Y%m%d_%H%M%S}.db"


def _press_enter(message: str) -> None:
    typer.prompt(message, default="", show_default=False)


def _read_key(message: str) -> str:
    """Read one keypress, falling back to line input for non-interactive stdin."""
    import sys

    typer.echo(message)
    if not sys.stdin.isatty():
        line = sys.stdin.readline()
        if not line:
            raise EOFError
        return line.strip().lower()[:1]

    import termios
    import tty

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
    if key == "\x03":
        raise KeyboardInterrupt
    if key in ("\r", "\n"):
        return ""
    return key.lower()


@app.command()
def teach(
    output: Path | None = typer.Argument(
        None,
        help="Memory2 .db output (default: timestamped file in the dimOS state directory)",
    ),
    task: str | None = typer.Option(None, "--task", help="Task label stored with each episode"),
    camera_index: int = typer.Option(
        0,
        "--camera-index",
        min=0,
        help="Linux camera index N for /dev/videoN",
    ),
    gripper_free_drive: bool = typer.Option(
        False,
        "--gripper-free-drive",
        help="Make the gripper hand-drivable instead of controlling it with the g key",
    ),
) -> None:
    """Hand-teach episodes into one Memory2 recording."""
    from dimos.control.coordinator import ControlCoordinator
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
    from dimos.robot.manipulators.a1z.blueprints.learning import make_a1z_teach_blueprint

    db_path = (output or _default_recording_path()).expanduser().resolve()
    if db_path.exists():
        typer.echo(f"error: refusing to overwrite existing recording: {db_path}", err=True)
        raise typer.Exit(2)

    typer.echo("A1Z hand-teach mode")
    typer.echo(f"Recording: {db_path}")
    typer.echo(f"Camera: /dev/video{camera_index} (640x480 at 15 FPS)")
    typer.echo("The arm will become hand-drivable after startup.")
    if gripper_free_drive:
        typer.echo("Gripper: free drive (open and close it by hand).")
    else:
        typer.echo("Gripper: powered; press g to toggle open/closed.")
    typer.echo("Keep the arm supported: it has no brakes and can fall when motors disable.\n")

    coordinator: ModuleCoordinator | None = None
    recording = False
    gripper_open: bool | None = None
    saved_count = 0
    episode_started_at = 0.0

    def status_line() -> str:
        if gripper_free_drive:
            gripper = "free-drive"
        elif gripper_open is None:
            gripper = "?"
        else:
            gripper = "open" if gripper_open else "closed"
        if recording:
            elapsed = time.monotonic() - episode_started_at
            state = f"RECORDING {int(elapsed // 60)}:{int(elapsed % 60):02d}"
            keys = "SPACE save · g gripper · d discard · q quit"
        else:
            state = "IDLE"
            keys = "SPACE record · d undo last · g gripper · q quit"
        return f"[{state} | saved: {saved_count} | gripper: {gripper}]  {keys}"

    try:
        coordinator = ModuleCoordinator.build(
            make_a1z_teach_blueprint(
                db_path,
                task_label=task,
                camera_index=camera_index,
                gripper_free_drive=gripper_free_drive,
            )
        )
        monitor: Any = coordinator.get_instance(EpisodeMonitorModule)
        control: Any = coordinator.get_instance(ControlCoordinator)
        if not gripper_free_drive:
            measured = control.get_gripper_position(_TEACH_HARDWARE_ID)
            gripper_open = measured is not None and measured > _GRIPPER_OPEN_M / 2
        typer.echo("Ready. Move only after starting an episode.")

        def toggle_gripper() -> None:
            nonlocal gripper_open
            if gripper_free_drive:
                typer.echo("Gripper is in free drive; open and close it by hand.")
                return
            target_open = not gripper_open
            target = _GRIPPER_OPEN_M if target_open else _GRIPPER_CLOSED_M
            if control.set_gripper_position(_TEACH_HARDWARE_ID, target):
                gripper_open = target_open
                typer.echo(f">> gripper {'opening' if target_open else 'closing'}")
            else:
                typer.echo(">> gripper command rejected; check hardware state", err=True)

        while True:
            command = _read_key(status_line())
            if command == "g":
                toggle_gripper()
                continue
            if command in (" ", ""):
                if not recording:
                    monitor.start_episode()
                    recording = True
                    episode_started_at = time.monotonic()
                    typer.echo(">> episode started - move the arm by hand")
                else:
                    episode_status = monitor.save_episode()
                    recording = False
                    saved_count = episode_status.episodes_saved
                    typer.echo(f">> episode saved ({saved_count} total)")
                continue
            if command == "d":
                saved_before = saved_count
                episode_status = monitor.discard_episode()
                if recording:
                    recording = False
                    typer.echo(">> episode discarded")
                elif episode_status.episodes_saved < saved_before:
                    saved_count = episode_status.episodes_saved
                    typer.echo(f">> previous saved episode discarded ({saved_count} remain)")
                else:
                    typer.echo(">> nothing saved to discard")
                continue
            if command == "q":
                if not recording:
                    break
                choice = _read_key(
                    "Episode in progress - s to save, d to discard, or another key to continue"
                )
                if choice == "s":
                    episode_status = monitor.save_episode()
                    recording = False
                    saved_count = episode_status.episodes_saved
                    typer.echo(f">> episode saved ({saved_count} total)")
                    break
                if choice == "d":
                    monitor.discard_episode()
                    recording = False
                    typer.echo(">> episode discarded")
                    break
                typer.echo(">> still recording")
                continue
            typer.echo(f">> unrecognized key {command!r}")
    except KeyboardInterrupt:
        if coordinator is not None and recording:
            monitor = coordinator.get_instance(EpisodeMonitorModule)
            monitor.discard_episode()
            typer.echo("\nActive episode discarded.")
    except Exception as exc:
        typer.echo(f"A1Z teach failed: {exc}", err=True)
        raise typer.Exit(1) from exc
    finally:
        if coordinator is not None:
            typer.echo("\nSupport the arm before the recording is flushed and motors disable.")
            try:
                _press_enter("Press ENTER when the arm is supported")
            except (KeyboardInterrupt, EOFError):
                pass
            coordinator.stop()

    typer.echo(f"Saved Memory2 recording: {db_path}")


@app.command()
def replay(
    source: Path = typer.Argument(..., help="Memory2 recording .db"),
    episode: int = typer.Option(-1, "--episode", "-e", help="Saved episode index; -1 is latest"),
    speed: float = typer.Option(1.0, "--speed", min=0.01, help="Requested playback speed"),
) -> None:
    """Validate and replay one saved A1Z episode through ControlCoordinator."""
    from dimos.control.coordinator import ControlCoordinator
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
    from dimos.robot.manipulators.a1z.blueprints.learning import (
        A1Z_REPLAY_TASK_NAME,
        make_a1z_replay_blueprint,
    )
    from dimos.robot.manipulators.a1z.teach_replay import (
        build_execution_trajectory,
        load_recorded_episode,
        prepare_episode,
    )

    source = source.expanduser().resolve()
    try:
        recorded = load_recorded_episode(source, episode)
        prepared = prepare_episode(recorded, speed=speed)
    except (IndexError, OSError, RuntimeError, ValueError) as exc:
        typer.echo(f"A1Z replay preflight failed: {exc}", err=True)
        raise typer.Exit(1) from exc

    typer.echo(f"Recording: {source}")
    typer.echo(
        f"Episode: {recorded.episode_index} ({len(recorded.timestamps)} measured samples, "
        f"{recorded.timestamps[-1]:.2f}s)"
    )
    if prepared.effective_speed < prepared.requested_speed * 0.999:
        typer.echo(
            f"Safety time-scaling: requested {prepared.requested_speed:.2f}x, "
            f"using {prepared.effective_speed:.2f}x"
        )
    else:
        typer.echo(f"Playback speed: {prepared.effective_speed:.2f}x")
    typer.echo("Raw recorded values passed command-limit validation; nothing was clipped.")
    typer.echo("Support the arm during startup. It has no brakes.\n")

    coordinator: ModuleCoordinator | None = None
    started = False
    try:
        coordinator = ModuleCoordinator.build(make_a1z_replay_blueprint())
        control: Any = coordinator.get_instance(ControlCoordinator)
        trajectory = build_execution_trajectory(control.get_joint_positions(), prepared)
        typer.echo(
            "The robot will approach the recorded start pose, then replay for "
            f"{prepared.duration:.2f}s. Total controlled motion: {trajectory.duration:.2f}s."
        )
        if not typer.confirm("Execute this motion now?", default=False):
            typer.echo("Replay cancelled before motion.")
            return

        accepted = control.task_invoke(
            A1Z_REPLAY_TASK_NAME,
            "execute",
            {"trajectory": trajectory},
        )
        if not accepted:
            raise RuntimeError("ControlCoordinator rejected the replay trajectory")
        started = True

        deadline = time.monotonic() + trajectory.duration + 5.0
        while time.monotonic() < deadline:
            state = TrajectoryState(control.task_invoke(A1Z_REPLAY_TASK_NAME, "get_state", {}))
            if state == TrajectoryState.COMPLETED:
                typer.echo("Replay complete. The arm is holding the final pose.")
                break
            if state in (TrajectoryState.ABORTED, TrajectoryState.FAULT):
                raise RuntimeError(f"Replay ended in state {state.name}")
            time.sleep(0.05)
        else:
            control.task_invoke(A1Z_REPLAY_TASK_NAME, "cancel", {})
            raise TimeoutError("Replay did not complete before its safety timeout")
    except KeyboardInterrupt:
        typer.echo("\nReplay interrupted.", err=True)
        if coordinator is not None and started:
            control = coordinator.get_instance(ControlCoordinator)
            control.task_invoke(A1Z_REPLAY_TASK_NAME, "cancel", {})
    except (OSError, RuntimeError, TimeoutError, ValueError) as exc:
        typer.echo(f"A1Z replay failed: {exc}", err=True)
        raise typer.Exit(1) from exc
    finally:
        if coordinator is not None:
            typer.echo("Support the arm before disabling its motors.")
            try:
                _press_enter("Press ENTER when the arm is supported")
            except (KeyboardInterrupt, EOFError):
                pass
            coordinator.stop()


@app.command("run-policy")
def run_policy(
    checkpoint: str = typer.Argument(
        ...,
        help="Local LeRobot pretrained_model directory or Hugging Face model ID",
    ),
    task: str = typer.Option("", "--task", help="Task prompt supplied to the policy"),
    duration: float = typer.Option(
        10.0,
        "--duration",
        min=0.1,
        help="Maximum policy execution time in seconds",
    ),
    camera_index: int = typer.Option(
        0,
        "--camera-index",
        min=0,
        help="Linux camera index N for /dev/videoN",
    ),
    device: str | None = typer.Option(
        None,
        "--device",
        help="Torch device override, for example cuda or cpu",
    ),
) -> None:
    """Execute a trained LeRobot policy on the live A1Z."""
    # Load the isolated runtime contract only when this command is used.
    try:
        from dimos.core.coordination.module_coordinator import ModuleCoordinator
        from dimos.imitation.policy.lerobot.module import LeRobotPolicyModule
        from dimos.robot.manipulators.a1z.blueprints.learning import make_a1z_policy_blueprint
    except ImportError as exc:
        typer.echo(f"A1Z policy execution is unavailable: {exc}", err=True)
        raise typer.Exit(1) from exc

    local_checkpoint = Path(checkpoint).expanduser()
    policy_path = str(local_checkpoint.resolve()) if local_checkpoint.exists() else checkpoint

    typer.echo("A1Z learned-policy execution")
    typer.echo(f"Checkpoint: {policy_path}")
    typer.echo(f"Camera: /dev/video{camera_index} (640x480 at 15 FPS)")
    typer.echo(f"Maximum execution: {duration:.1f}s")
    typer.echo("The arm has no brakes. Support it during startup and clear the workspace.\n")
    if not typer.confirm("Load the policy and initialize the robot?", default=False):
        typer.echo("Policy execution cancelled.")
        return

    coordinator: ModuleCoordinator | None = None
    policy: Any = None
    try:
        coordinator = ModuleCoordinator.build(
            make_a1z_policy_blueprint(
                policy_path,
                task=task,
                camera_index=camera_index,
                device=device,
            )
        )
        policy = coordinator.get_instance(LeRobotPolicyModule)
        observation_deadline = time.monotonic() + 5.0
        while time.monotonic() < observation_deadline:
            policy_status = policy.rollout_status()
            if policy_status["observations_ready"]:
                break
            time.sleep(0.1)
        else:
            raise RuntimeError(
                f"live policy observations did not become ready: {policy_status['last_error']}"
            )
        policy_status = policy.start_rollout(duration)
        if not policy_status["active"]:
            raise RuntimeError(policy_status["last_error"] or "policy rollout did not start")
        typer.echo("Policy rollout started.")

        deadline = time.monotonic() + duration + 5.0
        while time.monotonic() < deadline:
            policy_status = policy.rollout_status()
            if not policy_status["active"]:
                if policy_status["last_error"]:
                    raise RuntimeError(policy_status["last_error"])
                typer.echo(
                    "Policy execution complete "
                    f"({policy_status['commands_published']} commands sent)."
                )
                break
            time.sleep(0.1)
        else:
            policy.stop_rollout()
            raise TimeoutError("Policy did not stop before its execution timeout")
    except KeyboardInterrupt:
        typer.echo("\nPolicy execution interrupted.", err=True)
        if policy is not None:
            policy.stop_rollout()
    except (ImportError, OSError, RuntimeError, TimeoutError, ValueError) as exc:
        typer.echo(f"A1Z policy execution failed: {exc}", err=True)
        raise typer.Exit(1) from exc
    finally:
        if coordinator is not None:
            typer.echo("Support the arm before disabling its motors.")
            try:
                _press_enter("Press ENTER when the arm is supported")
            except (KeyboardInterrupt, EOFError):
                pass
            coordinator.stop()
