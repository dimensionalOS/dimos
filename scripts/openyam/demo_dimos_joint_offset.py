"""Compare settled OpenYAM joint offsets with DimOS tracking correction off/on.

Run with the modified /home/dimos/dimos checkout's Python environment. This
script commands real arm motors only when --execute is supplied. It never
commands grippers, changes motor zero, or writes motor firmware.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path
import signal
import socket
import struct
import threading
import time
import traceback
from typing import Any

import can_motor_control
from can_motor_control import damiao
import numpy as np

from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.dual_openyam_damiao.adapter import (
    DualOpenYamDamiaoAdapter,
    _arm_motors,
)
from dimos.hardware.whole_body.spec import MotorCommand

HERE = Path(__file__).absolute().parent
SESSION = HERE.parent
CHECKER = HERE / "bounded_integral_stability" / "check_workspace.py"
KP = np.array([80.0, 80.0, 80.0, 10.0, 10.0, 10.0] * 2)
KD = np.array([5.0, 5.0, 5.0, 1.5, 1.5, 1.5] * 2)
SIDES = ("left", "right")
# MAX_DELTA_DEG = 5.0
# MAX_TRACKING_ERROR_RAD = 0.15
# TEMPERATURE_MARGIN_C = 53
# TEMPERATURE_HARD_C = 60
PERIOD_S = 0.01
stop_requested = False


class ArmOnlyDimOSAdapter(DualOpenYamDamiaoAdapter):
    """Use DimOS's normal arm command path without enabling grippers."""

    gripper_joints: dict[str, str] = {}

    def _build_robot(self) -> can_motor_control.Robot:
        return (
            can_motor_control.Robot.builder()
            .add_bus("left", self._make_can_bus("left"), damiao.DamiaoCodec())
            .add_bus("right", self._make_can_bus("right"), damiao.DamiaoCodec())
            .add_arm("left_arm", bus="left", motors=_arm_motors("left"))
            .add_arm("right_arm", bus="right", motors=_arm_motors("right"))
            .build()
        )

class RawMonitor:
    """Independent SocketCAN encoder/status/temperature observation."""

    def __init__(self, ports: dict[str, str], output: Path) -> None:
        self.sockets: dict[str, socket.socket] = {}
        self.latest: dict[str, dict[int, dict[str, Any]]] = {side: {} for side in SIDES}
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.error: str | None = None
        self.log = output.open("w", buffering=65536)
        for side, port in ports.items():
            sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.bind((port,))
            sock.settimeout(0.1)
            self.sockets[side] = sock
        self.threads = [
            threading.Thread(target=self._run, args=(side, sock), daemon=True)
            for side, sock in self.sockets.items()
        ]
        for thread in self.threads:
            thread.start()

    def _run(self, side: str, sock: socket.socket) -> None:
        try:
            while not self.stop.is_set():
                try:
                    packet = sock.recv(16)
                except TimeoutError:
                    continue
                mono = time.monotonic()
                can_id, length, payload = struct.unpack("=IB3x8s", packet)
                payload = payload[:length]
                item = self._decode(can_id, payload)
                with self.lock:
                    self.log.write(
                        json.dumps(
                            {
                                "time": time.time(),
                                "mono": mono,
                                "side": side,
                                "can_id": can_id,
                                "payload": payload.hex(),
                                "state": item,
                            }
                        )
                        + "\n"
                    )
                    if item is not None:
                        item["mono"] = mono
                        self.latest[side][item["joint"]] = item
        except Exception as exc:
            self.error = repr(exc)

    @staticmethod
    def _decode(can_id: int, payload: bytes) -> dict[str, Any] | None:
        if len(payload) != 8 or can_id not in range(17, 23):
            return None
        joint = can_id - 16
        if payload[0] & 0x0F != joint:
            return None
        # Parameter replies use motor ID in little-endian bytes 0:2. A normal
        # position byte may be 0x33 or 0x55, so byte 2 alone is not enough.
        param_id = int.from_bytes(payload[:2], "little")
        if param_id <= 0x7FF and payload[2] in (0x33, 0x55):
            return None
        position = ((payload[1] << 8) | payload[2]) * 25.0 / 65535.0 - 12.5
        return {
            "joint": joint,
            "status": payload[0] >> 4,
            "q": position,
            "temperature_mos": payload[6],
            "temperature_rotor": payload[7],
        }

    def snapshot(self) -> dict[str, dict[int, dict[str, Any]]]:
        with self.lock:
            return {
                side: {joint: row.copy() for joint, row in rows.items()}
                for side, rows in self.latest.items()
            }

    def close(self) -> None:
        self.stop.set()
        for thread in self.threads:
            thread.join(timeout=1.0)
        for sock in self.sockets.values():
            sock.close()
        self.log.close()


def read_register(port: str, motor: int, register: int, *, integer: bool) -> float:
    """Read a motor parameter without enabling or changing it."""
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((port,))
        sock.settimeout(0.02)
        query = bytes([motor, 0, 0x33, register, 0, 0, 0, 0])
        for _ in range(3):
            sock.send(struct.pack("=IB3x8s", 0x7FF, 8, query))
            deadline = time.monotonic() + 0.12
            while time.monotonic() < deadline:
                try:
                    packet = sock.recv(16)
                except TimeoutError:
                    continue
                _, length, payload = struct.unpack("=IB3x8s", packet)
                if length == 8 and payload[:4] == query[:4]:
                    return float(struct.unpack("<I" if integer else "<f", payload[4:8])[0])
        raise RuntimeError(f"No parameter response for {port} motor {motor} register {register}")
    finally:
        sock.close()


def load_workspace_checker() -> Any:
    spec = importlib.util.spec_from_file_location("offset_workspace_check", CHECKER)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load workspace checker: {CHECKER}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_path(checker: Any, start: np.ndarray, target: np.ndarray) -> None:
    for fraction in np.linspace(0.0, 1.0, 31):
        q = start * (1.0 - fraction) + target * fraction
        result = checker.check({"left": q[:6], "right": q[6:]})
        if not result[0]:
            raise ValueError(f"Workspace check rejected path at {fraction:.2f}: {result[1]}")


def raw_vectors(monitor: RawMonitor, *, enabled: bool | None = None) -> tuple[np.ndarray, int]:
    if monitor.error is not None:
        raise RuntimeError(f"Raw CAN monitor failed: {monitor.error}")
    snapshot = monitor.snapshot()
    # now = time.monotonic()
    q = np.empty(12)
    highest_temperature = 0
    for side_index, side in enumerate(SIDES):
        for joint in range(1, 7):
            row = snapshot[side].get(joint)
            if row is None:
                raise RuntimeError(f"Missing raw feedback: {side} J{joint}")
            # if now - row["mono"] > 0.15:
            #     raise RuntimeError(f"Stale raw feedback: {side} J{joint}")
            if enabled is not None and row["status"] != int(enabled):
                raise RuntimeError(f"Unexpected motor status: {side} J{joint}={row['status']}")
            q[side_index * 6 + joint - 1] = row["q"]
            highest_temperature = max(
                highest_temperature, row["temperature_mos"], row["temperature_rotor"]
            )
    return q, highest_temperature


def wait_disabled(monitor: RawMonitor, timeout: float = 2.0) -> np.ndarray:
    deadline = time.monotonic() + timeout
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            return raw_vectors(monitor, enabled=False)[0]
        except RuntimeError as exc:
            last_error = exc
            time.sleep(0.02)
    raise RuntimeError(f"Could not confirm all twelve arm motors disabled: {last_error}")


def settle_seconds(joint: int, override: float | None) -> float:
    if override is not None:
        return override
    return 40.0 if joint == 3 else 20.0 if joint == 4 else 15.0


class Runner:
    def __init__(
        self, args: argparse.Namespace, checker: Any, monitor: RawMonitor, samples: Any
    ) -> None:
        self.args = args
        self.checker = checker
        self.monitor = monitor
        self.samples = samples
        self.adapter: ArmOnlyDimOSAdapter | None = None
        self.reference: np.ndarray | None = None
        self.last_command: np.ndarray | None = None
        self.next_tick = time.monotonic()
        self.recovering = False
        self.last_temp = 0
        self.activated_at = 0.0
        self.last_sample_at: float | None = None

    def preflight(self) -> np.ndarray:
        probe = ArmOnlyDimOSAdapter(
            runtime_config=DamiaoRuntimeConfig(
                bus_devices={"left": self.args.left_can, "right": self.args.right_can},
                gravity_comp=True,
                tracking_comp=False,
            )
        )
        try:
            if not probe.connect():
                raise RuntimeError("Read-only DimOS preflight connection failed")
            for _ in range(10):
                probe.read_motor_states()
                time.sleep(0.01)
            raw_q, temperature = raw_vectors(self.monitor, enabled=False)
        finally:
            probe.disconnect()
        # if temperature >= TEMPERATURE_MARGIN_C:
        #     raise RuntimeError(f"Motor is too warm to start ({temperature} C)")
        check_path(self.checker, raw_q, raw_q)
        for side, port in (("left", self.args.left_can), ("right", self.args.right_can)):
            for joint in range(1, 7):
                pmax = read_register(port, joint, 21, integer=False)
                vmax = read_register(port, joint, 22, integer=False)
                tmax = read_register(port, joint, 23, integer=False)
                mode = read_register(port, joint, 10, integer=True)
                expected = (12.5, 10.0, 28.0) if joint <= 3 else (12.5, 30.0, 10.0)
                if not np.allclose([pmax, vmax, tmax], expected, atol=0.02) or mode != 1:
                    raise RuntimeError(f"Unexpected protocol limits or MIT mode: {side} J{joint}")
        return raw_q

    def connect(self, mode: str, expected_q: np.ndarray) -> None:
        self.adapter = ArmOnlyDimOSAdapter(
            runtime_config=DamiaoRuntimeConfig(
                bus_devices={"left": self.args.left_can, "right": self.args.right_can},
                gravity_comp=True,
                tracking_comp=(mode == "candidate"),
                openyam_anvil_parity=(mode == "candidate"),
            )
        )
        if not self.adapter.connect():
            raise RuntimeError("DimOS adapter failed to connect")
        for _ in range(10):
            states = self.adapter.read_motor_states()
            time.sleep(0.01)
        # library_q = np.array([state.q for state in states])
        raw_q, _ = raw_vectors(self.monitor, enabled=False)
        # if np.max(np.abs(raw_q - expected_q)) > 0.04:
        #     raise RuntimeError("Starting pose changed since path preflight")
        # if np.max(np.abs(raw_q - library_q)) > 0.003:
        #     raise RuntimeError("DimOS and raw encoder positions disagree")
        self.reference = raw_q.copy()
        self.last_command = raw_q.copy()
        if not self.adapter.activate():
            raise RuntimeError("DimOS adapter failed to activate")
        # Activation sends mode, enable, and refresh frames in quick succession.
        # Let the SocketCAN transmit queue drain before the first trial refresh.
        time.sleep(0.01)
        self.activated_at = time.monotonic()
        self.last_sample_at = None
        self.next_tick = time.monotonic()

    def sample(self, target: np.ndarray, phase: str) -> dict[str, Any]:
        if self.adapter is None or self.reference is None:
            raise RuntimeError("Adapter is inactive")
        now = time.monotonic()
        states = self.adapter.read_motor_states()
        raw_q, temp = raw_vectors(self.monitor)
        if now - self.activated_at >= 0.25:
            snapshot = self.monitor.snapshot()
            for side in SIDES:
                for joint in range(1, 7):
                    if snapshot[side][joint]["status"] != 1:
                        raise RuntimeError(f"Arm motor is not enabled and healthy: {side} J{joint}")
        self.last_temp = temp
        # if temp >= TEMPERATURE_HARD_C:
        #     raise RuntimeError(f"Motor hard temperature guard reached: {temp} C")
        # if temp >= TEMPERATURE_MARGIN_C and not self.recovering:
        #     raise RuntimeError(f"Motor cooling margin reached: {temp} C")
        # if np.max(np.abs(raw_q - target)) > MAX_TRACKING_ERROR_RAD:
        #     raise RuntimeError("Position tracking error exceeded 0.15 rad")
        if not self.recovering and stop_requested:
            raise InterruptedError("Stop requested")
        # if self.last_command is not None and np.max(np.abs(target - self.last_command)) > 0.01:
        #     raise RuntimeError("Command jump exceeded 0.01 rad per cycle")
        command = [
            MotorCommand(q=float(target[i]), dq=0.0, kp=float(KP[i]), kd=float(KD[i]), tau=0.0)
            for i in range(12)
        ]
        if not self.adapter.write_motor_commands(command):
            raise RuntimeError("DimOS rejected motor command")
        self.last_command = target.copy()
        row = {
            "time": time.time(),
            "mono": now,
            "phase": phase,
            "loop_period_ms": (
                None if self.last_sample_at is None else (now - self.last_sample_at) * 1000.0
            ),
            "q_command": target.tolist(),
            "q_raw": raw_q.tolist(),
            "q_dimos": [float(state.q) for state in states],
            "dq_dimos": [float(state.dq) for state in states],
            "tau_dimos": [float(state.tau) for state in states],
            "temperature_max_c": temp,
            "tracking_bias": {
                name: policy.bias.tolist()
                for name, policy in getattr(self.adapter, "_holding_corrections", {}).items()
            },
        }
        self.last_sample_at = now
        self.samples.write(json.dumps(row) + "\n")
        self.next_tick += PERIOD_S
        time.sleep(max(0.0, self.next_tick - time.monotonic()))
        if self.next_tick < time.monotonic() - 0.03:
            self.next_tick = time.monotonic()
        return row

    def hold(self, target: np.ndarray, seconds: float, phase: str) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rows.append(self.sample(target, phase))
        return rows

    def move(self, target: np.ndarray, phase: str, *, speed: float = 0.04) -> None:
        if self.last_command is None:
            raise RuntimeError("No starting command")
        start = self.last_command.copy()
        check_path(self.checker, start, target)
        distance = float(np.max(np.abs(target - start)))
        duration = max(2.0, 1.875 * distance / speed)
        begun = time.monotonic()
        while True:
            u = min(1.0, (time.monotonic() - begun) / duration)
            blend = 10 * u**3 - 15 * u**4 + 6 * u**5
            self.sample(start + blend * (target - start), phase)
            if u >= 1.0:
                break

    def trial(self, side: str, joint: int, direction: int, mode: str) -> dict[str, Any]:
        if self.reference is None:
            raise RuntimeError("No reference pose")
        center = self.reference.copy()
        self.hold(center, 2.0, f"{mode}:{side}:J{joint}:center")
        start, _ = raw_vectors(self.monitor, enabled=True)
        index = (0 if side == "left" else 6) + joint - 1
        target = center.copy()
        target[index] = start[index] + direction * math.radians(self.args.delta_deg)
        check_path(self.checker, center, target)
        check_path(self.checker, start, target)
        label = f"{mode}:{side}:J{joint}:{'plus' if direction > 0 else 'minus'}"
        self.move(target, label + ":move")
        rows = self.hold(target, settle_seconds(joint, self.args.hold_seconds), label + ":settle")
        final_rows = [row for row in rows if row["mono"] >= rows[-1]["mono"] - 1.0]
        final_angle = float(np.mean([row["q_raw"][index] for row in final_rows]))
        arm_name = f"{side}_arm"
        max_bias = max(
            abs(row["tracking_bias"].get(arm_name, [0.0] * 6)[joint - 1]) for row in rows
        )
        summary = {
            "mode": mode,
            "side": side,
            "joint": joint,
            "direction": direction,
            "requested_delta_deg": direction * self.args.delta_deg,
            "measured_delta_deg": math.degrees(final_angle - start[index]),
            "offset_deg": math.degrees(target[index] - final_angle),
            "start_rad": float(start[index]),
            "target_rad": float(target[index]),
            "final_mean_rad": final_angle,
            "final_std_deg": math.degrees(
                float(np.std([row["q_raw"][index] for row in final_rows]))
            ),
            "temperature_max_c": max(row["temperature_max_c"] for row in rows),
            "max_abs_tracking_bias": max_bias,
        }
        self.move(center, label + ":return", speed=0.025)
        self.hold(center, 3.0, label + ":return_settle")
        return summary

    def close(self) -> None:
        if self.adapter is None:
            return
        try:
            self.adapter.deactivate()
        finally:
            self.adapter.disconnect()
            self.adapter = None


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--delta-deg",
        type=float,
        required=True,
        help="Absolute requested change per joint in degrees",
    )
    parser.add_argument("--mode", choices=("baseline", "candidate", "both"), default="both")
    parser.add_argument("--side", choices=SIDES, help="Test only this arm")
    parser.add_argument("--joint", type=int, choices=range(1, 7), help="Test only this joint")
    parser.add_argument("--left-can", default="can1")
    parser.add_argument("--right-can", default="can0")
    parser.add_argument(
        "--hold-seconds", type=float, help="Override default 15 s; J3 40 s; J4 20 s"
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--execute", action="store_true", help="Enable real arm motors")
    args = parser.parse_args()
    # if not math.isfinite(args.delta_deg) or not 0 < args.delta_deg <= MAX_DELTA_DEG:
    #     parser.error("--delta-deg must be in (0, 5]")
    if args.hold_seconds is not None and (
        not math.isfinite(args.hold_seconds) or not 2 <= args.hold_seconds <= 60
    ):
        parser.error("--hold-seconds must be in [2, 60]")
    if args.left_can == args.right_can:
        parser.error("left and right CAN ports must differ")
    return args


def main() -> int:
    global stop_requested
    args = arguments()
    if not args.execute:
        print("No motors were enabled. Add --execute to run the checked hardware test.")
        return 0
    args.output_dir.mkdir(parents=True, exist_ok=False)

    def request_stop(_signum: int, _frame: Any) -> None:
        global stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    checker = load_workspace_checker()
    ports = {"left": args.left_can, "right": args.right_can}
    monitor = RawMonitor(ports, args.output_dir / "raw_can.jsonl")
    samples = (args.output_dir / "samples.jsonl").open("w", buffering=65536)
    runner = Runner(args, checker, monitor, samples)
    results: list[dict[str, Any]] = []
    error: str | None = None
    final_disabled: bool | None = None
    try:
        time.sleep(0.2)
        initial = runner.preflight()
        selected = [
            (side, joint)
            for side in SIDES
            for joint in range(1, 7)
            if (args.side is None or side == args.side)
            and (args.joint is None or joint == args.joint)
        ]
        directions: dict[tuple[str, int], int] = {}
        for side, joint in selected:
            index = (0 if side == "left" else 6) + joint - 1
            for direction in (1, -1):
                target = initial.copy()
                target[index] += direction * math.radians(args.delta_deg)
                try:
                    check_path(checker, initial, target)
                except ValueError:
                    continue
                directions[(side, joint)] = direction
                break
            if (side, joint) not in directions:
                raise RuntimeError(f"No checked direction for {side} J{joint}")
        (args.output_dir / "manifest.json").write_text(
            json.dumps(
                {
                    "delta_deg": args.delta_deg,
                    "mode": args.mode,
                    "ports": ports,
                    "initial_raw_q": initial.tolist(),
                    "kp": KP.tolist(),
                    "kd": KD.tolist(),
                    "workspace_checker": str(CHECKER),
                    "directions": {
                        f"{side}_J{joint}": direction
                        for (side, joint), direction in directions.items()
                    },
                    "can_motor_control": str(can_motor_control.__file__),
                },
                indent=2,
            )
        )
        modes = ("baseline", "candidate") if args.mode == "both" else (args.mode,)
        for mode in modes:
            runner.preflight()
            runner.connect(mode, initial)
            try:
                for side, joint in selected:
                    if stop_requested:
                        raise InterruptedError("Stop requested")
                    result = runner.trial(side, joint, directions[(side, joint)], mode)
                    results.append(result)
                    print(json.dumps(result), flush=True)
                if runner.reference is not None:
                    runner.move(runner.reference, mode + ":final_return", speed=0.025)
                    runner.hold(runner.reference, 3.0, mode + ":final_hold")
            finally:
                if runner.adapter is not None:
                    try:
                        if runner.reference is not None:
                            runner.recovering = True
                            runner.move(runner.reference, mode + ":guarded_return", speed=0.02)
                            runner.hold(runner.reference, 2.0, mode + ":guarded_hold")
                    except Exception as recovery_error:
                        print(f"Guarded return failed: {recovery_error!r}", flush=True)
                    runner.close()
                    runner.recovering = False
            wait_disabled(monitor)
            # if np.max(np.abs(returned - initial)) > 0.04:
            #     raise RuntimeError("Return pose differs from initial by more than 0.04 rad")
    except BaseException as exc:
        traceback.print_exc()
        error = repr(exc)
        print(f"Test stopped: {error}", flush=True)
    finally:
        try:
            runner.close()
        except Exception as close_error:
            if error is None:
                error = repr(close_error)
        try:
            wait_disabled(monitor)
            final_disabled = True
        except RuntimeError as verify_error:
            final_disabled = False
            if error is None:
                error = repr(verify_error)
        samples.close()
        monitor.close()
        (args.output_dir / "results.json").write_text(
            json.dumps(
                {
                    "finished": error is None,
                    "error": error,
                    "all_arm_motors_disabled": final_disabled,
                    "trials": results,
                },
                indent=2,
            )
        )
        if results:
            with (args.output_dir / "results.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(results[0]))
                writer.writeheader()
                writer.writerows(results)
            baseline = {
                (row["side"], row["joint"], row["direction"]): row
                for row in results
                if row["mode"] == "baseline"
            }
            candidate = {
                (row["side"], row["joint"], row["direction"]): row
                for row in results
                if row["mode"] == "candidate"
            }
            comparison = [
                {
                    "side": side,
                    "joint": joint,
                    "direction": direction,
                    "baseline_abs_offset_deg": abs(baseline[key]["offset_deg"]),
                    "candidate_abs_offset_deg": abs(candidate[key]["offset_deg"]),
                    "improvement_deg": abs(baseline[key]["offset_deg"])
                    - abs(candidate[key]["offset_deg"]),
                    "candidate_max_abs_tracking_bias": candidate[key]["max_abs_tracking_bias"],
                }
                for key in sorted(baseline.keys() & candidate.keys())
                for side, joint, direction in [key]
            ]
            if comparison:
                with (args.output_dir / "comparison.csv").open("w", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(comparison[0]))
                    writer.writeheader()
                    writer.writerows(comparison)
    return 0 if error is None else 1


if __name__ == "__main__":
    raise SystemExit(main())
