# Copyright 2025-2026 Dimensional Inc.
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

"""Shared-memory buffers for sim-manipulator IPC.

Layout for exchanging joint state and commands between ``MujocoSimModule``
(which owns the physics engine) and ``ShmMujocoAdapter`` (which plugs into
ControlCoordinator). Modeled after ``dimos.simulation.mujoco.shared_memory``
(the Go2 SHM pattern).

Both sides derive a deterministic discovery name from the MJCF path. Each sim
run publishes a random generation there and creates its data buffers under that
generation, so a new run cannot be confused with orphaned buffers from an old one.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from multiprocessing import resource_tracker
from multiprocessing.shared_memory import SharedMemory
import os
from pathlib import Path
import secrets
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.utils.logging_config import setup_logger
from dimos.utils.shm import ShmNotReadyError, attach_shm, create_or_attach_shm, unregister

logger = setup_logger()

# Upper bound on joint count per sim.  Manipulators use <=10; humanoids
# (Unitree G1: 29) push higher.  32 leaves headroom while keeping all
# per-joint buffers tiny (32 floats = 256 B).
MAX_JOINTS = 32
_FLOAT_BYTES = 8  # float64
_INT32_BYTES = 4

# IMU layout: quat (4) + gyro (3) + accel (3) = 10 floats.
_IMU_FLOATS = 10

_joint_array_size = MAX_JOINTS * _FLOAT_BYTES  # float64 array

# Gripper segment slots: position, target, range_lo, range_hi.
_GRP_SLOTS = 4

# One aligned int64 makes generation publication atomic for readers.
_DISCOVERY_FIELDS = 1
_DISCOVERY_BYTES = _DISCOVERY_FIELDS * _FLOAT_BYTES
_DISCOVERY_GENERATION = 0
_OWNER_FIELDS = 2
_OWNER_PID = 0
_OWNER_HEARTBEAT_NS = 1
_HEARTBEAT_TIMEOUT_NS = 5_000_000_000

# Element counts for control and sequence arrays.
_NUM_CTRL_FIELDS = 5  # [ready, stop, command_mode, num_joints, arm_joints]
_NUM_SEQ_COUNTERS = 12  # one per buffer type (manipulator + WB additions)

# Buffer sizes (in bytes).
# Keys are short to stay under macOS PSHMNAMLEN (31 bytes).
_shm_sizes = {
    # Manipulator-shared layout
    "pos": _joint_array_size,
    "vel": _joint_array_size,
    "eff": _joint_array_size,
    "pos_t": _joint_array_size,
    "vel_t": _joint_array_size,
    # [gripper_qpos, gripper_target, range_lo, range_hi]. gripper_qpos is the
    # raw MJCF joint value; gripper_target is in the command coordinate, which
    # runs the other way (see ManipShmReader.read_gripper_position).
    "grp": _GRP_SLOTS * _FLOAT_BYTES,
    # Whole-body additions (unused by manipulator path).
    "imu": _IMU_FLOATS * _FLOAT_BYTES,  # [w,x,y,z, gx,gy,gz, ax,ay,az]
    "kp_t": _joint_array_size,  # per-joint position-gain target
    "kd_t": _joint_array_size,  # per-joint velocity-gain target
    "tau_t": _joint_array_size,  # per-joint feedforward torque
    # Bookkeeping
    "owner": _OWNER_FIELDS * _FLOAT_BYTES,  # creator PID and heartbeat time_ns
    "seq": _NUM_SEQ_COUNTERS * _FLOAT_BYTES,  # int64 counters
    "ctl": _NUM_CTRL_FIELDS * _INT32_BYTES,  # [ready, stop, command_mode, num_joints]
}

# Sequence counter indices.
SEQ_POSITIONS = 0
SEQ_VELOCITIES = 1
SEQ_EFFORTS = 2
SEQ_POSITION_CMD = 3
SEQ_VELOCITY_CMD = 4
SEQ_GRIPPER_STATE = 5
SEQ_GRIPPER_CMD = 6
# Whole-body additions
SEQ_IMU = 7
SEQ_KP_CMD = 8
SEQ_KD_CMD = 9
SEQ_TAU_CMD = 10

# Control indices.
CTRL_READY = 0
CTRL_STOP = 1
CTRL_COMMAND_MODE = 2
CTRL_NUM_JOINTS = 3
CTRL_ARM_JOINTS = 4

# Command modes.
CMD_MODE_POSITION = 0
CMD_MODE_VELOCITY = 1
# Whole-body PD-with-feedforward: ctrl = kp*(q_t - q) + kd*(0 - dq) + tau_t.
# Per-step kp/kd lets a policy retune gains online if it wants to.
CMD_MODE_PD_TAU = 2

_NAME_PREFIX = "dmjm"


def shm_key_from_path(config_path: Path | str) -> str:
    """Derive a deterministic short key from an MJCF path.

    Both sim module and adapter compute the same key from the same path,
    so SHM buffer names can be agreed upon without an RPC round-trip.
    """
    resolved = str(Path(config_path).expanduser().resolve())
    return hashlib.md5(resolved.encode("utf-8")).hexdigest()[:12]


_ATTACH_WINDOW_S = 0.5


def _buffer_name(key: str, buffer: str) -> str:
    return f"{_NAME_PREFIX}_{key}_{buffer}"


def _generation_key(generation: int) -> str:
    return f"{generation:016x}"


def _unregister(shm: SharedMemory) -> SharedMemory:
    """Detach ``shm`` from ``resource_tracker`` to silence spurious warnings.

    Same technique as ``dimos.simulation.mujoco.shared_memory._unregister``.
    """
    try:
        resource_tracker.unregister(shm._name, "shared_memory")  # type: ignore[attr-defined]
    except Exception:
        pass
    return shm


class _Discovery:
    def __init__(self, key: str, *, create: bool) -> None:
        name = _buffer_name(key, "gen")
        if create:
            self.shm, owner = create_or_attach_shm(name, _DISCOVERY_BYTES)
            if owner:
                unregister(self.shm)
        else:
            self.shm = attach_shm(name, timeout=_ATTACH_WINDOW_S)

    def publish(self, generation: int) -> None:
        self._array()[_DISCOVERY_GENERATION] = generation

    def read(self) -> int:
        return int(self._array()[_DISCOVERY_GENERATION])

    def close(self) -> None:
        try:
            self.shm.close()
        except (FileNotFoundError, OSError):
            pass

    def _array(self) -> NDArray[np.int64]:
        return np.ndarray((_DISCOVERY_FIELDS,), dtype=np.int64, buffer=self.shm.buf)


@dataclass(frozen=True)
class ManipShmSet:
    """Frozen set of named SharedMemory buffers for sim <-> adapter IPC.

    Despite the name (kept for backward compat with existing manipulator
    consumers), the layout now also covers whole-body needs: IMU, per-joint
    PD gain commands, and per-joint feedforward torque commands.  The
    extra buffers are unused by the manipulator path.
    """

    pos: SharedMemory
    vel: SharedMemory
    eff: SharedMemory
    pos_t: SharedMemory
    vel_t: SharedMemory
    grp: SharedMemory
    # Whole-body additions
    imu: SharedMemory
    kp_t: SharedMemory
    kd_t: SharedMemory
    tau_t: SharedMemory
    # Bookkeeping
    owner: SharedMemory
    seq: SharedMemory
    ctl: SharedMemory

    @classmethod
    def create(cls, key: str) -> ManipShmSet:
        """Create a complete generation of SHM buffers."""
        buffers: dict[str, SharedMemory] = {}
        try:
            for buffer_name, size in _shm_sizes.items():
                name = _buffer_name(key, buffer_name)
                try:
                    stale = _unregister(SharedMemory(name=name))
                    stale.close()
                    try:
                        stale.unlink()
                        logger.info("ManipShmSet: unlinked stale SHM", name=name)
                    except FileNotFoundError:
                        pass
                except FileNotFoundError:
                    pass
                buffers[buffer_name] = SharedMemory(create=True, size=size, name=name)
        except Exception:
            cls._close_buffers(buffers, unlink=True)
            raise
        return cls(**buffers)

    @classmethod
    def attach(cls, key: str) -> ManipShmSet:
        """Attach to existing SHM buffers created by the sim side."""
        buffers: dict[str, SharedMemory] = {}
        try:
            for buffer_name in _shm_sizes:
                name = _buffer_name(key, buffer_name)
                buffers[buffer_name] = attach_shm(name, timeout=_ATTACH_WINDOW_S)
        except Exception:
            cls._close_buffers(buffers, unlink=False)
            raise
        return cls(**buffers)

    def as_list(self) -> list[SharedMemory]:
        return [getattr(self, k) for k in _shm_sizes]

    @staticmethod
    def _close_buffers(buffers: dict[str, SharedMemory], *, unlink: bool) -> None:
        for shm in buffers.values():
            try:
                shm.close()
            except (FileNotFoundError, OSError):
                pass
            if unlink:
                try:
                    shm.unlink()
                except (FileNotFoundError, OSError):
                    pass


class ManipShmWriter:
    """Sim-side handle: writes joint state, reads command targets.
    Owned by ``MujocoSimModule``. Creates the SHM buffers on init and
    unlinks them on cleanup.
    """

    shm: ManipShmSet

    def __init__(self, key: str) -> None:
        self._discovery = _Discovery(key, create=True)
        self._generation = secrets.randbits(63) or 1
        try:
            self.shm = ManipShmSet.create(_generation_key(self._generation))
        except Exception:
            self._discovery.close()
            raise
        self._last_pos_cmd_seq = 0
        self._last_vel_cmd_seq = 0
        self._last_gripper_cmd_seq = 0
        self._last_kp_cmd_seq = 0
        self._last_kd_cmd_seq = 0
        self._last_tau_cmd_seq = 0
        self._closed = False
        # Zero everything.
        for buf in self.shm.as_list():
            np.ndarray((buf.size,), dtype=np.uint8, buffer=buf.buf)[:] = 0
        self._owner()[_OWNER_PID] = os.getpid()
        self._discovery.publish(self._generation)

    def write_joint_state(
        self,
        positions: list[float],
        velocities: list[float],
        efforts: list[float],
    ) -> None:
        n = min(len(positions), MAX_JOINTS)
        pos_arr = self._array(self.shm.pos, MAX_JOINTS, np.float64)
        vel_arr = self._array(self.shm.vel, MAX_JOINTS, np.float64)
        eff_arr = self._array(self.shm.eff, MAX_JOINTS, np.float64)
        pos_arr[:n] = positions[:n]
        vel_arr[:n] = velocities[:n]
        eff_arr[:n] = efforts[:n]
        self._owner()[_OWNER_HEARTBEAT_NS] = time.monotonic_ns()
        self._increment_seq(SEQ_POSITIONS)
        self._increment_seq(SEQ_VELOCITIES)
        self._increment_seq(SEQ_EFFORTS)

    def write_gripper_state(self, position: float) -> None:
        """Publish the gripper's raw MJCF joint value."""
        arr = self._array(self.shm.grp, _GRP_SLOTS, np.float64)
        arr[0] = position
        self._increment_seq(SEQ_GRIPPER_STATE)

    def write_gripper_range(self, low: float, high: float) -> None:
        """Publish the gripper joint's MJCF range; written once at startup."""
        arr = self._array(self.shm.grp, _GRP_SLOTS, np.float64)
        arr[2] = low
        arr[3] = high

    def read_position_command(self, num_joints: int) -> NDArray[np.float64] | None:
        """Return a copy of position targets if a new command arrived since last call."""
        seq = self._get_seq(SEQ_POSITION_CMD)
        if seq <= self._last_pos_cmd_seq:
            return None
        self._last_pos_cmd_seq = seq
        arr = self._array(self.shm.pos_t, MAX_JOINTS, np.float64)
        result: NDArray[np.float64] = arr[:num_joints].copy()
        return result

    def read_velocity_command(self, num_joints: int) -> NDArray[np.float64] | None:
        seq = self._get_seq(SEQ_VELOCITY_CMD)
        if seq <= self._last_vel_cmd_seq:
            return None
        self._last_vel_cmd_seq = seq
        arr = self._array(self.shm.vel_t, MAX_JOINTS, np.float64)
        result: NDArray[np.float64] = arr[:num_joints].copy()
        return result

    def read_gripper_command(self) -> float | None:
        seq = self._get_seq(SEQ_GRIPPER_CMD)
        if seq <= self._last_gripper_cmd_seq:
            return None
        self._last_gripper_cmd_seq = seq
        arr = self._array(self.shm.grp, _GRP_SLOTS, np.float64)
        return float(arr[1])

    def read_command_mode(self) -> int:
        return int(self._control()[CTRL_COMMAND_MODE])

    # Whole-body additions

    def write_imu(
        self,
        quaternion: tuple[float, float, float, float],
        gyroscope: tuple[float, float, float],
        accelerometer: tuple[float, float, float],
    ) -> None:
        """Write IMU sample.  Quaternion is (w, x, y, z)."""
        arr = self._array(self.shm.imu, _IMU_FLOATS, np.float64)
        arr[0:4] = quaternion
        arr[4:7] = gyroscope
        arr[7:10] = accelerometer
        self._increment_seq(SEQ_IMU)

    def read_kp_command(self, num_joints: int) -> NDArray[np.float64] | None:
        """Per-joint position-gain target if a new command landed since last call."""
        seq = self._get_seq(SEQ_KP_CMD)
        if seq <= self._last_kp_cmd_seq:
            return None
        self._last_kp_cmd_seq = seq
        arr = self._array(self.shm.kp_t, MAX_JOINTS, np.float64)
        return arr[:num_joints].copy()

    def read_kd_command(self, num_joints: int) -> NDArray[np.float64] | None:
        seq = self._get_seq(SEQ_KD_CMD)
        if seq <= self._last_kd_cmd_seq:
            return None
        self._last_kd_cmd_seq = seq
        arr = self._array(self.shm.kd_t, MAX_JOINTS, np.float64)
        return arr[:num_joints].copy()

    def read_tau_command(self, num_joints: int) -> NDArray[np.float64] | None:
        """Per-joint feedforward torque if a new command landed since last call."""
        seq = self._get_seq(SEQ_TAU_CMD)
        if seq <= self._last_tau_cmd_seq:
            return None
        self._last_tau_cmd_seq = seq
        arr = self._array(self.shm.tau_t, MAX_JOINTS, np.float64)
        return arr[:num_joints].copy()

    def signal_ready(self, num_joints: int, arm_joints: int | None = None) -> None:
        ctrl = self._control()
        ctrl[CTRL_NUM_JOINTS] = num_joints
        ctrl[CTRL_ARM_JOINTS] = num_joints if arm_joints is None else arm_joints
        ctrl[CTRL_READY] = 1

    def signal_stop(self) -> None:
        self._control()[CTRL_STOP] = 1

    def should_stop(self) -> bool:
        return bool(self._control()[CTRL_STOP] == 1)

    def cleanup(self) -> None:
        if self._closed:
            return
        try:
            if self._discovery.read() == self._generation:
                self._discovery.publish(0)
            for shm in self.shm.as_list():
                try:
                    shm.close()
                except FileNotFoundError:
                    pass  # already detached
                except OSError as exc:
                    logger.warning("SHM close failed", name=shm.name, error=str(exc))
                try:
                    shm.unlink()
                except FileNotFoundError:
                    pass  # already unlinked (e.g. cleanup called twice)
                except OSError as exc:
                    logger.warning("SHM unlink failed", name=shm.name, error=str(exc))
            self._discovery.close()
        finally:
            self._closed = True

    def _array(self, buf: SharedMemory, n: int, dtype: Any) -> NDArray[Any]:
        return np.ndarray((n,), dtype=dtype, buffer=buf.buf)

    def _control(self) -> NDArray[np.int32]:
        return np.ndarray((_NUM_CTRL_FIELDS,), dtype=np.int32, buffer=self.shm.ctl.buf)

    def _owner(self) -> NDArray[np.int64]:
        return np.ndarray((_OWNER_FIELDS,), dtype=np.int64, buffer=self.shm.owner.buf)

    def _increment_seq(self, index: int) -> None:
        seq_arr = np.ndarray((_NUM_SEQ_COUNTERS,), dtype=np.int64, buffer=self.shm.seq.buf)
        seq_arr[index] += 1

    def _get_seq(self, index: int) -> int:
        seq_arr = np.ndarray((_NUM_SEQ_COUNTERS,), dtype=np.int64, buffer=self.shm.seq.buf)
        return int(seq_arr[index])


class ManipShmReader:
    """Adapter-side handle: reads joint state, writes command targets.

    Owned by ``ShmMujocoAdapter``. Attaches to existing buffers created by
    the sim module; does not unlink them on cleanup.
    """

    shm: ManipShmSet

    def __init__(self, key: str) -> None:
        self._discovery = _Discovery(key, create=False)
        self._generation = self._discovery.read()
        if self._generation <= 0:
            self._discovery.close()
            raise ShmNotReadyError("MuJoCo SHM discovery has no active generation")
        try:
            self.shm = ManipShmSet.attach(_generation_key(self._generation))
        except Exception:
            self._discovery.close()
            raise
        self._closed = False
        if not self.is_current_generation():
            self.cleanup()
            raise ShmNotReadyError("MuJoCo SHM generation changed during attachment")
        self._owner_pid = int(self._owner()[_OWNER_PID])

    @classmethod
    def wait_for_live(
        cls,
        key: str,
        *,
        attach_timeout_s: float,
        ready_timeout_s: float,
        poll_s: float,
    ) -> ManipShmReader | None:
        deadline = time.monotonic() + attach_timeout_s
        found_generation = False
        while time.monotonic() <= deadline:
            try:
                candidate = cls(key)
            except FileNotFoundError:
                time.sleep(poll_s)
                continue
            if not candidate.is_owner_current():
                candidate.cleanup()
                time.sleep(poll_s)
                continue
            if not found_generation:
                found_generation = True
                deadline = time.monotonic() + ready_timeout_s
            while time.monotonic() <= deadline:
                if not candidate.is_owner_current():
                    break
                if candidate.is_ready() and candidate.is_live():
                    return candidate
                time.sleep(poll_s)
            candidate.cleanup()
        return None

    def read_positions(self, num_joints: int) -> list[float]:
        arr = np.ndarray((MAX_JOINTS,), dtype=np.float64, buffer=self.shm.pos.buf)
        return [float(x) for x in arr[:num_joints]]

    def read_velocities(self, num_joints: int) -> list[float]:
        arr = np.ndarray((MAX_JOINTS,), dtype=np.float64, buffer=self.shm.vel.buf)
        return [float(x) for x in arr[:num_joints]]

    def read_efforts(self, num_joints: int) -> list[float]:
        arr = np.ndarray((MAX_JOINTS,), dtype=np.float64, buffer=self.shm.eff.buf)
        return [float(x) for x in arr[:num_joints]]

    def read_gripper_position(self) -> float:
        """Measured gripper position in the coordinate commands are given in.

        The sim publishes the raw MJCF joint value, which runs opposite to the
        command coordinate (``MujocoSimModule._gripper_joint_to_ctrl`` inverts),
        so reflect it about the joint range. Without that reflection a closed
        gripper reads as the open end of its range.
        """
        arr = np.ndarray((_GRP_SLOTS,), dtype=np.float64, buffer=self.shm.grp.buf)
        low, high = float(arr[2]), float(arr[3])
        if high <= low:
            return float(arr[0])
        return low + high - float(arr[0])

    def read_gripper_range(self) -> tuple[float, float]:
        """Read the published MJCF gripper range; (0.0, 1.0) if undeclared."""
        arr = np.ndarray((_GRP_SLOTS,), dtype=np.float64, buffer=self.shm.grp.buf)
        low, high = float(arr[2]), float(arr[3])
        return (low, high) if high > low else (0.0, 1.0)

    def write_position_command(self, positions: list[float]) -> None:
        n = min(len(positions), MAX_JOINTS)
        arr = np.ndarray((MAX_JOINTS,), dtype=np.float64, buffer=self.shm.pos_t.buf)
        arr[:n] = positions[:n]
        self._set_command_mode(CMD_MODE_POSITION)
        self._increment_seq(SEQ_POSITION_CMD)

    def write_velocity_command(self, velocities: list[float]) -> None:
        n = min(len(velocities), MAX_JOINTS)
        arr = np.ndarray((MAX_JOINTS,), dtype=np.float64, buffer=self.shm.vel_t.buf)
        arr[:n] = velocities[:n]
        self._set_command_mode(CMD_MODE_VELOCITY)
        self._increment_seq(SEQ_VELOCITY_CMD)

    def write_gripper_command(self, position: float) -> None:
        arr = np.ndarray((_GRP_SLOTS,), dtype=np.float64, buffer=self.shm.grp.buf)
        arr[1] = position
        self._increment_seq(SEQ_GRIPPER_CMD)

    # Whole-body additions

    def read_imu(
        self,
    ) -> tuple[
        tuple[float, float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]:
        """Read IMU sample: ((qw, qx, qy, qz), (gx, gy, gz), (ax, ay, az))."""
        arr = np.ndarray((_IMU_FLOATS,), dtype=np.float64, buffer=self.shm.imu.buf)
        return (
            (float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3])),
            (float(arr[4]), float(arr[5]), float(arr[6])),
            (float(arr[7]), float(arr[8]), float(arr[9])),
        )

    def write_kp_command(self, kp: list[float]) -> None:
        """Per-joint position-gain target.  Switches command mode to PD+tau."""
        n = min(len(kp), MAX_JOINTS)
        arr = np.ndarray((MAX_JOINTS,), dtype=np.float64, buffer=self.shm.kp_t.buf)
        arr[:n] = kp[:n]
        self._set_command_mode(CMD_MODE_PD_TAU)
        self._increment_seq(SEQ_KP_CMD)

    def write_kd_command(self, kd: list[float]) -> None:
        n = min(len(kd), MAX_JOINTS)
        arr = np.ndarray((MAX_JOINTS,), dtype=np.float64, buffer=self.shm.kd_t.buf)
        arr[:n] = kd[:n]
        self._set_command_mode(CMD_MODE_PD_TAU)
        self._increment_seq(SEQ_KD_CMD)

    def write_tau_command(self, tau: list[float]) -> None:
        """Per-joint feedforward torque, applied on top of PD."""
        n = min(len(tau), MAX_JOINTS)
        arr = np.ndarray((MAX_JOINTS,), dtype=np.float64, buffer=self.shm.tau_t.buf)
        arr[:n] = tau[:n]
        self._set_command_mode(CMD_MODE_PD_TAU)
        self._increment_seq(SEQ_TAU_CMD)

    def write_pd_tau_command(
        self,
        positions: list[float],
        kp: list[float],
        kd: list[float],
        tau: list[float],
    ) -> None:
        """Write a whole-body PD+tau command without transient mode flips.

        The sim engine runs in a different process, so setting position mode
        first and PD mode later creates a small but real race. Write all arrays,
        publish PD mode once, then bump the sequence counters.
        """
        n_pos = min(len(positions), MAX_JOINTS)
        n_kp = min(len(kp), MAX_JOINTS)
        n_kd = min(len(kd), MAX_JOINTS)
        n_tau = min(len(tau), MAX_JOINTS)
        np.ndarray((MAX_JOINTS,), dtype=np.float64, buffer=self.shm.pos_t.buf)[:n_pos] = positions[
            :n_pos
        ]
        np.ndarray((MAX_JOINTS,), dtype=np.float64, buffer=self.shm.kp_t.buf)[:n_kp] = kp[:n_kp]
        np.ndarray((MAX_JOINTS,), dtype=np.float64, buffer=self.shm.kd_t.buf)[:n_kd] = kd[:n_kd]
        np.ndarray((MAX_JOINTS,), dtype=np.float64, buffer=self.shm.tau_t.buf)[:n_tau] = tau[:n_tau]
        self._set_command_mode(CMD_MODE_PD_TAU)
        self._increment_seq(SEQ_KP_CMD)
        self._increment_seq(SEQ_KD_CMD)
        self._increment_seq(SEQ_TAU_CMD)
        # Position is the engine-side trigger for latching a new PD target,
        # so publish it last after gains/torque are visible.
        self._increment_seq(SEQ_POSITION_CMD)

    def is_ready(self) -> bool:
        return bool(self._control()[CTRL_READY] == 1)

    def owner_alive(self) -> bool:
        if self._owner_pid <= 0:
            return False
        try:
            os.kill(self._owner_pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def is_current_generation(self) -> bool:
        return self._discovery.read() == self._generation

    def is_owner_current(self) -> bool:
        return self.owner_alive() and self.is_current_generation() and not self.should_stop()

    def is_live(self) -> bool:
        heartbeat_ns = int(self._owner()[_OWNER_HEARTBEAT_NS])
        heartbeat_fresh = 0 <= time.monotonic_ns() - heartbeat_ns <= _HEARTBEAT_TIMEOUT_NS
        return heartbeat_fresh and self.is_owner_current()

    def should_stop(self) -> bool:
        return bool(self._control()[CTRL_STOP] == 1)

    def num_joints(self) -> int:
        return int(self._control()[CTRL_NUM_JOINTS])

    def arm_joints(self) -> int:
        return int(self._control()[CTRL_ARM_JOINTS])

    def signal_stop(self) -> None:
        self._control()[CTRL_STOP] = 1

    def cleanup(self) -> None:
        if self._closed:
            return
        try:
            for shm in self.shm.as_list():
                try:
                    shm.close()
                except FileNotFoundError:
                    pass  # already detached
                except OSError as exc:
                    logger.warning("SHM close failed", name=shm.name, error=str(exc))
            self._discovery.close()
        finally:
            self._closed = True

    def _control(self) -> NDArray[np.int32]:
        return np.ndarray((_NUM_CTRL_FIELDS,), dtype=np.int32, buffer=self.shm.ctl.buf)

    def _owner(self) -> NDArray[np.int64]:
        return np.ndarray((_OWNER_FIELDS,), dtype=np.int64, buffer=self.shm.owner.buf)

    def _set_command_mode(self, mode: int) -> None:
        self._control()[CTRL_COMMAND_MODE] = mode

    def _increment_seq(self, index: int) -> None:
        seq_arr = np.ndarray((_NUM_SEQ_COUNTERS,), dtype=np.int64, buffer=self.shm.seq.buf)
        seq_arr[index] += 1
