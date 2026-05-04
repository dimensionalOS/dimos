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

"""Unitree Go2 TwistBase adapter — SDK2 high-level control plane.

Implements TwistBaseAdapter (3 DOF: vx, vy, wz) on top of unitree_sdk2py.
Connects via DDS (ChannelFactoryInitialize → MotionSwitcher → SportClient
→ StandUp → FreeWalk). No Rage Mode by default (opt-in via
rage_mode=True) by publishing synthesized WirelessController_ messages
on rt/wirelesscontroller_unprocessed.

"""

from __future__ import annotations

from dataclasses import dataclass
import json
import threading
import time
from typing import TYPE_CHECKING, Any

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.default import unitree_go_msg_dds__WirelessController_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import (
    SportModeState_,
    WirelessController_,
)

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.hardware.drive_trains.registry import TwistBaseAdapterRegistry

logger = setup_logger()


def _clip(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass
class _Session:
    """Active connection state for a Go2.

    The session object is created by connect() and set on the adapter under
    _session_lock. All mutable state that can be touched by both the DDS
    callback thread and the control thread lives here, guarded by `lock`.
    """

    client: SportClient
    motion_switcher: MotionSwitcherClient
    lock: threading.Lock
    state_sub: ChannelSubscriber | None = None
    latest_state: SportModeState_ | None = None
    enabled: bool = False
    locomotion_ready: bool = False

    # Rage Mode joystick publisher (rt/wirelesscontroller_unprocessed path)
    rage_active: bool = False
    rage_pub: ChannelPublisher | None = None
    rage_thread: threading.Thread | None = None
    rage_stop: threading.Event | None = None
    rage_cmd: tuple[float, float, float] = (0.0, 0.0, 0.0)


class UnitreeGo2TwistAdapter:
    """TwistBaseAdapter for the Unitree Go2 quadruped via unitree_sdk2py (DDS).

    3 DOF velocity: [vx, vy, wz].
      - vx: forward/backward linear velocity (m/s)
      - vy: lateral (left positive) linear velocity (m/s)
      - wz: yaw rate (rad/s)

    Thread model:
      - _session_lock guards the self._session reference across threads.
      - session.lock guards latest_state and SportClient RPC serialization.
      Never take _session_lock while holding session.lock - the DDS callback
      already holds session.lock briefly during state updates.
    """

    # AI-controller API ID for the Rage Mode toggle.
    _SPORT_API_ID_RAGEMODE: int = 2059

    # Rage velocity envelope (m/s, m/s, rad/s) from rage_mode_export_cfg.json.
    _RAGE_UP_VX: float = 2.5
    _RAGE_UP_VY: float = 1.0
    _RAGE_UP_VYAW: float = 5.0

    _RAGE_PUBLISH_HZ: float = 100.0
    _RAGE_LY_SIGN: float = 1.0  # vx → ly
    _RAGE_LX_SIGN: float = -1.0  # vy → lx
    _RAGE_RX_SIGN: float = -1.0  # wz → rx

    def __init__(
        self,
        dof: int = 3,
        speed_level: int = 1,
        rage_mode: bool = False,
        **_: Any,
    ) -> None:
        if dof != 3:
            raise ValueError(f"Go2 only supports 3 DOF (vx, vy, wz), got {dof}")

        self._session: _Session | None = None
        self._session_lock = threading.Lock()
        self._speed_level = speed_level
        self._rage_mode_default = rage_mode
        self._last_guard_warn_ts: float = 0.0

    def connect(self) -> bool:
        """Connect to Go2, verify sport mode, stand up, enter FreeWalk.

        Sequence:
          1. ChannelFactoryInitialize(0) — default domain, default NIC.
          2. MotionSwitcher.Init + poll CheckMode() until a sport mode
             is reported (DDS discovery) or _DISCOVERY_TIMEOUT_S elapses.
          3. Subscribe rt/sportmodestate for telemetry.
          4. SportClient.Init.
          5. _initialize_locomotion(): StandUp + FreeWalk + SpeedLevel.
          6. If rage_mode=True, set_rage_mode(True).

        Returns True on success, False on connect/init/locomotion
        failure. On failure, logs guidance and the adapter stays in a
        clean "not connected" state so a retry can succeed.
        """
        with self._session_lock:
            if self._session is not None:
                logger.warning("[Go2] Already connected — disconnect first")
                return False

        # ChannelFactoryInitialize raises if the factory already exists.
        try:
            ChannelFactoryInitialize(0)
        except Exception:
            pass

        motion_switcher = MotionSwitcherClient()
        motion_switcher.SetTimeout(0.5)
        motion_switcher.Init()

        # Poll CheckMode() through DDS discovery
        mode = ""
        for _ in range(50):
            try:
                code, data = motion_switcher.CheckMode()
            except (OSError, RuntimeError, TimeoutError):
                time.sleep(0.1)
                continue
            if code == 0 and isinstance(data, dict):
                mode = (data.get("name") or "").strip()
                if mode:
                    break
            time.sleep(0.1)
        motion_switcher.SetTimeout(5.0)
        if not mode:
            logger.error("[Go2] No sport mode active")
            return False
        logger.info(f"[Go2] Sport mode '{mode}' active")

        client = SportClient()
        client.SetTimeout(10.0)

        session = _Session(
            client=client,
            motion_switcher=motion_switcher,
            lock=threading.Lock(),
        )

        def state_callback(msg: SportModeState_) -> None:
            with session.lock:
                session.latest_state = msg

        state_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        state_sub.Init(state_callback, 10)
        session.state_sub = state_sub

        with self._session_lock:
            self._session = session

        # disconnect() must run on any failure
        try:
            client.Init()
            logger.info("[Go2] Connected")

            if not self._initialize_locomotion():
                logger.error("[Go2] Failed to initialize locomotion mode")
                self.disconnect()
                return False

            if self._rage_mode_default and not self.set_rage_mode(True):
                logger.warning("[Go2] Rage Mode enable failed — continuing with regular locomotion")
        except Exception:
            self.disconnect()
            raise

        return True

    def disconnect(self) -> None:
        """Stop motion, stand the robot down, and tear down DDS resources.

        Safe to call multiple times. Explicitly Close()s the state
        subscriber to prevent DDS reader leaks across reconnects.
        """
        with self._session_lock:
            session = self._session
            self._session = None

        if session is None:
            return

        self._stop_rage_joystick(session)
        try:
            with session.lock:
                session.client.StopMove()
            with session.lock:
                session.client.StandDown()
        except (OSError, RuntimeError, TimeoutError) as e:
            logger.error(f"[Go2] Error during disconnect: {e}")

        if session.state_sub is not None:
            try:
                session.state_sub.Close()
            except (OSError, RuntimeError) as e:
                logger.error(f"[Go2] Error closing state subscriber: {e}")

    def is_connected(self) -> bool:
        with self._session_lock:
            return self._session is not None

    def get_dof(self) -> int:
        """Always 3 for Go2 (vx, vy, wz)."""
        return 3

    def read_velocities(self) -> list[float]:
        """Measured velocities [vx, vy, wz] from SportModeState_.

        Sources:
          vx, vy: state.velocity[0], state.velocity[1]
          wz:     state.imu_state.gyroscope[2]

        Returns [0.0, 0.0, 0.0] during the startup gap before the first
        DDS callback has populated latest_state.
        """
        session = self._get_session()
        with session.lock:
            if session.latest_state is None:
                return [0.0, 0.0, 0.0]
            state = session.latest_state
            return [
                float(state.velocity[0]),
                float(state.velocity[1]),
                float(state.imu_state.gyroscope[2]),
            ]

    def read_odometry(self) -> list[float] | None:
        """Measured pose [x, y, theta] from SportModeState_.

        Sources:
          x, y:  state.position[0], state.position[1]
          theta: state.imu_state.rpy[2]  (yaw)

        Returns None if no state message has arrived yet.
        """
        session = self._get_session()
        with session.lock:
            if session.latest_state is None:
                return None
            state = session.latest_state
            return [
                float(state.position[0]),
                float(state.position[1]),
                float(state.imu_state.rpy[2]),
            ]

    def write_velocities(self, velocities: list[float]) -> bool:
        """Send a Twist command [vx, vy, wz] to the Go2.

        When Rage Mode is active, the command is stashed in
        session.rage_cmd and the 100 Hz joystick publisher thread picks
        it up on its next tick (Rage's FSM ignores SportClient.Move).
        Otherwise the command is forwarded directly via
        SportClient.Move() → FsmFreeWalk.

        Refuses (returns False) if:
          - len(velocities) != 3
          - session not enabled (write_enable(True) not called)
          - locomotion not ready (StandUp/FreeWalk incomplete)

        Guard warnings are rate-limited to 1 Hz since this is called
        at 100 Hz from the tick loop.
        """
        if len(velocities) != 3:
            return False

        session = self._get_session()

        if not session.enabled:
            self._warn_guard("Not enabled, ignoring velocity command")
            return False

        if not session.locomotion_ready:
            self._warn_guard("Locomotion not ready, ignoring velocity command")
            return False

        vx, vy, wz = velocities

        if session.rage_active:
            session.rage_cmd = (vx, vy, wz)
            return True

        return self._send_velocity(vx, vy, wz)

    def _warn_guard(self, msg: str) -> None:
        """Rate-limited guard warning (at most once per second).

        write_velocities runs at 100 Hz from the tick loop; without
        throttling, a sustained guard miss would emit 100 warnings/s.
        """
        now = time.monotonic()
        if now - self._last_guard_warn_ts < 1.0:
            return
        self._last_guard_warn_ts = now
        logger.warning(f"[Go2] {msg}")

    def write_stop(self) -> bool:
        """Stop motion via SportClient.StopMove(). Leaves robot standing."""
        session = self._get_session()
        with session.lock:
            session.client.StopMove()
        return True

    def write_enable(self, enable: bool) -> bool:
        """Enable/disable velocity command path.

        enable=True: ensures locomotion is ready (re-initializes if needed),
                     then flips session.enabled.
        enable=False: calls write_stop() and clears session.enabled. Does
                     NOT stand the robot down — use disconnect() for that.
        """
        session = self._get_session()

        if enable:
            if not session.locomotion_ready:
                if not self._initialize_locomotion():
                    logger.error("[Go2] Failed to initialize locomotion")
                    return False
            session.enabled = True
            logger.info("[Go2] Enabled")
            return True

        self.write_stop()
        session.enabled = False
        logger.info("[Go2] Disabled")
        return True

    def read_enabled(self) -> bool:
        with self._session_lock:
            return self._session is not None and self._session.enabled

    def check_mode(self) -> str | None:
        """Return the current MotionSwitcher mode name, or None on RPC fail.

        Wraps MotionSwitcher.CheckMode(). Empty string means no controller
        active; None means the RPC returned a non-zero code or non-dict data.
        """
        session = self._get_session()
        code, data = session.motion_switcher.CheckMode()
        if code == 0 and isinstance(data, dict):
            return (data.get("name") or "").strip()
        return None

    def get_sport_state(self) -> SportModeState_ | None:
        """Return the latest SportModeState_ snapshot for diagnostics.

        Returned object is the live SDK message — do not mutate it. None
        if no state message has arrived.
        """
        session = self._get_session()
        with session.lock:
            return session.latest_state

    def get_status(self) -> dict[str, Any]:
        """One-shot snapshot of adapter + robot state"""
        with self._session_lock:
            session = self._session

        if session is None:
            return {
                "connected": False,
                "mode": None,
                "enabled": False,
                "locomotion_ready": False,
                "rage_active": False,
                "speed_level": self._speed_level,
                "has_state": False,
                "velocity": None,
                "position": None,
                "body_height": None,
                "sport_mode_num": None,
            }

        mode = self.check_mode()

        with session.lock:
            state = session.latest_state
            enabled = session.enabled
            locomotion_ready = session.locomotion_ready
            rage_active = session.rage_active

        velocity: list[float] | None = None
        position: list[float] | None = None
        body_height: float | None = None
        sport_mode_num: int | None = None

        if state is not None:
            try:
                velocity = [
                    float(state.velocity[0]),
                    float(state.velocity[1]),
                    float(state.imu_state.gyroscope[2]),
                ]
                position = [
                    float(state.position[0]),
                    float(state.position[1]),
                    float(state.imu_state.rpy[2]),
                ]
                body_height = float(state.body_height)
                sport_mode_num = int(state.mode)
            except (AttributeError, IndexError, TypeError, ValueError):
                pass

        return {
            "connected": True,
            "mode": mode,
            "enabled": enabled,
            "locomotion_ready": locomotion_ready,
            "rage_active": rage_active,
            "speed_level": self._speed_level,
            "has_state": state is not None,
            "velocity": velocity,
            "position": position,
            "body_height": body_height,
            "sport_mode_num": sport_mode_num,
        }

    def set_speed_level(self, level: int) -> bool:
        """Set the SportClient speed envelope at runtime.

        Go2 SDK convention: -1 = slow, 0 = normal, 1 = fast (max). When
        Rage is active, the Rage envelope (_RAGE_UP_VX etc.) applies
        instead. Updates self._speed_level so subsequent
        _initialize_locomotion() calls apply the same level.

        Returns True if the RPC returned 0.
        """
        session = self._get_session()
        with session.lock:
            ret = session.client.SpeedLevel(level)

        if ret != 0:
            logger.warning(f"[Go2] SpeedLevel({level}) returned {ret}")
            return False

        self._speed_level = level
        logger.info(f"[Go2] SpeedLevel set to {level}")
        return True

    def set_rage_mode(self, enable: bool) -> bool:
        """Toggle Rage Mode (api_id 2059) — widens forward envelope to ~2.5 m/s.

        Velocity input flows via rt/wirelesscontroller_unprocessed, not
        SportClient.Move (FsmRageMode isn't in AiController::Move's dispatch).
        Idempotent. Returns True on 2059 success; publisher/SwitchJoystick
        failures are logged but don't fail the call.
        """
        session = self._get_session()

        if session.rage_active == enable:
            return True

        with session.lock:
            ret = session.client.BalanceStand()
        if ret != 0:
            # Non-zero is usually benign here (already balanced / FSM transition
            # in progress) — only fatal if the rage toggle below also fails.
            logger.info(f"[Go2] BalanceStand returned {ret} (likely already balanced — proceeding)")
        time.sleep(0.3)

        if not self._call_sport_api(self._SPORT_API_ID_RAGEMODE, {"data": enable}):
            return False

        if enable:
            time.sleep(2.0)  # let FsmRageMode transition settle
            self._start_rage_joystick(session)
            with session.lock:
                sj_ret = session.client.SwitchJoystick(True)
            if sj_ret != 0:
                logger.warning(f"[Go2] SwitchJoystick(True) after rage returned {sj_ret}")
        else:
            self._stop_rage_joystick(session)
            with session.lock:
                sj_ret = session.client.SwitchJoystick(False)
            if sj_ret != 0:
                logger.warning(f"[Go2] SwitchJoystick(False) after rage returned {sj_ret}")

        logger.info(f"[Go2] Rage Mode {'enabled' if enable else 'disabled'}")
        return True

    def _start_rage_joystick(self, session: _Session) -> None:
        """Create the WirelessController publisher and spawn the 100Hz thread."""
        if session.rage_pub is not None:
            return
        pub = ChannelPublisher("rt/wirelesscontroller_unprocessed", WirelessController_)
        pub.Init()
        session.rage_pub = pub

        session.rage_stop = threading.Event()
        session.rage_cmd = (0.0, 0.0, 0.0)
        session.rage_active = True
        session.rage_thread = threading.Thread(
            target=self._rage_joystick_loop,
            args=(session,),
            name="go2-rage-joystick",
            daemon=True,
        )
        session.rage_thread.start()

    def _stop_rage_joystick(self, session: _Session) -> None:
        """Stop the publisher thread and release the DDS writer.

        Closes ChannelPublisher explicitly to avoid leaking the DDS writer
        across repeated set_rage_mode(True/False) cycles.
        """
        session.rage_active = False
        if session.rage_stop is not None:
            session.rage_stop.set()
        if session.rage_thread is not None:
            session.rage_thread.join(timeout=1.0)
            session.rage_thread = None
        session.rage_stop = None
        if session.rage_pub is not None:
            try:
                session.rage_pub.Close()
            except (OSError, RuntimeError) as e:
                logger.warning(f"[Go2] Rage publisher Close raised: {e}")
            session.rage_pub = None

    def _rage_joystick_loop(self, session: _Session) -> None:
        """Publish the latest rage_cmd as a WirelessController_ message.

        Runs at _RAGE_PUBLISH_HZ. On each tick, reads session.rage_cmd,
        normalizes to stick axes via the envelope constants, and writes
        a WirelessController_ message. Exits when rage_stop is set or
        the session's publisher is torn down.
        """
        period = 1.0 / self._RAGE_PUBLISH_HZ
        msg = unitree_go_msg_dds__WirelessController_()
        msg.keys = 0
        msg.ry = 0.0

        while session.rage_stop is not None and not session.rage_stop.wait(period):
            pub = session.rage_pub
            if pub is None:
                return
            vx, vy, wz = session.rage_cmd

            ly = _clip(vx / self._RAGE_UP_VX, -1.0, 1.0) * self._RAGE_LY_SIGN
            lx = _clip(vy / self._RAGE_UP_VY, -1.0, 1.0) * self._RAGE_LX_SIGN
            rx = _clip(wz / self._RAGE_UP_VYAW, -1.0, 1.0) * self._RAGE_RX_SIGN

            msg.lx = float(lx)
            msg.ly = float(ly)
            msg.rx = float(rx)

            try:
                pub.Write(msg)
            except (OSError, RuntimeError) as e:
                logger.warning(f"[Go2] Rage joystick publish raised: {e}")
                return

    def _call_sport_api(self, api_id: int, payload: dict[str, Any] | None = None) -> bool:
        """Generic escape hatch for undocumented mcf sport API IDs.

        SportClient's internal dispatcher rejects unregistered api_ids
        with code 3103 (RPC_ERR_CLIENT_API_NOT_REG) before any message
        leaves the process — the public SDK only registers its named
        methods in __init__. We call _RegistApi() first (idempotent dict
        set) so undocumented IDs like RAGEMODE reach the robot.

        Uses leading-underscore SDK methods (_RegistApi, _Call) — these
        are not part of the public SDK contract. Verified working against
        unitree-sdk2py-dimos>=1.0.2; retest if the SDK is upgraded.

        Returns True on RPC code 0. On failure, logs code + response.
        """
        session = self._get_session()
        body = json.dumps(payload or {})
        with session.lock:
            session.client._RegistApi(api_id, 0)
            code, data = session.client._Call(api_id, body)

        if code != 0:
            logger.warning(f"[Go2] _Call({api_id}, {body}) -> code={code} data={data!r}")
            return False
        return True

    def _get_session(self) -> _Session:
        """Return active session or raise RuntimeError if disconnected.

        Note: callers using the returned session.lock must NEVER then
        try to acquire self._session_lock — see the lock-ordering rule
        in the class docstring.
        """
        session = self._session
        if session is None:
            raise RuntimeError("Go2 not connected")
        return session

    def _initialize_locomotion(self) -> bool:
        """StandUp → 3s settle → FreeWalk → 2s settle → SpeedLevel.

        Called from connect() and from write_enable(True) if locomotion
        was not yet ready. Assumes a sport mode is already active.
        """
        session = self._get_session()

        if not self.check_mode():
            logger.error("[Go2] No sport mode active")
            return False

        logger.info("[Go2] Standing up...")
        with session.lock:
            ret = session.client.StandUp()
        if ret != 0:
            logger.error(f"[Go2] StandUp failed with code {ret}")
            return False
        time.sleep(3)

        logger.info("[Go2] Activating FreeWalk...")
        with session.lock:
            ret = session.client.FreeWalk()
        if ret != 0:
            logger.error(f"[Go2] FreeWalk failed with code {ret}")
            return False
        time.sleep(2)

        with session.lock:
            sl_ret = session.client.SpeedLevel(self._speed_level)
        if sl_ret == 0:
            logger.info(f"[Go2] SpeedLevel({self._speed_level}) applied")
        else:
            logger.warning(f"[Go2] SpeedLevel({self._speed_level}) returned {sl_ret}")

        session.locomotion_ready = True
        logger.info("[Go2] Locomotion ready")
        return True

    def _send_velocity(self, vx: float, vy: float, wz: float) -> bool:
        session = self._get_session()
        with session.lock:
            ret = session.client.Move(vx, vy, wz)
        if ret != 0:
            logger.warning(f"[Go2] Move() returned code {ret}")
            return False
        return True


def register(registry: TwistBaseAdapterRegistry) -> None:
    registry.register("unitree_go2", UnitreeGo2TwistAdapter)


__all__ = ["UnitreeGo2TwistAdapter"]
