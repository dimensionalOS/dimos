from typing import Any

class _MavlinkMessage:
    base_mode: int
    command: int
    result: int

class _MavSender:
    def command_long_send(self, *args: Any, **kwargs: Any) -> None: ...
    def set_mode_send(self, *args: Any, **kwargs: Any) -> None: ...
    def set_position_target_local_ned_send(self, *args: Any, **kwargs: Any) -> None: ...

class MavlinkConnection:
    target_system: int
    target_component: int
    mav: _MavSender
    def wait_heartbeat(self, timeout: float | None = ...) -> _MavlinkMessage: ...
    def recv_match(
        self,
        type: str | list[str] | None = ...,
        blocking: bool = ...,
        timeout: float | None = ...,
    ) -> _MavlinkMessage | None: ...
    def close(self) -> None: ...

def mavlink_connection(
    device: str,
    baud: int = ...,
    source_system: int = ...,
    source_component: int = ...,
    **kwargs: Any,
) -> MavlinkConnection: ...

class _MavlinkConstants:
    MAV_CMD_COMPONENT_ARM_DISARM: int
    MAV_CMD_DO_SET_MODE: int
    MAV_CMD_NAV_LAND: int
    MAV_CMD_NAV_TAKEOFF: int
    MAV_FRAME_BODY_NED: int
    MAV_FRAME_LOCAL_NED: int
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: int
    MAV_MODE_FLAG_SAFETY_ARMED: int
    MAV_RESULT_ACCEPTED: int

mavlink: _MavlinkConstants
