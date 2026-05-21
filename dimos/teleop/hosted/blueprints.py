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

"""Hosted teleop blueprints using WebRTCTransport (no intermediate module).

These blueprints use the :class:`BrokerProvider` to receive operator commands
directly via WebRTC DataChannels over the Cloudflare Realtime SFU. The
broker (``dimensional-teleop``) handles session lifecycle and operator
management.

No ``HostedTeleopModule`` — the transport layer handles message delivery
just like LCM or SHM. For stateless keyboard teleop, no extra module is
needed at all. For VR teleop with engagement gating, a thin
``TeleopScalerModule`` handles the speed scaling logic.

Usage:
    dimos run teleop-hosted-go2
    dimos run teleop-hosted-go2-scaled

Blueprints are constructed via factory functions to avoid creating
network connections at import time.

Environment:
    TELEOP_BROKER_URL   — Broker URL (default: https://teleop.dimensionalos.com)
    TELEOP_API_KEY      — Robot API key (dtk_live_*)
    TELEOP_ROBOT_ID     — Robot identifier
    TELEOP_ROBOT_NAME   — Human-readable robot name (optional)
"""

from __future__ import annotations

from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import WebRTCTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# ─── TeleopScalerModule (thin module for speed scaling) ──────────────


class TeleopScalerConfig(ModuleConfig):
    """Speed scaling config for hosted keyboard teleop."""

    linear_speed: float = 0.5
    angular_speed: float = 0.8


class TeleopScalerModule(Module):
    """Thin module that scales incoming TwistStamped by configured speeds.

    The operator's keyboard sends normalized commands in [-1, 1]
    (Shift = 2x, Ctrl = 0.5x). This module multiplies by linear_speed
    and angular_speed to produce final m/s and rad/s.

    This is the ONLY logic that justified HostedTeleopModule for keyboard
    mode. Everything else (PeerConnection, heartbeat, broker, decode) is
    now handled by the transport layer.
    """

    config: TeleopScalerConfig

    cmd_vel_in: In[TwistStamped]
    cmd_vel: Out[Twist]
    cmd_vel_stamped: Out[TwistStamped]

    @rpc
    def start(self) -> None:
        super().start()
        self.cmd_vel_in.subscribe(self._on_twist)

    def _on_twist(self, msg: TwistStamped) -> None:
        ls = self.config.linear_speed
        as_ = self.config.angular_speed
        linear = Vector3(msg.linear.x * ls, msg.linear.y * ls, msg.linear.z * ls)
        angular = Vector3(msg.angular.x * as_, msg.angular.y * as_, msg.angular.z * as_)
        self.cmd_vel.publish(Twist(linear=linear, angular=angular))
        self.cmd_vel_stamped.publish(
            TwistStamped(ts=msg.ts, frame_id=msg.frame_id, linear=linear, angular=angular)
        )


# ─── Blueprint factory functions ─────────────────────────────────────


def make_teleop_hosted_go2(
    broker_url: str | None = None,
    api_key: str | None = None,
    robot_id: str | None = None,
    robot_name: str | None = None,
) -> Any:
    """Create a hosted Go2 keyboard teleop blueprint (pure transport, no module).

    Operator sends TwistStamped from browser; the transport decodes via LCM
    fingerprint filtering and delivers directly to Go2's cmd_vel stream.

    This is the simplest possible hosted teleop — zero intermediate modules.
    """
    from dimos.protocol.pubsub.impl.webrtc_providers.broker import BrokerProvider
    from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic

    provider = BrokerProvider(
        broker_url=broker_url,
        api_key=api_key,
        robot_id=robot_id,
        robot_name=robot_name,
    )

    return unitree_go2_basic.transports(
        {
            ("cmd_vel", Twist): WebRTCTransport(
                "cmd_unreliable",
                msg_type=TwistStamped,
                provider=provider,
            ),
        }
    )


def make_teleop_hosted_go2_scaled(
    broker_url: str | None = None,
    api_key: str | None = None,
    robot_id: str | None = None,
    robot_name: str | None = None,
    linear_speed: float = 0.5,
    angular_speed: float = 0.8,
) -> Any:
    """Create a hosted Go2 keyboard teleop blueprint with speed scaling.

    WebRTCTransport → TeleopScalerModule → unitree_go2_basic
    """
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.protocol.pubsub.impl.webrtc_providers.broker import BrokerProvider
    from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic

    provider = BrokerProvider(
        broker_url=broker_url,
        api_key=api_key,
        robot_id=robot_id,
        robot_name=robot_name,
    )

    return autoconnect(
        TeleopScalerModule.blueprint(
            linear_speed=linear_speed,
            angular_speed=angular_speed,
        ),
        unitree_go2_basic,
    ).transports(
        {
            ("cmd_vel_in", TwistStamped): WebRTCTransport(
                "cmd_unreliable",
                msg_type=TwistStamped,
                provider=provider,
            ),
        }
    )


__all__ = [
    "TeleopScalerConfig",
    "TeleopScalerModule",
    "make_teleop_hosted_go2",
    "make_teleop_hosted_go2_scaled",
]
