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

"""Explicit Dreamlabs observation/inference RPCs. This module never executes actions."""

from __future__ import annotations

from copy import deepcopy
from threading import RLock
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.typing import NDArray

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.imitation.policy.dreamlabs.config import DreamlabsConfig
from dimos.imitation.policy.dreamlabs.conversion import (
    action_trajectory,
    encode_image,
    policy_config,
    state_vector,
)
from dimos.imitation.policy.dreamlabs.kinematics import DreamlabsKinematics
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory

if TYPE_CHECKING:
    from dream import DreamClient


def create_client() -> DreamClient:
    # Optional dependency: importing and starting this module do not need the SDK.
    import dream

    return dream.DreamClient()


class DreamlabsModule(Module):
    """Accept explicitly supplied camera ticks and return unexecuted action previews.

    SDK requests/reset/close are serialized. Observation buffering can continue during
    inference. There is no background inference, live
    camera synchronization, or coordinator connection.
    """

    config: DreamlabsConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._request_lock = RLock()
        self._client: DreamClient | None = None
        self._fk: DreamlabsKinematics | None = None
        self._latest: NDArray[np.float64] | None = None
        self._last_action_response: dict[str, Any] | None = None

    @rpc
    def set_observation(
        self,
        overhead: Image,
        left_wrist: Image,
        right_wrist: Image,
        state: JointState,
        timestamp: float,
    ) -> None:
        """Give the SDK one pre-paired tick; timestamp is monotonic capture seconds.

        This buffers observations locally. No inference request is sent until an
        explicit get_action(). The caller supplies the camera/state pairing.
        """
        with self._lock:
            vector = state_vector(state)
            frames = {
                "video.overhead": encode_image(overhead),
                "video.wrist_l": encode_image(left_wrist),
                "video.wrist_r": encode_image(right_wrist),
            }
            if self._fk is None:
                self._fk = DreamlabsKinematics()
            if self._client is None:
                client = create_client()
                try:
                    client.policy(policy_config(self.config.checkpoint, self.config.timeout_ms))
                except Exception:
                    client.close()
                    raise
                self._client = client
            self._client.set_observation(
                frames, self._fk.observation(vector, self.config.prompt), timestamp=timestamp
            )
            self._latest = vector

    @rpc
    def get_action(self) -> JointTrajectory:
        """Make one potentially billed API request and return an unexecuted preview.

        Requires prior observation input and approval of the image/state upload.
        The result has protocol checks, not physical rollout safety validation.
        """
        with self._request_lock:
            with self._lock:
                if self._client is None or self._latest is None:
                    raise RuntimeError("set_observation must precede get_action")
                client = self._client
            response = client.get_action()
            with self._lock:
                self._last_action_response = deepcopy(response)
                assert self._latest is not None
                assert self._fk is not None
                return action_trajectory(response, self._latest.copy(), self._fk.joint_bounds)

    @rpc
    def get_last_action_response(self) -> dict[str, Any] | None:
        """Return the last SDK action dict before conversion, including rejected values.

        Retained in memory across episode resets for diagnostics; no request payload
        or credentials are captured. This never requests inference or executes actions.
        """
        with self._lock:
            return deepcopy(self._last_action_response)

    @rpc
    def reset_episode(self) -> None:
        """Clear the SDK's frame history before another trial."""
        with self._request_lock, self._lock:
            if self._client is not None:
                self._client.reset_episode()
            self._latest = None

    @rpc
    def stop(self) -> None:
        try:
            with self._request_lock, self._lock:
                if self._client is not None:
                    self._client.close()
                    self._client = None
                self._latest = None
        finally:
            super().stop()
