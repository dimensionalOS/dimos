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

"""One description's joints on tf, posed from the joint state as it arrives.

Hand `JointTf` the model that says what the joints are, the stream the joint state comes
in on, and the tf stream to publish on; it does the rest. Nothing about it is tied to the
module that owns it, so a world of three robots is three of these -- each robot poses the
joints of its own description, under its own frame names, from its own joint stream.

A description with no joint-state source at all still has fixed edges worth knowing, so
the loop publishes those on their own until a state shows up.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from threading import Event, Lock, Thread
import time
from typing import TYPE_CHECKING, Any, Protocol

import numpy as np

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.robot.assets.model import JointDescription

logger = setup_logger()


# Their pose comes from tf (odometry, a planar base), so they are read from tf, never published to it.
MULTI_DOF_JOINT_TYPES = frozenset({"floating", "planar"})

# Less than this much movement on an edge is not movement: a resting joint's noise is not news.
_MOVED = 1e-6


class HasJoints(Protocol):
    """A description that can say what its joints are: a RobotModel, a UsdModel, a loaded one."""

    @property
    def joints(self) -> tuple[JointDescription, ...]: ...


class TfSink(Protocol):
    """Somewhere to put transforms -- a module's `tfbuffer`, or any TF over a tf stream."""

    def publish(self, *transforms: Transform) -> None: ...


class JointStateSource(Protocol):
    """A stream joint states arrive on: `In[JointState]`, `IO[JointState]`, a transport."""

    def subscribe(self, cb: Callable[[JointState], Any]) -> Callable[[], None]: ...


def _joints_of(model: Any) -> dict[str, JointDescription]:
    """The model's joints by name, loading it first if it is still a lazy handle."""
    described: HasJoints = model if hasattr(model, "joints") else model.load()
    return {joint.name: joint for joint in described.joints}


class JointTf:
    """Puts one description's joints on tf, and keeps them there.

    `start()` subscribes to the joint stream and begins publishing; `stop()` undoes both.
    The streams can be given here or at `start()`, since a module's tf buffer and its ports
    are only alive once it has a transport.

    Rates: a moved edge goes out at most `joint_tf_hz`, and every edge is repeated at
    `unchanged_tf_hz` whether it moved or not -- a tf lookup needs a sample inside its time
    tolerance, so keep 1 / unchanged_tf_hz under twice the tightest tolerance of the readers.
    """

    def __init__(
        self,
        model: Any,
        joint_state: JointStateSource | None = None,
        tf: TfSink | None = None,
        *,
        frame: Callable[[str], str] | None = None,
        joint_tf_hz: float = 30.0,
        unchanged_tf_hz: float = 4.0,
    ) -> None:
        if joint_tf_hz <= 0.0 or unchanged_tf_hz <= 0.0:
            raise ValueError(
                f"joint_tf_hz and unchanged_tf_hz are rates, so both have to be above zero; "
                f"got {joint_tf_hz} and {unchanged_tf_hz}"
            )
        self._joints = _joints_of(model)
        self._frame = frame or (lambda name: name)
        self._joint_period = 1.0 / joint_tf_hz
        self._keepalive = 1.0 / unchanged_tf_hz
        self._joint_state = joint_state
        self._tf = tf
        self._lock = Lock()
        # Per edge: the last transform sent (translation + rotation) and when.
        self._sent_edges: dict[tuple[str, str], tuple[tuple[float, ...], float]] = {}
        self._next_joint_t = 0.0
        self._latest: JointState | None = None
        self._woke = Event()
        self._stop = Event()
        self._thread: Thread | None = None
        self._unsubscribe: Callable[[], None] | None = None

    # ----------------------------------------------------------------- wiring

    def start(self, joint_state: JointStateSource | None = None, tf: TfSink | None = None) -> None:
        """Subscribe to the joint stream and start publishing. Idempotent."""
        self._joint_state = joint_state if joint_state is not None else self._joint_state
        self._tf = tf if tf is not None else self._tf
        if self._tf is None:
            raise ValueError("JointTf has nowhere to publish: give it a tf buffer or stream")
        if self._thread is not None:
            return
        if self._joint_state is None and not self.static_transforms():
            # No stream to hear from and no edge that stands on its own: there is nothing a loop
            # here could ever publish, so it does not get one.
            return
        self._stop.clear()
        if self._joint_state is not None:
            self._unsubscribe = self._joint_state.subscribe(self._received)
        self._thread = Thread(target=self._run, daemon=True, name="JointTf")
        self._thread.start()

    def stop(self) -> None:
        """Stop publishing and let go of the joint stream. Idempotent."""
        self._stop.set()
        self._woke.set()
        if self._unsubscribe is not None:
            self._unsubscribe()
            self._unsubscribe = None
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=2.0)

    def _received(self, state: JointState) -> None:
        """Keep only the newest state: a pose that was already overtaken is not worth sending."""
        with self._lock:
            self._latest = state
        self._woke.set()

    # ------------------------------------------------------------------- loop

    def _run(self) -> None:
        while not self._stop.is_set():
            state = self._take()
            if state is not None:
                self._next_joint_t = time.monotonic() + self._joint_period
                self.publish(self.joint_transforms(state))
                self._wait_until(self._next_joint_t, for_a_state=False)
                continue
            self.publish(self.static_transforms())
            self._wait_until(time.monotonic() + self._keepalive, for_a_state=True)

    def _take(self) -> JointState | None:
        with self._lock:
            state, self._latest = self._latest, None
        return state

    def _wait_until(self, deadline: float, *, for_a_state: bool) -> None:
        """Sleep to `deadline`, but let a joint state cut the wait short once its rate allows."""
        while not self._stop.is_set():
            left = deadline - time.monotonic()
            if left <= 0.0:
                return
            if not self._woke.wait(left):
                return
            self._woke.clear()
            if for_a_state and time.monotonic() >= self._next_joint_t:
                return

    # --------------------------------------------------------------- the work

    def add_joint_state(self, state: JointState, tf: TfSink | None = None) -> None:
        """Pose the joints from `state` now, for a caller that already has one in hand."""
        if time.monotonic() < self._next_joint_t:
            return
        self._next_joint_t = time.monotonic() + self._joint_period
        self.publish(self.joint_transforms(state), tf)

    def publish(self, transforms: Iterable[Transform], tf: TfSink | None = None) -> None:
        """Send the edges that moved since they last went out, and the ones now stale."""
        sink = tf if tf is not None else self._tf
        if sink is None:
            raise ValueError("JointTf has nowhere to publish: give it a tf buffer or stream")
        due = self._edges_to_send(transforms)
        if due:
            sink.publish(*due)

    def _edges_to_send(self, transforms: Iterable[Transform]) -> list[Transform]:
        """The edges that moved since they were last sent, and those not sent for one keepalive period."""
        now = time.monotonic()
        due = []
        with self._lock:
            for transform in transforms:
                key = (transform.frame_id, transform.child_frame_id)
                value = (*transform.translation.to_tuple(), *transform.rotation.to_tuple())
                sent = self._sent_edges.get(key)
                if (
                    sent is None
                    or now - sent[1] >= self._keepalive
                    or not np.allclose(value, sent[0], atol=_MOVED)
                ):
                    due.append(transform)
                    self._sent_edges[key] = (value, now)
        return due

    def static_transforms(self) -> list[Transform]:
        """The edges that never move, so they need no joint state to be known."""
        return [
            Transform.from_matrix(
                joint.parent_from_child(0.0),
                frame_id=self._frame(joint.parent_link),
                child_frame_id=self._frame(joint.child_link),
                ts=time.time(),
            )
            for joint in self._joints.values()
            if joint.type == "fixed"
        ]

    def joint_transforms(self, state: JointState) -> list[Transform]:
        """One parent -> child transform per joint; joints the state leaves out stay at zero."""
        positions = dict(zip(state.name, state.position, strict=False))
        for joint in self._joints.values():
            if joint.mimic is not None and joint.mimic[0] in positions:
                leader, multiplier, offset = joint.mimic
                positions[joint.name] = multiplier * positions[leader] + offset
        # Fixed joints are republished with the moving ones: a chained lookup is only as fresh as
        # its stalest edge.
        return [
            Transform.from_matrix(
                joint.parent_from_child(positions.get(joint.name, 0.0)),
                frame_id=self._frame(joint.parent_link),
                child_frame_id=self._frame(joint.child_link),
                ts=state.ts,
            )
            for joint in self._joints.values()
            if joint.type not in MULTI_DOF_JOINT_TYPES
        ]

    @property
    def joints(self) -> dict[str, JointDescription]:
        """The description's joints by name, which is what it poses."""
        return self._joints

    @property
    def joint_names(self) -> set[str]:
        """The names it poses joints by, which is what a joint state has to use."""
        return set(self._joints)
