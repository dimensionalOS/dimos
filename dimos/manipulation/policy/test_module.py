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

"""Unit tests for `PolicyModule`.

These tests exercise observation assembly, command validation, the
joint-state publish path, and the teleop-takeover behavior. We drive the
internal `_on_*` handlers and `_tick_once()` directly rather than going
through the LCM-backed transport so the tests stay fast and hermetic.
"""

from __future__ import annotations

from collections.abc import Sequence
import threading
import time

import numpy as np
import pytest

from dimos.manipulation.policy import (
    JointPositionCommand,
    NoOpCommand,
    PolicyModule,
    PolicyModuleConfig,
    PolicyObservation,
    register_backend,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.quest.quest_types import Buttons


def _img(seed: int = 0) -> Image:
    arr = np.full((4, 4, 3), seed, dtype=np.uint8)
    return Image(data=arr, format=ImageFormat.RGB)


class _RecordingBackend:
    """Test backend that records every call and emits a fixed command."""

    def __init__(
        self,
        joint_names: Sequence[str],
        positions: Sequence[float] | None = None,
    ) -> None:
        self.joint_names = tuple(joint_names)
        self._positions = tuple(positions or [0.0] * len(joint_names))
        self.observations: list[PolicyObservation] = []
        self.init_calls = 0
        self.reset_calls = 0
        self.close_calls = 0
        self._next_command: JointPositionCommand | NoOpCommand | None = None

    def initialize(self) -> None:
        self.init_calls += 1

    def select_action(self, observation: PolicyObservation) -> JointPositionCommand | NoOpCommand:
        self.observations.append(observation)
        if self._next_command is not None:
            cmd = self._next_command
            return cmd
        return JointPositionCommand(joint_names=self.joint_names, positions=self._positions)

    def queue(self, command: JointPositionCommand | NoOpCommand) -> None:
        self._next_command = command

    def reset(self) -> None:
        self.reset_calls += 1
        self._next_command = None  # mirror "drop buffered chunk"

    def close(self) -> None:
        self.close_calls += 1


@pytest.fixture
def node_factory(request):
    """Create PolicyModule + injected backend pairs and clean each up on teardown."""
    created: list[PolicyModule] = []

    def _make(*, backend_name: str, **config_overrides) -> tuple[PolicyModule, _RecordingBackend]:
        backend = _RecordingBackend(
            joint_names=config_overrides.pop("backend_joint_names", ("j1", "j2")),
            positions=config_overrides.pop("backend_positions", (0.1, 0.2)),
        )
        register_backend(backend_name, lambda **_: backend)

        cfg_kwargs = dict(
            backend=backend_name,
            policy_rate=200.0,
            joint_names=["j1", "j2"],
            camera_key="main",
            # Default the engage-button list off so existing tests don't
            # need to supply a Buttons message before they can publish.
            # Tests of the buttons-gating behavior override this.
            teleop_engage_buttons=config_overrides.pop("teleop_engage_buttons", []),
        )
        cfg_kwargs.update(config_overrides)

        node = PolicyModule(**cfg_kwargs)
        node._backend = backend
        # The rollout gate defaults to False on a fresh node; tests for
        # the gate flip it explicitly via the public RPCs. All other tests
        # opt in here so they exercise the publish path directly.
        node._rollout_enabled = True
        created.append(node)
        return node, backend

    yield _make

    for n in created:
        try:
            n._close_module()
        except Exception:
            pass


# ── 6.3 observation assembly ──────────────────────────────────────────────


def test_observation_assembly_with_single_camera_input(node_factory):
    node, _ = node_factory(
        backend_name="__test_assembly__",
        camera_key="main",
    )
    node._on_image(_img(seed=1))
    node._on_joint_state(JointState(name=["j1", "j2"], position=[0.1, 0.2]))
    node._on_task_description("pick up the cube")

    obs = node.assemble_observation()
    assert list(obs.images.keys()) == ["main"]
    assert obs.joint_state is not None
    assert obs.joint_state.name == ["j1", "j2"]
    assert obs.task == "pick up the cube"


def test_observation_assembly_omits_camera_until_first_frame(node_factory):
    node, _ = node_factory(backend_name="__test_no_image__")
    node._on_joint_state(JointState(name=["j1", "j2"], position=[0.0, 0.0]))

    obs = node.assemble_observation()
    assert obs.images == {}
    assert obs.joint_state is not None


def test_default_task_used_when_no_task_input_arrived(node_factory):
    node, _ = node_factory(
        backend_name="__test_default_task__",
        default_task="open the drawer",
    )
    obs = node.assemble_observation()
    assert obs.task == "open the drawer"


# ── 6.4 command validation + JointState.position publishing ───────────────


def test_publishes_joint_state_with_matching_positions(node_factory):
    node, backend = node_factory(
        backend_name="__test_publish__",
        backend_positions=(0.4, 0.5),
    )

    published: list[JointState] = []
    node.joint_command.subscribe(published.append)

    node._on_joint_state(JointState(name=["j1", "j2"], position=[0.0, 0.0]))
    cmd = node._tick_once()
    assert isinstance(cmd, JointPositionCommand)

    assert len(published) == 1
    msg = published[0]
    assert msg.name == ["j1", "j2"]
    assert msg.position == [0.4, 0.5]


def test_rejects_command_with_unmatched_joint_names(node_factory):
    node, backend = node_factory(backend_name="__test_unmatched__")
    backend.joint_names = ("foreign_j",)
    backend._positions = (1.0,)

    published: list[JointState] = []
    node.joint_command.subscribe(published.append)

    node._on_joint_state(JointState(name=["j1"], position=[0.0]))
    node._tick_once()
    assert published == []  # rejected


def test_rejects_command_when_command_mode_disabled(node_factory):
    node, _ = node_factory(
        backend_name="__test_disabled_mode__",
        enabled_command_modes=[],
    )

    published: list[JointState] = []
    node.joint_command.subscribe(published.append)

    node._on_joint_state(JointState(name=["j1", "j2"], position=[0.0, 0.0]))
    node._tick_once()
    assert published == []


def test_noop_command_does_not_publish(node_factory):
    node, backend = node_factory(backend_name="__test_noop__")
    backend.queue(NoOpCommand(reason="explicit no-op"))

    published: list[JointState] = []
    node.joint_command.subscribe(published.append)

    node._on_joint_state(JointState(name=["j1", "j2"], position=[0.0, 0.0]))
    node._tick_once()
    assert published == []


def test_skips_step_when_no_joint_state_yet(node_factory):
    node, backend = node_factory(backend_name="__test_no_js__")
    cmd = node._tick_once()
    assert cmd is None
    assert backend.observations == []


def test_joint_name_map_is_applied_before_validation(node_factory):
    node, backend = node_factory(
        backend_name="__test_map__",
        joint_name_map={"raw1": "j1", "raw2": "j2"},
    )
    backend.joint_names = ("raw1", "raw2")
    backend._positions = (0.1, 0.2)

    published: list[JointState] = []
    node.joint_command.subscribe(published.append)

    node._on_joint_state(JointState(name=["j1", "j2"], position=[0.0, 0.0]))
    node._tick_once()
    assert len(published) == 1
    assert published[0].name == ["j1", "j2"]


# ── 6.6 teleop takeover ───────────────────────────────────────────────────


def test_engage_button_suspends_publication_and_resets_backend(node_factory):
    node, backend = node_factory(
        backend_name="__test_engage__",
        teleop_engage_buttons=["right_primary"],
    )
    # Mark the first-buttons gate as satisfied so the publish path runs.
    node._first_buttons_received = True

    published: list[JointState] = []
    node.joint_command.subscribe(published.append)

    node._on_joint_state(JointState(name=["j1", "j2"], position=[0.0, 0.0]))
    node._tick_once()
    assert len(published) == 1
    pre_resets = backend.reset_calls

    # Engage event: simulate the right-primary button going high.
    btns = Buttons()
    btns.right_primary = True
    node._on_buttons(btns)
    assert backend.reset_calls == pre_resets + 1
    assert node.is_engaged() is True

    cmd = node._tick_once()
    assert cmd is None
    assert len(published) == 1  # no new publication while engaged


def test_disengage_resumes_publication_with_fresh_select_action(node_factory):
    node, backend = node_factory(
        backend_name="__test_disengage__",
        teleop_engage_buttons=["right_primary"],
    )
    node._first_buttons_received = True

    published: list[JointState] = []
    node.joint_command.subscribe(published.append)

    node._on_joint_state(JointState(name=["j1", "j2"], position=[0.0, 0.0]))

    # Pre-engage: simulate the backend buffering a stale chunk that would
    # be replayed at the next select_action call if `reset()` did not run.
    backend.queue(JointPositionCommand(joint_names=("j1", "j2"), positions=(9.9, 9.9)))

    # Engage event triggers backend.reset() → buffered chunk dropped.
    btns_engaged = Buttons()
    btns_engaged.right_primary = True
    node._on_buttons(btns_engaged)
    assert backend.reset_calls == 1

    # During engagement, no commands are published.
    node._tick_once()
    assert published == []

    # Disengage and run one tick: the next select_action call must return
    # a fresh command computed from the current observation, not the
    # pre-engage buffered (9.9, 9.9) chunk.
    btns_idle = Buttons()
    node._on_buttons(btns_idle)
    assert node.is_engaged() is False

    obs_count_before = len(backend.observations)
    node._tick_once()
    assert len(backend.observations) == obs_count_before + 1
    assert len(published) == 1
    assert published[0].position == [0.1, 0.2]  # default, not the 9.9 stale


def test_engage_then_repeated_engage_only_resets_once_per_edge(node_factory):
    node, backend = node_factory(
        backend_name="__test_edge__",
        teleop_engage_buttons=["right_primary"],
    )

    btns_engaged = Buttons()
    btns_engaged.right_primary = True

    pre = backend.reset_calls
    node._on_buttons(btns_engaged)
    node._on_buttons(btns_engaged)  # held
    node._on_buttons(btns_engaged)  # held
    assert backend.reset_calls == pre + 1


# ── config validation ────────────────────────────────────────────────────


def test_unsupported_command_mode_raises():
    with pytest.raises(Exception, match="enabled_command_modes|joint_position"):
        PolicyModuleConfig(
            backend="test",
            joint_names=["j1"],
            enabled_command_modes=["cartesian"],
        )


# ── 6.1 engage edge during in-flight inference drops the command ──────────


class _SlowBackend:
    """Backend whose `select_action` blocks until a release event is set,
    letting the test deliver a Buttons engage frame mid-flight."""

    def __init__(self, release: threading.Event) -> None:
        self._release = release
        self.reset_calls = 0
        self.observations: list[PolicyObservation] = []

    def initialize(self) -> None: ...

    def reset(self) -> None:
        self.reset_calls += 1

    def close(self) -> None: ...

    def select_action(self, observation: PolicyObservation):
        self.observations.append(observation)
        # Block until the test signals: this is the in-flight window.
        self._release.wait(timeout=2.0)
        return JointPositionCommand(joint_names=("j1", "j2"), positions=(0.4, 0.5))


def test_engage_edge_during_in_flight_select_action_drops_command(node_factory):
    """Race: engage edge fires while `select_action` is still running.

    Inference thread enters `_tick_once`, passes the initial gate, then
    enters a slow `select_action`. While it's blocked, the test thread
    delivers a `right_primary=True` Buttons frame. The second gate
    (post-inference) must drop the command rather than publish it.
    """
    release = threading.Event()
    backend = _SlowBackend(release)
    register_backend("__test_race__", lambda **_: backend)

    node = PolicyModule(
        backend="__test_race__",
        policy_rate=200.0,
        joint_names=["j1", "j2"],
        camera_key="main",
        teleop_engage_buttons=["right_primary"],
    )
    node._backend = backend
    node._rollout_enabled = True
    node._first_buttons_received = True
    node._on_joint_state(JointState(name=["j1", "j2"], position=[0.0, 0.0]))

    published: list[JointState] = []
    node.joint_command.subscribe(published.append)

    # Run _tick_once in a background thread; it will block in select_action.
    tick_done = threading.Event()

    def _run_tick():
        node._tick_once()
        tick_done.set()

    t = threading.Thread(target=_run_tick, daemon=True)
    t.start()

    # Wait until the backend has actually entered select_action.
    deadline = time.monotonic() + 1.0
    while not backend.observations and time.monotonic() < deadline:
        time.sleep(0.01)
    assert backend.observations, "select_action did not start in time"

    # Deliver an engage frame mid-flight; release the backend so tick proceeds.
    btns = Buttons()
    btns.right_primary = True
    node._on_buttons(btns)
    release.set()

    tick_done.wait(timeout=2.0)
    assert tick_done.is_set()
    assert published == [], "policy published a command after engage edge fired mid-inference"
    assert backend.reset_calls >= 1
    try:
        node._close_module()
    except Exception:
        pass


# ── 6.2 mandatory buttons subscription when teleop_engage_buttons is set ─


def test_subscribe_inputs_raises_when_buttons_fails_with_engage_buttons_configured(
    node_factory,
):
    """When `teleop_engage_buttons` is non-empty, a failing `buttons`
    subscription propagates from `_subscribe_inputs()` instead of being
    log-and-continued. We exercise `_subscribe_inputs` directly to avoid
    spinning up module runtime threads (which the broader `start()` path
    would do before the buttons check fires).
    """
    node, _ = node_factory(
        backend_name="__test_buttons_required__",
        teleop_engage_buttons=["right_primary"],
    )

    class _RaisingStream:
        def subscribe(self, _handler):
            raise RuntimeError("simulated transport failure")

    node.buttons = _RaisingStream()  # type: ignore[assignment]

    with pytest.raises(RuntimeError, match="simulated transport failure"):
        node._subscribe_inputs()


def test_subscribe_inputs_tolerates_missing_buttons_when_no_engage_buttons(node_factory):
    """With `teleop_engage_buttons=[]`, a failing `buttons` subscription
    is swallowed as before — the node runs without a teleop-preempt gate.
    """
    node, _ = node_factory(
        backend_name="__test_buttons_optional__",
        teleop_engage_buttons=[],
    )

    class _RaisingStream:
        def subscribe(self, _handler):
            raise RuntimeError("simulated transport failure")

    node.buttons = _RaisingStream()  # type: ignore[assignment]

    # Should not raise.
    node._subscribe_inputs()


# ── 6.3 first-Buttons-message gate ────────────────────────────────────────


def test_no_publication_before_first_buttons_message(node_factory):
    node, backend = node_factory(
        backend_name="__test_first_msg__",
        teleop_engage_buttons=["right_primary"],
    )
    # No Buttons message yet.
    assert node._first_buttons_received is False

    published: list[JointState] = []
    node.joint_command.subscribe(published.append)

    node._on_joint_state(JointState(name=["j1", "j2"], position=[0.0, 0.0]))
    cmd = node._tick_once()
    assert cmd is None, "tick should have been skipped pre-first-Buttons"
    assert published == []

    # Now deliver a Buttons message (button low, just to mark first-received).
    node._on_buttons(Buttons())
    assert node._first_buttons_received is True

    node._tick_once()
    assert len(published) == 1


# ── 6.4 grace-period warning when no buttons arrive ───────────────────────


def test_buttons_grace_period_marks_warning_after_timeout(node_factory):
    """After `buttons_grace_period` elapses with no Buttons message, the
    node flips `_grace_warning_logged=True` on the next tick so the warning
    is emitted at most once. We assert state rather than capture structlog
    output (structlog bypasses pytest's `caplog` by default).
    """
    node, _ = node_factory(
        backend_name="__test_grace__",
        teleop_engage_buttons=["right_primary"],
        buttons_grace_period=0.05,
    )
    node._start_monotonic = time.monotonic() - 1.0
    node._first_buttons_received = False
    assert node._grace_warning_logged is False

    node._on_joint_state(JointState(name=["j1", "j2"], position=[0.0, 0.0]))
    cmd = node._tick_once()
    assert cmd is None
    assert node._grace_warning_logged is True

    # Subsequent ticks must not re-log (the flag stays True).
    node._tick_once()
    assert node._grace_warning_logged is True


def test_buttons_grace_period_does_not_mark_before_timeout(node_factory):
    node, _ = node_factory(
        backend_name="__test_grace_nope__",
        teleop_engage_buttons=["right_primary"],
        buttons_grace_period=5.0,  # generous
    )
    node._start_monotonic = time.monotonic()  # just started
    node._first_buttons_received = False

    node._on_joint_state(JointState(name=["j1", "j2"], position=[0.0, 0.0]))
    node._tick_once()
    assert node._grace_warning_logged is False


# ── 6.5 rollout RPCs ──────────────────────────────────────────────────────


def test_rollout_disabled_by_default_blocks_publication(node_factory):
    node, _ = node_factory(backend_name="__test_default_off__")
    # Override the factory's opt-in: simulate a freshly-constructed node.
    node._rollout_enabled = False
    assert node.is_rollout_active() is False

    published: list[JointState] = []
    node.joint_command.subscribe(published.append)

    node._on_joint_state(JointState(name=["j1", "j2"], position=[0.0, 0.0]))
    cmd = node._tick_once()
    assert cmd is None
    assert published == []


def test_start_rollout_enables_publication_and_resets(node_factory):
    node, backend = node_factory(backend_name="__test_start_rollout__")
    node._rollout_enabled = False
    pre_resets = backend.reset_calls

    node.start_rollout()
    assert node.is_rollout_active() is True
    assert backend.reset_calls == pre_resets + 1

    published: list[JointState] = []
    node.joint_command.subscribe(published.append)
    node._on_joint_state(JointState(name=["j1", "j2"], position=[0.0, 0.0]))
    node._tick_once()
    assert len(published) == 1


def test_stop_rollout_disables_publication_and_resets(node_factory):
    node, backend = node_factory(backend_name="__test_stop_rollout__")
    pre_resets = backend.reset_calls

    node.stop_rollout()
    assert node.is_rollout_active() is False
    assert backend.reset_calls == pre_resets + 1

    published: list[JointState] = []
    node.joint_command.subscribe(published.append)
    node._on_joint_state(JointState(name=["j1", "j2"], position=[0.0, 0.0]))
    node._tick_once()
    assert published == []


def test_rollout_toggle_loop_publishes_then_stops(node_factory):
    node, _ = node_factory(backend_name="__test_toggle_loop__")
    node._rollout_enabled = False

    published: list[JointState] = []
    node.joint_command.subscribe(published.append)
    node._on_joint_state(JointState(name=["j1", "j2"], position=[0.0, 0.0]))

    node._tick_once()
    assert published == []

    node.start_rollout()
    node._tick_once()
    assert len(published) == 1

    node.stop_rollout()
    node._tick_once()
    assert len(published) == 1  # no new publish after stop
