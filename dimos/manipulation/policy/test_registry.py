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

"""Unit tests for the policy backend registry."""

from __future__ import annotations

import pytest

# Importing the package registers the built-in entries.
import dimos.manipulation.policy  # noqa: F401
from dimos.manipulation.policy.command import JointPositionCommand, PolicyCommand
from dimos.manipulation.policy.observation import PolicyObservation
from dimos.manipulation.policy.registry import (
    available_backends,
    create_backend,
    is_registered,
    register_backend,
)


def test_builtin_backends_are_registered():
    backends = available_backends()
    assert "test" in backends
    assert "lerobot" in backends


def test_create_known_backend_returns_instance():
    backend = create_backend("test", joint_names=["a", "b"], amplitude=0.5, frequency=1.0)
    backend.initialize()
    cmd = backend.select_action(PolicyObservation())
    assert isinstance(cmd, JointPositionCommand)
    assert cmd.joint_names == ("a", "b")


def test_unknown_backend_raises_keyerror_listing_options():
    with pytest.raises(KeyError) as excinfo:
        create_backend("does-not-exist")
    msg = str(excinfo.value)
    assert "does-not-exist" in msg
    assert "test" in msg  # error message should list available backends
    assert "lerobot" in msg


def test_register_backend_overrides_and_isolates():
    sentinel: list[str] = []

    def _factory(**_: object) -> object:
        sentinel.append("called")
        return _DummyBackend()

    register_backend("__test_only__", _factory)  # type: ignore[arg-type]
    assert is_registered("__test_only__")
    backend = create_backend("__test_only__")
    assert sentinel == ["called"]
    # Run through the protocol smoke
    backend.initialize()
    cmd = backend.select_action(PolicyObservation())
    assert isinstance(cmd, PolicyCommand)
    backend.reset()
    backend.close()


class _DummyBackend:
    def initialize(self) -> None:
        pass

    def select_action(self, observation: PolicyObservation) -> PolicyCommand:
        del observation
        return JointPositionCommand(joint_names=("x",), positions=(0.0,))

    def reset(self) -> None:
        pass

    def close(self) -> None:
        pass
