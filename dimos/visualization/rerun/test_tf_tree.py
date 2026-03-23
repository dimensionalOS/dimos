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

"""Tests for TF tree rendering in RerunBridgeModule."""

from __future__ import annotations

import builtins
import sys
from types import ModuleType
from typing import Any
from unittest.mock import MagicMock, patch

# Stub out heavy/unavailable dependencies before importing bridge.
_real_import = builtins.__import__


def _mock_import(
    name: str, globals: Any = None, locals: Any = None, fromlist: Any = (), level: int = 0
) -> Any:
    try:
        return _real_import(name, globals, locals, fromlist, level)
    except (ModuleNotFoundError, ImportError):
        if "lazy_loader" in name:
            m = ModuleType(name)
            m.attach = lambda *a, **kw: (lambda n: None, lambda: [], [])  # type: ignore[attr-defined]
            sys.modules[name] = m
            return m
        mock_mod: Any = MagicMock()
        sys.modules[name] = mock_mod
        return mock_mod  # type: ignore[return-value]


builtins.__import__ = _mock_import  # type: ignore[assignment]

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs import TFMessage
from dimos.protocol.tf import MultiTBuffer
from dimos.visualization.rerun.bridge import Config, RerunBridgeModule

# Restore normal import after our modules are loaded.
builtins.__import__ = _real_import


def _make_transform(
    parent: str,
    child: str,
    tx: float = 0.0,
    ty: float = 0.0,
    tz: float = 0.0,
) -> Transform:
    return Transform(
        translation=Vector3(tx, ty, tz),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id=parent,
        child_frame_id=child,
    )


def _make_bridge(*, tf_enabled: bool = True) -> RerunBridgeModule:
    """Create a RerunBridgeModule without running Module lifecycle."""
    bridge = object.__new__(RerunBridgeModule)
    bridge.config = Config(pubsubs=[], tf_enabled=tf_enabled, entity_prefix="world")
    bridge._last_log = {}
    bridge._last_tf_render_time = 0.0
    if tf_enabled:
        bridge._tf = MultiTBuffer()  # type: ignore[assignment]  # inject plain buffer for testing (production uses LCMTF which extends both)
    else:
        bridge._tf = None
    return bridge


class TestRenderTfTree:
    """Tests for _render_tf_tree DFS walk and entity path construction."""

    @patch("rerun.log")
    @patch("rerun.Transform3D")
    @patch("rerun.Quaternion")
    def test_simple_chain(
        self, mock_quat: MagicMock, mock_t3d: MagicMock, mock_log: MagicMock
    ) -> None:
        """A→B→C chain produces world/A/B and world/A/B/C entity paths."""
        bridge = _make_bridge()
        bridge.tf.receive_transform(
            _make_transform("odom", "base_link", tx=1.0),
            _make_transform("base_link", "camera", tz=0.5),
        )

        bridge._render_tf_tree()

        logged_paths = [c.args[0] for c in mock_log.call_args_list]
        assert "world/odom/base_link" in logged_paths
        assert "world/odom/base_link/camera" in logged_paths
        assert len(logged_paths) == 2

    @patch("rerun.log")
    @patch("rerun.Transform3D")
    @patch("rerun.Quaternion")
    def test_multiple_roots(
        self, mock_quat: MagicMock, mock_t3d: MagicMock, mock_log: MagicMock
    ) -> None:
        """Two disjoint trees produce separate root paths."""
        bridge = _make_bridge()
        bridge.tf.receive_transform(
            _make_transform("odom", "base_link"),
            _make_transform("map", "marker"),
        )

        bridge._render_tf_tree()

        logged_paths = {c.args[0] for c in mock_log.call_args_list}
        assert "world/odom/base_link" in logged_paths
        assert "world/map/marker" in logged_paths
        assert len(logged_paths) == 2

    @patch("rerun.log")
    def test_tf_disabled_falls_through(self, mock_log: MagicMock) -> None:
        """When tf_enabled=False, TFMessage is NOT intercepted by the TF path."""
        bridge = _make_bridge(tf_enabled=False)

        msg = TFMessage(_make_transform("odom", "base_link"))

        # _on_message should NOT enter the TF intercept branch.
        # It will try the visual override path which needs _visual_override_for_entity_path.
        # We patch that to verify the fallthrough.
        with patch.object(bridge, "_visual_override_for_entity_path") as mock_vo:
            mock_vo.return_value = lambda m: None  # suppress further processing
            bridge._on_message(msg, "/tf")

        # visual override path was reached (not short-circuited by TF intercept)
        mock_vo.assert_called_once()

    @patch("rerun.log")
    @patch("rerun.Transform3D")
    @patch("rerun.Quaternion")
    def test_incremental_update(
        self, mock_quat: MagicMock, mock_t3d: MagicMock, mock_log: MagicMock
    ) -> None:
        """Adding a new child after initial render extends the tree."""
        bridge = _make_bridge()
        bridge.tf.receive_transform(
            _make_transform("odom", "base_link"),
        )
        bridge._render_tf_tree()
        assert len(mock_log.call_args_list) == 1

        mock_log.reset_mock()
        bridge.tf.receive_transform(
            _make_transform("base_link", "lidar"),
        )
        bridge._render_tf_tree()

        logged_paths = {c.args[0] for c in mock_log.call_args_list}
        assert "world/odom/base_link" in logged_paths
        assert "world/odom/base_link/lidar" in logged_paths

    @patch("rerun.log")
    @patch("rerun.Transform3D")
    @patch("rerun.Quaternion")
    def test_cycle_protection(
        self, mock_quat: MagicMock, mock_t3d: MagicMock, mock_log: MagicMock
    ) -> None:
        """A cycle in the TF graph does not cause infinite recursion."""
        bridge = _make_bridge()
        # Create A→B→C→A cycle
        bridge.tf.receive_transform(
            _make_transform("A", "B"),
            _make_transform("B", "C"),
            _make_transform("C", "A"),
        )

        # Should not raise or hang
        bridge._render_tf_tree()

        logged_paths = [c.args[0] for c in mock_log.call_args_list]
        # A pure cycle has no root (every frame is a child of some other frame),
        # so roots list is empty and no DFS starts → zero logs.
        assert len(logged_paths) == 0  # pure cycle → no root → no DFS starts
