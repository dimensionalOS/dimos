#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

"""
Test for foxglove bridge import and basic functionality
"""

import warnings

import pytest

warnings.filterwarnings("ignore", category=DeprecationWarning, module="websockets.server")
warnings.filterwarnings("ignore", category=DeprecationWarning, module="websockets.legacy")


def test_foxglove_bridge_import() -> None:
    """Test that the foxglove bridge can be imported successfully."""
    try:
        from dimos_lcm.foxglove_bridge import FoxgloveBridge
    except ImportError as e:
        pytest.fail(f"Failed to import foxglove bridge: {e}")


def test_foxglove_bridge_runner_init() -> None:
    """Test that LcmFoxgloveBridge can be initialized with default parameters."""
    try:
        from dimos_lcm.foxglove_bridge import FoxgloveBridge

        runner = FoxgloveBridge(host="localhost", port=8765, debug=False, num_threads=2)

        # Check that the runner was created successfully
        assert runner is not None

    except Exception as e:
        pytest.fail(f"Failed to initialize LcmFoxgloveBridge: {e}")


def test_foxglove_bridge_runner_params() -> None:
    """Test that LcmFoxgloveBridge accepts various parameter configurations."""
    try:
        from dimos_lcm.foxglove_bridge import FoxgloveBridge

        configs = [
            {"host": "0.0.0.0", "port": 8765, "debug": True, "num_threads": 1},
            {"host": "127.0.0.1", "port": 9090, "debug": False, "num_threads": 4},
            {"host": "localhost", "port": 8080, "debug": True, "num_threads": 2},
        ]

        for config in configs:
            runner = FoxgloveBridge(**config)
            assert runner is not None

    except Exception as e:
        pytest.fail(f"Failed to create runner with different configs: {e}")


def test_bridge_runner_has_run_method() -> None:
    """Test that the bridge runner has a run method that can be called."""
    try:
        from dimos_lcm.foxglove_bridge import FoxgloveBridge

        runner = FoxgloveBridge(host="localhost", port=8765, debug=False, num_threads=1)

        # Check that the run method exists
        assert hasattr(runner, "run")
        assert callable(runner.run)

    except Exception as e:
        pytest.fail(f"Failed to verify run method: {e}")
