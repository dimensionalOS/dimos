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

"""Integration tests: verify all paths resolve and blueprint is constructable."""

import importlib
from pathlib import Path

import pytest

from dimos.core.native_module import NativeModule


class TestAllNativeModulePaths:
    """Every NativeModule in smart_nav must have valid, existing paths."""

    @pytest.fixture(
        params=[
            "terrain_analysis",
            "local_planner",
            "path_follower",
            "far_planner",
            "tare_planner",
            "arise_slam",
        ]
    )
    def native_module(self, request):
        """Parametrized fixture that yields each native module class."""
        name = request.param
        mod = importlib.import_module(f"dimos.navigation.smart_nav.modules.{name}.{name}")
        # The class name varies; find the NativeModule subclass
        for attr_name in dir(mod):
            attr = getattr(mod, attr_name)
            if (
                isinstance(attr, type)
                and issubclass(attr, NativeModule)
                and attr is not NativeModule
            ):
                return attr
        pytest.fail(f"No NativeModule subclass found in {name}")

    def test_cwd_exists(self, native_module):
        m = native_module()
        m._resolve_paths()
        try:
            assert Path(m.config.cwd).exists()
        finally:
            m.stop()

    def test_executable_exists(self, native_module):
        m = native_module()
        m._resolve_paths()
        try:
            exe = Path(m.config.executable)
            if not exe.exists():
                pytest.skip(f"Native binary not built: {exe}")
            assert exe.is_file()
        finally:
            m.stop()

    def test_cwd_is_smart_nav_root(self, native_module):
        m = native_module()
        m._resolve_paths()
        try:
            cwd = Path(m.config.cwd).resolve()
            cmake = cwd / "CMakeLists.txt"
            if not cmake.exists():
                pytest.skip(f"CMakeLists.txt not found in cwd {cwd}")
            assert cmake.exists()
        finally:
            m.stop()


class TestDataFiles:
    def test_path_data_exists(self):
        from dimos.utils.data import get_data

        data = get_data("smart_nav_paths")
        if not data.exists():
            pytest.skip(f"Data directory not found: {data} (LFS not pulled?)")
        for f in ["startPaths.ply", "pathList.ply", "paths.ply"]:
            assert (data / f).exists(), f"Missing data file: {data / f}"


class TestBlueprintImport:
    def test_g1_nav_sim_blueprint_importable(self):
        from dimos.robot.unitree.g1.blueprints.navigation.unitree_g1_nav_sim import (
            unitree_g1_nav_sim,
        )

        assert unitree_g1_nav_sim is not None

    def test_simulation_blueprint_importable(self):
        from dimos.navigation.smart_nav.blueprints.simulation import simulation_blueprint

        assert simulation_blueprint is not None
