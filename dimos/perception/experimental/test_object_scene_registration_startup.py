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

import importlib
import os
from pathlib import Path
import subprocess
import sys
import textwrap

import pytest

from dimos.core.stream import In
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.experimental.objectDB import ObjectDB
from dimos.protocol.rpc.zenohrpc import ZenohRPC

DETECTORS = {
    "moondream": ("dimos.models.vl.moondream", "MoondreamVlModel"),
    "owlv2": ("dimos.perception.detection.detectors.owlv2", "Owlv2Detector"),
}


def _run_probe(code, *args):
    result = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(code), *args],
        cwd=Path(__file__).resolve().parents[3],
        env={**os.environ, "HF_HUB_OFFLINE": "1"},
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("backend", ["yoloe", "moondream", "owlv2"])
def test_imports_only_selected_detector_at_construction(backend):
    # Fresh interpreters keep test collection from masking eager imports.
    _run_probe(
        """
        import sys
        from unittest.mock import Mock
        import pytest

        from dimos.models.base import LocalModel
        from dimos.perception.experimental.object_scene_registration import (
            ObjectSceneRegistrationModule,
        )
        from dimos.protocol.rpc.zenohrpc import ZenohRPC
        from dimos.perception.experimental.objectDB import ObjectDB

        detector_modules = {
            "moondream": "dimos.models.vl.moondream",
            "owlv2": "dimos.perception.detection.detectors.owlv2",
        }
        assert all(name not in sys.modules for name in detector_modules.values())
        assert "transformers.models.auto.modeling_auto" not in sys.modules

        backend = sys.argv[1]
        with pytest.MonkeyPatch.context() as patch:
            # No transport session, subscriptions, or model construction.
            patch.setattr(ZenohRPC, "start", Mock())
            patch.setattr(ZenohRPC, "serve_module_rpc", Mock())
            patch.setattr(ZenohRPC, "stop", Mock())
            # Scene cleanup imports Open3D; it is unrelated to detector startup.
            patch.setattr(ObjectDB, "clear", Mock())
            construct = Mock(side_effect=AssertionError("Model constructed before start"))
            patch.setattr(LocalModel, "__init__", construct)
            module = ObjectSceneRegistrationModule(
                detector_backend=backend, rpc_transport=ZenohRPC,
            )
            try:
                assert {
                    key for key, name in detector_modules.items() if name in sys.modules
                } == ({backend} if backend != "yoloe" else set())
                assert module._detector is None
                construct.assert_not_called()
                assert ("transformers.models.auto.modeling_auto" in sys.modules) == (
                    backend == "moondream"
                )
            finally:
                module.stop()
        """,
        backend,
    )


@pytest.fixture
def rpc_boundary(mocker):
    mocker.patch.object(ZenohRPC, "start")
    mocker.patch.object(ZenohRPC, "serve_module_rpc")
    mocker.patch.object(ObjectDB, "clear")
    return mocker.patch.object(ZenohRPC, "stop")


@pytest.mark.parametrize("backend", DETECTORS)
def test_detector_construction_and_loading_wait_until_start(backend, rpc_boundary, mocker):
    module_name, class_name = DETECTORS[backend]
    detector_class = mocker.patch.object(importlib.import_module(module_name), class_name)
    mocker.patch.object(In, "subscribe")
    mocker.patch.object(In, "observable")
    mocker.patch("dimos.perception.experimental.object_scene_registration.align_timestamped")
    mocker.patch("dimos.perception.experimental.object_scene_registration.backpressure")
    mocker.patch("dimos.perception.experimental.object_scene_registration.YoloeBoxSegmenter")

    module = ObjectSceneRegistrationModule(detector_backend=backend, rpc_transport=ZenohRPC)
    try:
        detector_class.assert_not_called()

        module.start()

        detector_class.assert_called_once_with()
        detector_class.return_value.start.assert_called_once_with()
        assert module._detector is detector_class.return_value
    finally:
        module.stop()


@pytest.mark.parametrize("backend", DETECTORS)
def test_missing_selected_dependency_closes_module(backend, rpc_boundary, monkeypatch):
    module_name, _ = DETECTORS[backend]
    monkeypatch.setitem(sys.modules, module_name, None)

    with pytest.raises(ModuleNotFoundError, match=module_name):
        ObjectSceneRegistrationModule(detector_backend=backend, rpc_transport=ZenohRPC)

    rpc_boundary.assert_called_once_with()
