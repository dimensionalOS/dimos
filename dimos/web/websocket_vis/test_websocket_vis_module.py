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

from threading import Lock
from typing import Any

from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


def _module_without_runtime() -> tuple[WebsocketVisModule, list[tuple[str, dict[str, Any]]]]:
    module = object.__new__(WebsocketVisModule)
    module.vis_state = {}
    module.state_lock = Lock()
    emitted: list[tuple[str, dict[str, Any]]] = []
    module._emit = lambda event, data: emitted.append((event, data))  # type: ignore[method-assign]
    return module, emitted


def test_set_world_model_state_stores_snapshot_and_emits_incremental_update() -> None:
    module, emitted = _module_without_runtime()
    state = {
        "prediction": {
            "risk": "high",
            "score": 0.2,
            "predicted_state_delta": {"navigation.state": "blocked_before_navigation"},
            "structural_causal_model": {
                "counterfactuals": [{"intervention": "do(spatial_has_matches=True)"}]
            },
        },
        "recent_interventions": [
            {
                "intervention_name": "look_out_for_then_tag_target",
                "target_variable": "spatial_has_matches",
            }
        ],
    }

    summary = module.set_world_model_state(state)

    assert summary["available"] is True
    assert summary["risk"] == "high"
    assert module.vis_state["world_model_state"]["prediction"]["risk"] == "high"
    assert (
        module.vis_state["world_model_state"]["recent_interventions"][0]["intervention_name"]
        == "look_out_for_then_tag_target"
    )
    assert emitted == [("world_model_state", module.vis_state["world_model_state"])]


def test_get_world_model_state_returns_current_dashboard_snapshot() -> None:
    module, _ = _module_without_runtime()
    module.vis_state["world_model_state"] = {
        "available": True,
        "prediction": {"risk": "medium", "score": 0.62},
    }

    state = module.get_world_model_state()

    assert state["available"] is True
    assert state["prediction"]["risk"] == "medium"
