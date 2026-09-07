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

import sys

from pytest_mock import MockerFixture

from dimos.core.coordination.module_coordinator import _log_blueprint_graph
from dimos.core.module import Module


class NoVisualizationModule(Module):
    pass


def test_log_blueprint_graph_does_not_import_rerun_for_unrelated_blueprint(
    mocker: MockerFixture,
) -> None:
    mocker.patch.dict(sys.modules, {"dimos.visualization.rerun.bridge": None})

    _log_blueprint_graph(NoVisualizationModule.blueprint(), mocker.MagicMock())
