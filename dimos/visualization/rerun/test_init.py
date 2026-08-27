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

from unittest.mock import MagicMock

import rerun as rr

from dimos.visualization.rerun.init import rerun_init


def test_rerun_init_forwards_newest_first_to_grpc_server(mocker) -> None:
    socket_instance = MagicMock()
    socket_instance.__enter__.return_value = socket_instance
    socket_instance.connect_ex.return_value = 1
    mocker.patch("dimos.visualization.rerun.init.socket.socket", return_value=socket_instance)
    mocker.patch.object(rr, "init")
    serve_grpc = mocker.patch.object(rr, "serve_grpc", return_value="rerun+http://test:9877/proxy")
    mocker.patch(
        "dimos.visualization.rerun.init.register_colormap_annotation",
    )

    result = rerun_init(
        start_grpc=True,
        grpc_config={
            "connect_url": "rerun+http://127.0.0.1:9877/proxy",
            "server_memory_limit": "32MB",
            "newest_first": True,
        },
    )

    assert result == "rerun+http://test:9877/proxy"
    serve_grpc.assert_called_once_with(
        grpc_port=9877,
        server_memory_limit="32MB",
        newest_first=True,
    )
