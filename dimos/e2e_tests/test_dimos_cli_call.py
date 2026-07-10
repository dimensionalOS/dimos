# Copyright 2025-2026 Dimensional Inc.
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

from unittest.mock import patch

from dimos.e2e_tests.dimos_cli_call import DimosCliCall


def test_start_aligns_mcp_client_url_with_ipv4_loopback_for_custom_port() -> None:
    call = DimosCliCall()
    call.demo_args = ["coordinator-mock"]
    call.mcp_port = 12345

    with patch("dimos.e2e_tests.dimos_cli_call.subprocess.Popen") as popen:
        call.start()

    kwargs = popen.call_args.kwargs
    assert kwargs["env"]["MCPCLIENT__MCP_SERVER_URL"] == "http://127.0.0.1:12345/mcp"
