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

import pytest

from dimos.web.fastapi_server import FastAPIServer


@pytest.mark.parametrize(
    ("host", "expected"),
    [
        ("0.0.0.0", ["localhost", "192.168.1.2", "10.0.0.2"]),
        ("127.0.0.1", ["127.0.0.1"]),
        ("192.168.1.2", ["192.168.1.2"]),
        ("::1", ["[::1]"]),
    ],
)
def test_browser_urls_respect_bind_address_and_include_page(host, expected, mocker):
    mocker.patch(
        "dimos.web.fastapi_server.get_local_ips",
        return_value=[("192.168.1.2", "eth0"), ("10.0.0.2", "wlan0")],
    )
    server = FastAPIServer(host=host, port=9443)

    assert server.get_urls("/teleop", ssl=True) == [
        f"https://{address}:9443/teleop" for address in expected
    ]
