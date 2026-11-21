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

import math
import socket
from types import SimpleNamespace
from uuid import UUID

import pytest

from dimos.utils.generic import finite_number, get_local_ips, short_id


def test_local_ips_default_stays_ipv4_only(mocker):
    mocker.patch(
        "dimos.utils.generic.psutil.net_if_addrs",
        return_value={
            "lo": [SimpleNamespace(family=socket.AF_INET, address="127.0.0.1")],
            "eth0": [
                SimpleNamespace(family=socket.AF_INET, address="192.168.1.2"),
                SimpleNamespace(family=socket.AF_INET6, address="fd00::2"),
            ],
        },
    )

    assert get_local_ips() == [("192.168.1.2", "eth0")]


def test_short_id_hello_world() -> None:
    assert short_id("HelloWorld") == "6GgJmzi1KYf4iaHVxk"


def test_short_id_uuid_one(mocker) -> None:
    mocker.patch("uuid.uuid4", return_value=UUID("11111111-1111-1111-1111-111111111111"))
    assert short_id() == "wcFtOGNXQnQFZ8QRh1"


def test_short_id_uuid_zero(mocker) -> None:
    mocker.patch("uuid.uuid4", return_value=UUID("00000000-0000-0000-0000-000000000000"))
    assert short_id() == "000000000000000000"


def test_finite_number_coerces_to_float() -> None:
    assert finite_number(2, "x") == 2.0
    assert isinstance(finite_number(2, "x"), float)


@pytest.mark.parametrize("value", [None, "1.5", True, math.nan, math.inf, [1.0]])
def test_finite_number_rejects_non_numbers(value: object) -> None:
    with pytest.raises(ValueError, match="x must be a finite number"):
        finite_number(value, "x")
