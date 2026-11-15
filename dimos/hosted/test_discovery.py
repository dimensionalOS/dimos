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

from dimos.hosted.discovery import Found, Probe, merge
from dimos.hosted.protocol import HostDescriptor


def _host(host_id: str, router: str) -> HostDescriptor:
    return HostDescriptor(host_id, host_id, {}, "r", (), router)


def test_merge_dedups_hosts_seen_through_several_routers() -> None:
    a, b = _host("a", "za"), _host("b", "zb")
    probes = [
        Probe("tcp/10.0.0.1:7447", "za", (a, b)),
        Probe("tcp/10.0.0.2:7447", "zb", (a, b)),
        Probe("tcp/192.168.1.2:7447", "zb", (b, a)),
    ]
    assert merge(probes) == [
        Found(a, ("tcp/10.0.0.1:7447",)),
        Found(b, ("tcp/10.0.0.2:7447", "tcp/192.168.1.2:7447")),
    ]
