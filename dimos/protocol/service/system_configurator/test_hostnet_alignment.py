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

"""Assert the R1 Lite host network assets match dimos' own configurators.

The systemd unit and sysctl file under scripts/galaxea/r1lite/host/ must set
the same loopback route and buffer sizes that MulticastConfiguratorLinux and
BufferConfiguratorLinux check at runtime. This test fails if they drift.
"""

from pathlib import Path

import pytest

from dimos.protocol.service.system_configurator.lcm import (
    BufferConfiguratorLinux,
    MulticastConfiguratorLinux,
)

_HOST_DIR = Path(__file__).resolve().parents[4] / "scripts" / "galaxea" / "r1lite" / "host"

pytestmark = pytest.mark.skipif(
    not _HOST_DIR.is_dir(),
    reason="repo checkout only (host assets are not packaged in the wheel)",
)


def test_hostnet_unit_matches_multicast_configurator() -> None:
    unit = (_HOST_DIR / "dimos-hostnet.service").read_text()
    cfg = MulticastConfiguratorLinux()
    assert cfg.MULTICAST_PREFIX in unit, "route prefix drifted from MulticastConfiguratorLinux"
    assert f"ip link set {cfg.loopback_interface} multicast on" in unit
    assert f"ip route replace {cfg.MULTICAST_PREFIX} dev {cfg.loopback_interface}" in unit


def test_sysctl_file_matches_buffer_configurator() -> None:
    conf = (_HOST_DIR / "60-dimos.conf").read_text()
    target = BufferConfiguratorLinux.TARGET_RMEM_SIZE
    assert f"net.core.rmem_max={target}" in conf
    assert f"net.core.rmem_default={target}" in conf
