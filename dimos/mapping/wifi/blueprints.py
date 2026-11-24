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

from pathlib import Path

from dimos.core.coordination.blueprints import MetaBlueprint, autoconnect
from dimos.hardware.sensors.wifi.module import WifiMonitor
from dimos.hardware.sensors.wifi.survey import NetworkSurvey
from dimos.mapping.wifi.module import WifiCoverage


def wifi_survey(
    interface: str,
    targets: dict[str, str],
    record_path: Path,
    desktop_diagnostics_url: str | None = None,
) -> MetaBlueprint:
    """Compose with any robot's odometry; collect coverage and performance evidence."""
    return autoconnect(
        WifiMonitor.blueprint(interface=interface),
        NetworkSurvey.blueprint(
            interface=interface, targets=targets, desktop_diagnostics_url=desktop_diagnostics_url
        ),
        WifiCoverage.blueprint(record_path=record_path),
    )
