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

from pathlib import Path


def test_rerun_dashboard_uses_current_host_and_rerun_ports() -> None:
    html = Path(__file__).with_name("rerun_dashboard.html").read_text()

    assert "window.location.origin}/command-center" in html
    assert "window.location.hostname" in html
    assert "rerunGrpcPort" in html
    assert "rerunWebPort" in html
    assert "window.location.protocol" in html
    assert "normalizeHttpProtocol" in html
    assert "rerunWebProtocol" in html
    assert "rerunGrpcProtocol" in html
    assert "9877" in html
    assert "9878" in html


def test_rerun_dashboard_does_not_use_stale_rerun_defaults() -> None:
    html = Path(__file__).with_name("rerun_dashboard.html").read_text()

    assert "localhost:9090" not in html
    assert "localhost:9876" not in html
    assert ":9090" not in html
    assert ":9876" not in html


def test_rerun_dashboard_does_not_hardcode_rerun_web_scheme() -> None:
    html = Path(__file__).with_name("rerun_dashboard.html").read_text()

    assert "rerun.src = `http://" not in html
    assert "rerun+http://${rerunUrlHost}" not in html
    assert "rerun.src = `${rerunWebProtocol}://" in html
