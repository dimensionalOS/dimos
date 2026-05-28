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

"""PACK MIND coordinator server tests — drives the real HTTP server over requests.

Starts the actual ``ThreadingHTTPServer`` on an ephemeral port (port 0) in a
background thread, exercising the no-overlap and stop-on-find guarantees through
the JSON API exactly as a dog's agent would.
"""

from __future__ import annotations

from collections.abc import Iterator
import threading

import pytest
import requests

from dimos.experimental.pack_mind.pack_coordinator_server import (
    _parse_prefs,
    _Server,
    make_server,
)

ZONES = ["north", "east", "south", "west"]
_TIMEOUT = 5.0


@pytest.fixture
def server_url() -> Iterator[str]:
    server: _Server = make_server("127.0.0.1", 0, ZONES)
    port = server.server_address[1]
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{port}"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=_TIMEOUT)


@pytest.mark.unit
def test_two_dogs_get_distinct_zones(server_url: str) -> None:
    requests.post(f"{server_url}/start_search", json={"target": "red object"}, timeout=_TIMEOUT)
    handed: list[str] = []
    for _ in range(len(ZONES)):
        for dog in ("alpha", "bravo"):
            resp = requests.post(
                f"{server_url}/assign_zone", json={"dog": dog}, timeout=_TIMEOUT
            )
            assert resp.status_code == 200
            zone = resp.json()["zone"]
            if zone is not None:
                handed.append(zone)
    assert sorted(handed) == sorted(ZONES)  # every zone handed exactly once
    assert len(handed) == len(set(handed))  # no double-assign → no overlap


@pytest.mark.unit
def test_finding_stops_other_dog_and_assignment(server_url: str) -> None:
    requests.post(f"{server_url}/start_search", json={"target": "red object"}, timeout=_TIMEOUT)
    requests.post(f"{server_url}/assign_zone", json={"dog": "alpha"}, timeout=_TIMEOUT)

    find = requests.post(
        f"{server_url}/report_finding",
        json={"dog": "alpha", "object": "red object", "zone": "north"},
        timeout=_TIMEOUT,
    )
    assert find.status_code == 200
    assert find.json()["finding"]["zone"] == "north"

    stop = requests.get(f"{server_url}/should_stop", params={"dog": "bravo"}, timeout=_TIMEOUT)
    assert stop.status_code == 200
    assert stop.json()["stop"] is True

    assigned = requests.post(
        f"{server_url}/assign_zone", json={"dog": "bravo"}, timeout=_TIMEOUT
    )
    assert assigned.json()["zone"] is None  # no more zones handed after a find


@pytest.mark.unit
def test_state_reflects_coverage_after_cleared(server_url: str) -> None:
    requests.post(f"{server_url}/start_search", json={"target": "red object"}, timeout=_TIMEOUT)
    assert requests.get(f"{server_url}/state", timeout=_TIMEOUT).json()["coverage"] == 0.0

    requests.post(
        f"{server_url}/report_cleared", json={"dog": "alpha", "zone": "north"}, timeout=_TIMEOUT
    )
    requests.post(
        f"{server_url}/report_cleared", json={"dog": "bravo", "zone": "south"}, timeout=_TIMEOUT
    )
    state = requests.get(f"{server_url}/state", timeout=_TIMEOUT).json()
    assert state["coverage"] == pytest.approx(0.5)


@pytest.mark.unit
def test_release_dog_reclaims_zone_over_http(server_url: str) -> None:
    requests.post(f"{server_url}/start_search", json={"target": "red object"}, timeout=_TIMEOUT)
    alpha = requests.post(
        f"{server_url}/assign_zone", json={"dog": "alpha"}, timeout=_TIMEOUT
    ).json()["zone"]
    bravo = requests.post(
        f"{server_url}/assign_zone", json={"dog": "bravo"}, timeout=_TIMEOUT
    ).json()["zone"]
    requests.post(
        f"{server_url}/report_cleared", json={"dog": "bravo", "zone": bravo}, timeout=_TIMEOUT
    )
    # Alpha drops; its claimed zone must come back so a survivor can inherit it.
    rel = requests.post(f"{server_url}/release_dog", json={"dog": "alpha"}, timeout=_TIMEOUT)
    assert rel.status_code == 200
    inherited = requests.post(
        f"{server_url}/assign_zone", json={"dog": "bravo"}, timeout=_TIMEOUT
    ).json()["zone"]
    assert inherited == alpha  # bravo inherits alpha's reclaimed ground
    assert "alpha" in requests.get(f"{server_url}/state", timeout=_TIMEOUT).json()["offline"]


@pytest.mark.unit
def test_where_is_returns_finding(server_url: str) -> None:
    requests.post(f"{server_url}/start_search", json={"target": "red kit"}, timeout=_TIMEOUT)
    assert requests.get(f"{server_url}/where_is", timeout=_TIMEOUT).json()["found"] is False
    requests.post(
        f"{server_url}/report_finding",
        json={"dog": "alpha", "object": "red kit", "zone": "east"},
        timeout=_TIMEOUT,
    )
    w = requests.get(f"{server_url}/where_is", timeout=_TIMEOUT).json()
    assert w["found"] is True and w["zone"] == "east" and w["by"] == "alpha"


@pytest.mark.unit
def test_report_finding_zone_optional_over_http(server_url: str) -> None:
    requests.post(f"{server_url}/start_search", json={"target": "red kit"}, timeout=_TIMEOUT)
    claimed = requests.post(
        f"{server_url}/assign_zone", json={"dog": "alpha"}, timeout=_TIMEOUT
    ).json()["zone"]
    # No zone field at all — coordinator falls back to alpha's claimed zone.
    f = requests.post(
        f"{server_url}/report_finding",
        json={"dog": "alpha", "object": "red kit"},
        timeout=_TIMEOUT,
    )
    assert f.status_code == 200
    assert f.json()["finding"]["zone"] == claimed


@pytest.mark.unit
def test_dashboard_served_at_root(server_url: str) -> None:
    resp = requests.get(f"{server_url}/", timeout=_TIMEOUT)
    assert resp.status_code == 200
    assert "text/html" in resp.headers.get("Content-Type", "")
    assert "PACK" in resp.text and "/state" in resp.text  # the polling dashboard


@pytest.mark.unit
def test_parse_prefs() -> None:
    prefs = _parse_prefs("alpha:north,east;bravo:south,west")
    assert prefs == {"alpha": ["north", "east"], "bravo": ["south", "west"]}
    assert _parse_prefs("") == {}
    assert _parse_prefs("garbage") == {}


@pytest.mark.unit
def test_bad_input_returns_400(server_url: str) -> None:
    resp = requests.post(f"{server_url}/start_search", json={}, timeout=_TIMEOUT)
    assert resp.status_code == 400
    missing = requests.get(f"{server_url}/should_stop", timeout=_TIMEOUT)
    assert missing.status_code == 400
