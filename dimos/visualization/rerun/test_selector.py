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

from dimos.visualization.rerun.selector import browser_connect_host, rerun_web_viewer_url


def test_rerun_web_viewer_url_adds_encoded_source_url() -> None:
    viewer_url = rerun_web_viewer_url(
        "http://localhost:9878",
        "rerun+http://127.0.0.1:9877/proxy",
    )

    assert viewer_url == (
        "http://localhost:9878/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A9877%2Fproxy"
    )


def test_rerun_web_viewer_url_preserves_existing_source_url() -> None:
    viewer_url = "http://localhost:9878/?url=rerun%2Bhttp%3A%2F%2Fhost%3A9877%2Fproxy"

    assert rerun_web_viewer_url(viewer_url, "rerun+http://other:9877/proxy") == viewer_url


def test_browser_connect_host_rewrites_wildcard_binds() -> None:
    assert browser_connect_host("0.0.0.0") == "localhost"
    assert browser_connect_host("::") == "localhost"
    assert browser_connect_host("127.0.0.1") == "127.0.0.1"
