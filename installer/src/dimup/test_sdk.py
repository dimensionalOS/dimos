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

from dimup.process import SetupError
from dimup.sdk import consumer_policy


def test_consumer_policy_keeps_sdk_extras_and_sources_without_dev_groups():
    manifest = {
        "project": {
            "optional-dependencies": {
                "all": [],
                "spot": [],
                "learning": [],
                "dds": [],
                "unitree-dds": [],
            }
        },
        "dependency-groups": {"docs": ["sphinx"]},
        "tool": {
            "uv": {
                "sources": {"graspgenx": {"git": "https://example.com/grasp", "rev": "abc"}},
                "override-dependencies": ["numpy>=2"],
                "default-groups": ["docs"],
            }
        },
    }
    extras, policy = consumer_policy(manifest)
    assert extras == ["all", "learning", "spot"]
    assert policy == {
        "sources": {"graspgenx": {"git": "https://example.com/grasp", "rev": "abc"}},
        "override-dependencies": ["numpy>=2"],
    }
    policy["sources"]["graspgenx"]["rev"] = "changed"
    assert manifest["tool"]["uv"]["sources"]["graspgenx"]["rev"] == "abc"


def test_consumer_policy_rejects_nonportable_sources():
    with pytest.raises(SetupError, match="checkout-local"):
        consumer_policy(
            {
                "project": {"optional-dependencies": {"all": []}},
                "tool": {"uv": {"sources": {"local": {"path": "../local"}}}},
            }
        )
