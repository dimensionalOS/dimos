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

from dimos.web.relay_bridge.topic_catalog import TopicCatalog
from dimos.web.relay_bridge.topic_protocol import TopicDescriptor


def descriptor(key: str) -> TopicDescriptor:
    return TopicDescriptor(key=key, encoding="application/json", type="", availability="observed")


def test_catalog_is_empty_until_observed_and_pages_do_not_imply_liveness():
    catalog = TopicCatalog()
    assert catalog.page("", "", 32) == ([], "")
    for key in ("demo/b", "other", "demo/a"):
        catalog.observe(key, "application/json")
    first, cursor = catalog.page("demo/", "", 1)
    assert first == [descriptor("demo/a")]
    assert catalog.page("demo/", cursor, 1) == ([descriptor("demo/b")], "")


def test_catalog_evicts_at_capacity_and_rejects_stale_or_foreign_cursors():
    catalog = TopicCatalog(capacity=2)
    catalog.observe("a", "application/json")
    catalog.observe("b", "application/json")
    _, cursor = catalog.page("", "", 1)
    with pytest.raises(ValueError, match="cursor invalid"):
        TopicCatalog().page("", cursor, 1)
    catalog.observe("c", "application/json")
    with pytest.raises(ValueError, match="catalog changed"):
        catalog.page("", cursor, 1)
    assert catalog.page("", "", 32) == ([descriptor("b"), descriptor("c")], "")


def test_catalog_cursors_are_bound_to_the_requested_prefix():
    catalog = TopicCatalog()
    for key in ("a/1", "a/2", "b/1", "b/2"):
        catalog.observe(key, "application/json")
    _, cursor = catalog.page("a/", "", 1)
    with pytest.raises(ValueError, match="cursor invalid"):
        catalog.page("b/", cursor, 1)


def test_descriptors_refresh_on_reload_and_expire_without_claiming_live_data():
    catalog = TopicCatalog()
    first = TopicDescriptor(
        key="domain/remapped/Reading",
        encoding="application/cbor",
        type="example.Reading",
        availability="known",
    )
    catalog.replace_known("bridge", 1, [first], 0)
    assert catalog.page("", "", 32) == ([first], "")
    catalog.observe(first.key, "zenoh/bytes")
    assert catalog.page("", "", 32) == ([first], ""), (
        "observing opaque bytes preserves the descriptor"
    )
    replacement = first.model_copy(update={"key": "domain/reloaded/Reading"})
    catalog.replace_known("bridge", 2, [replacement], 5)
    assert catalog.page("", "", 32)[0] == [
        replacement,
        first.model_copy(update={"availability": "observed"}),
    ]
    catalog.replace_known("bridge", 1, [first], 6)
    catalog.expire(21)
    assert catalog.page("", "", 32)[0] == [first.model_copy(update={"availability": "observed"})]


def test_liveliness_is_advertised_and_disappears_without_observation():
    catalog = TopicCatalog(capacity=2)
    catalog.advertise("quiet", True)
    assert catalog.page("", "", 32)[0] == [
        TopicDescriptor(key="quiet", encoding="", type="", availability="advertised")
    ]
    catalog.advertise("quiet", False)
    assert catalog.page("", "", 32) == ([], "")
    catalog.advertise("active", True)
    catalog.observe("active", "application/json")
    catalog.advertise("active", False)
    assert catalog.page("", "", 32) == ([descriptor("active")], "")


def test_catalog_bounds_descriptor_and_liveliness_keys_together():
    catalog = TopicCatalog(capacity=2)
    catalog.replace_known("bridge", 1, [descriptor("a"), descriptor("b")], 0)
    catalog.advertise("c", True)
    assert [entry.key for entry in catalog.page("", "", 32)[0]] == ["b", "c"]
    catalog.remove_owner("bridge")
    assert [entry.key for entry in catalog.page("", "", 32)[0]] == ["c"]
