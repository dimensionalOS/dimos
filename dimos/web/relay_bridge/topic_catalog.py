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

"""Bounded metadata catalog; never subscribes to payloads for discovery."""

from collections import OrderedDict
from dataclasses import dataclass, field
import hashlib
import secrets

from dimos.web.relay_bridge.topic_protocol import MAX_TOPIC_PAGE, TopicDescriptor


@dataclass
class _Entry:
    data: TopicDescriptor
    observed: bool = False
    advertised: bool = False
    owner: str | None = None


@dataclass
class _Owner:
    revision: int
    expires: float
    keys: set[str] = field(default_factory=set)


class TopicCatalog:
    def __init__(self, capacity: int = 4096) -> None:
        self._capacity = capacity
        self._entries: OrderedDict[str, _Entry] = OrderedDict()
        self._owners: OrderedDict[str, _Owner] = OrderedDict()
        self._epoch = secrets.token_hex(6)
        self._revision = 0

    def _trim(self) -> None:
        while len(self._entries) > self._capacity:
            key, entry = self._entries.popitem(last=False)
            if entry.owner is not None and entry.owner in self._owners:
                self._owners[entry.owner].keys.discard(key)
            self._revision += 1

    def _refresh(self, key: str) -> None:
        entry = self._entries[key]
        if not (entry.observed or entry.advertised or entry.owner):
            del self._entries[key]
            self._revision += 1
            return
        availability = "advertised" if entry.advertised else "known" if entry.owner else "observed"
        if entry.data.availability != availability:
            entry.data = entry.data.model_copy(update={"availability": availability})
            self._revision += 1

    def observe(self, key: str, encoding: str, type_name: str = "") -> None:
        entry = self._entries.get(key)
        if entry is None:
            self._entries[key] = _Entry(
                TopicDescriptor(
                    key=key, encoding=encoding, type=type_name, availability="observed"
                ),
                observed=True,
            )
            self._revision += 1
            self._trim()
            return
        entry.observed = True
        # Confirmed transport descriptors take precedence over Zenoh's default
        # opaque encoding; seeing bytes neither proves nor erases their schema.
        if entry.owner is None and (
            entry.data.encoding != encoding or (type_name and entry.data.type != type_name)
        ):
            entry.data = TopicDescriptor(
                key=key,
                encoding=encoding,
                type=type_name or entry.data.type,
                availability=entry.data.availability,
            )
            self._revision += 1
        self._refresh(key)

    def advertise(self, key: str, present: bool) -> None:
        entry = self._entries.get(key)
        if entry is None:
            if not present:
                return
            entry = _Entry(
                TopicDescriptor(key=key, encoding="", type="", availability="advertised")
            )
            self._entries[key] = entry
            self._revision += 1
        entry.advertised = present
        self._refresh(key)
        self._trim()

    def replace_known(
        self,
        owner: str,
        revision: int,
        descriptors: list[TopicDescriptor],
        now: float,
        *,
        ttl: float = 15,
    ) -> None:
        previous = self._owners.get(owner)
        if previous is not None and revision < previous.revision:
            return
        if previous is not None:
            previous.expires = now + ttl
            if revision == previous.revision:
                return
            for key in previous.keys:
                entry = self._entries.get(key)
                if entry is not None and entry.owner == owner:
                    entry.owner = None
                    self._refresh(key)
        state = _Owner(revision=revision, expires=now + ttl)
        self._owners[owner] = state
        self._owners.move_to_end(owner)
        while len(self._owners) > 64:
            oldest = next(iter(self._owners))
            self.remove_owner(oldest)
        for descriptor in descriptors[: self._capacity]:
            entry = self._entries.get(descriptor.key)
            if entry is None:
                entry = _Entry(descriptor)
                self._entries[descriptor.key] = entry
                self._revision += 1
            elif entry.data != descriptor:
                entry.data = descriptor
                self._revision += 1
            if entry.owner is not None and entry.owner in self._owners:
                self._owners[entry.owner].keys.discard(descriptor.key)
            entry.owner = owner
            state.keys.add(descriptor.key)
            self._refresh(descriptor.key)
        self._trim()

    def remove_owner(self, owner: str) -> None:
        state = self._owners.pop(owner, None)
        if state is None:
            return
        for key in state.keys:
            entry = self._entries.get(key)
            if entry is not None and entry.owner == owner:
                entry.owner = None
                self._refresh(key)

    def expire(self, now: float) -> None:
        for owner, state in list(self._owners.items()):
            if state.expires <= now:
                self.remove_owner(owner)

    def page(self, prefix: str, cursor: str, limit: int) -> tuple[list[TopicDescriptor], str]:
        if not 1 <= limit <= MAX_TOPIC_PAGE:
            raise ValueError("invalid page limit")
        start = 0
        prefix_id = hashlib.sha256(prefix.encode()).hexdigest()[:16]
        if cursor:
            parts = cursor.split(":")
            if (
                len(parts) != 4
                or parts[0] != self._epoch
                or parts[1] != str(self._revision)
                or parts[2] != prefix_id
                or not parts[3].isdigit()
            ):
                raise ValueError("catalog changed or cursor invalid")
            start = int(parts[3])
        keys = sorted(key for key in self._entries if key.startswith(prefix))
        if start > len(keys):
            raise ValueError("cursor exceeds catalog")
        end = min(len(keys), start + limit)
        next_cursor = f"{self._epoch}:{self._revision}:{prefix_id}:{end}" if end < len(keys) else ""
        return [self._entries[key].data for key in keys[start:end]], next_cursor
