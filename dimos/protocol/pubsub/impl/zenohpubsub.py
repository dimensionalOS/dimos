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

"""Zenoh PubSub implementation — stub for Phase 1 scaffolding.

This module will be implemented in Phase 2. The stub exists so that
transport.py can import from it when ZENOH_AVAILABLE is True.
"""

from __future__ import annotations

from dimos.protocol.pubsub.impl.lcmpubsub import Topic

# Phase 2 will implement these:
# - ZenohPubSubBase(ZenohService, AllPubSub[Topic, bytes])
# - Zenoh(LCMEncoderMixin, ZenohPubSubBase)
# - PickleZenoh(PickleEncoderMixin, ZenohPubSubBase)


class Zenoh:
    """Stub — LCM-encoded Zenoh PubSub. Implemented in Phase 2."""

    def __init__(self, **kwargs):  # type: ignore[no-untyped-def]
        raise NotImplementedError("Zenoh transport not yet implemented")


class PickleZenoh:
    """Stub — Pickle-encoded Zenoh PubSub. Implemented in Phase 2."""

    def __init__(self, **kwargs):  # type: ignore[no-untyped-def]
        raise NotImplementedError("PickleZenoh transport not yet implemented")


__all__ = [
    "PickleZenoh",
    "Topic",
    "Zenoh",
]
