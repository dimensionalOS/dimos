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

"""Bridge utilities for connecting pubsub systems."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, TypeVar

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.protocol.pubsub.spec import AllPubSub, PubSub


TopicT = TypeVar("TopicT")
MsgT = TypeVar("MsgT")
TopicFrom = TypeVar("TopicFrom")
TopicTo = TypeVar("TopicTo")
MsgFrom = TypeVar("MsgFrom")
MsgTo = TypeVar("MsgTo")


class Translator(Protocol[TopicFrom, TopicTo, MsgFrom, MsgTo]):  # type: ignore[misc]
    """Protocol for translating topics and messages between pubsub systems."""

    def topic(self, topic: TopicFrom) -> TopicTo:
        """Translate a topic from source to destination format."""
        ...

    def msg(self, msg: MsgFrom) -> MsgTo:
        """Translate a message from source to destination format."""
        ...


def bridge(
    pubsub1: AllPubSub[TopicFrom, MsgFrom],
    pubsub2: PubSub[TopicTo, MsgTo],
    translator: Translator[TopicFrom, TopicTo, MsgFrom, MsgTo],
    # optionally we can override subscribe_all
    # and only bridge a specific part of the pubsub tree
    topic_from: TopicFrom | None = None,
) -> Callable[[], None]:
    def pass_msg(msg: MsgFrom, topic: TopicFrom) -> None:
        pubsub2.publish(translator.topic(topic), translator.msg(msg))

    # Bridge only specific messages from pubsub1 to pubsub2
    if topic_from:
        return pubsub1.subscribe(topic_from, pass_msg)

    # Bridge all messages from pubsub1 to pubsub2
    return pubsub1.subscribe_all(pass_msg)
