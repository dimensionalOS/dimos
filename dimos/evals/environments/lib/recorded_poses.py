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

"""Ground-truth object poses that a simulator published on ``tf``."""

from __future__ import annotations

from typing import TYPE_CHECKING, cast

if TYPE_CHECKING:
    from dimos.memory.store.base import Store
    from dimos.msgs.geometry_msgs.Transform import Transform


def last_body_transform(recording: Store, body: str) -> Transform:
    """The newest ``world -> body`` transform in the recording."""
    return _body_transform(recording, body, newest=True)


def first_body_transform(recording: Store, body: str) -> Transform:
    """The oldest ``world -> body`` transform in the recording."""
    return _body_transform(recording, body, newest=False)


def _body_transform(recording: Store, body: str, *, newest: bool) -> Transform:
    # Several modules publish on tf, so scan messages until one names the body.
    if "tf" not in recording.streams:
        raise LookupError("No tf recorded")
    for record in recording.streams.tf.order_by("ts", desc=newest):
        for transform in record.data.transforms:
            if transform.child_frame_id == body:
                return cast("Transform", transform)
    raise LookupError(f"No tf for body {body!r}; is it in tracked_bodies?")
