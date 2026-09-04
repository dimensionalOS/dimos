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

"""Profile-driven native recorder declarations."""

from __future__ import annotations

from dimos.core.stream import In
from dimos.experimental.memory.rust_recorder import RustRecorder
from dimos.imitation.profile import ImageSource, PolicyIOProfile
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState


def declare_recorder(
    name: str,
    module_name: str,
    profile: PolicyIOProfile,
) -> type[RustRecorder]:
    """Declare a recorder whose typed ports exactly match ``profile``."""
    annotations: dict[str, object] = {"status": In[EpisodeStatus]}
    sources = [*profile.observations.values(), profile.action.demonstration]
    for source in sources:
        annotations[source.stream] = (
            In[Image] if isinstance(source, ImageSource) else In[JointState]
        )

    return type(
        name,
        (RustRecorder,),
        {
            "__annotations__": annotations,
            "__doc__": f"Native recorder for the {profile.name!r} policy profile.",
            "__module__": module_name,
            "__qualname__": name,
            "profile": profile,
        },
    )
