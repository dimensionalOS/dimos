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

"""Static MID360-to-body mount transform for the MAVSDK drone.

Point-LIO already publishes the dynamic ``odom -> mid360_link`` edge. This
module attaches ``base_link`` below that sensor pose using the inverse of
:data:`BASE_TO_MID360`, avoiding a second parent for ``mid360_link``.

Published frame relationship::

    odom ─── mid360_link ─── base_link
           (Point-LIO)       (this publisher)

The fixed transform is periodically re-published by :class:`StaticTfPublisher`.
"""

from typing import Final

import numpy as np
from typing_extensions import override

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher, StaticTfPublisherConfig

BASE_TO_MID360: Final[Transform] = Transform.from_matrix(
    np.array(
        (
            (0.9659258, 0.0, -0.2588190, 0.010625),
            (0.0, 1.0, 0.0, 0.023290),
            (0.2588190, 0.0, 0.9659258, 0.061153),
            (0.0, 0.0, 0.0, 1.0),
        )
    ),
    frame_id="base_link",
    child_frame_id="mid360_link",
)


class Mid360MountConfig(StaticTfPublisherConfig):
    """Settings for the ``mid360_link -> base_link`` static TF broadcast.

    Inherits all fields from :class:`StaticTfPublisherConfig` (publish rate,
    frame expiry, etc.). No additional fields are needed because the actual
    transform values are supplied by :data:`BASE_TO_MID360`.
    """


class Mid360MountStaticTf(StaticTfPublisher):
    """Publish the calibrated ``mid360_link -> base_link`` static transform.

    The inverse mount transform is derived from :data:`BASE_TO_MID360`, the
    same calibration used to convert Point-LIO sensor odometry to body pose.

    Why a static TF?

    * The mount geometry is fixed per vehicle; it never changes at runtime.
    * Point-LIO owns the dynamic ``odom -> mid360_link`` edge.
    * This static edge completes ``odom -> mid360_link -> base_link`` without
      assigning two parents to ``mid360_link``.
    """

    @override
    def transforms(self) -> list[Transform]:
        """Build a single static TF entry from the calibrated mount values.

        Publishes the inverse calibration with ``mid360_link`` as parent of
        ``base_link``.

        The parent class (:class:`StaticTfPublisher`) calls this once and
        publishes the result. Subclasses override this to supply the
        frame definitions.
        """
        return [BASE_TO_MID360.inverse()]
