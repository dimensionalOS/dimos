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

"""Enterprise Pico WebXR body tracking on the standalone SONIC stack."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import pZenohTransport
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, Topic as ZenohTopic
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sonic_wbc import (
    _zenoh_latest_transport,
    sonic_wbc_blueprint,
)
from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot
from dimos.teleop.webxr.controller_types import Buttons
from dimos.teleop.webxr.sonic import SonicWebXRTeleopModule

unitree_g1_sonic_webxr_teleop = autoconnect(
    sonic_wbc_blueprint(webxr_enabled=True),
    SonicWebXRTeleopModule.blueprint(),
).transports(
    {
        ("body_tracking", BodyTrackingSnapshot): pZenohTransport(
            ZenohTopic("dimos/g1/body_tracking", queue_capacity=1, qos=QOS_LATEST_WINS)
        ),
        ("teleop_buttons", Buttons): _zenoh_latest_transport("g1/teleop_buttons", Buttons),
    }
)
