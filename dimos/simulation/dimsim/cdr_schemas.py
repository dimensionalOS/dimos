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

"""Export canonical generated ROS 2 schemas for the standalone DimSim bundle.

Run ``python -m dimos.simulation.dimsim.cdr_schemas`` after changing messages.
"""

import json

from dimos_generated.geometry_msgs.msg import PoseStamped, Twist
from dimos_generated.sensor_msgs.msg import Image, PointCloud2

from dimos.constants import DIMOS_PROJECT_ROOT

SCHEMA_PATH = DIMOS_PROJECT_ROOT / "misc/DimSim/cli/bridge/cdr_schemas.ts"


def schema_source() -> str:
    definitions = {t.msg_name: t.schema for t in (Image, PointCloud2, PoseStamped, Twist)}
    return (
        "// Generated from the canonical .msg definitions; do not edit by hand.\n"
        "export const schemas: Record<string, string> = "
        + json.dumps(definitions, indent=2)
        + ";\n"
    )


if __name__ == "__main__":
    SCHEMA_PATH.write_text(schema_source())
