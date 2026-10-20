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


"""One explicitly requested synthetic API call, with no robot connection."""

import argparse

import dream
import numpy as np

from dimos.imitation.policy.dreamlabs.conversion import (
    CAMERAS,
    HORIZON,
    encode_image,
    observation,
    policy_config,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--live", action="store_true", help="send one potentially billed inference after approval"
    )
    args = parser.parse_args()
    if not args.live:
        parser.error(
            "--live requires approval to send synthetic images/state to Dreamlabs and incur inference cost"
        )
    image = encode_image(
        Image(data=np.zeros((176, 320, 3), dtype=np.uint8), format=ImageFormat.RGB)
    )
    joints = [0.0, 0.5, 0.5, -0.35, 0.0, 0.0]
    vector = np.array([*joints, *joints, 0.5, 0.5])
    try:
        with dream.DreamClient() as client:
            client.policy(policy_config("dreamlabs/dream-1-beta", 1500))
            client.set_observation(
                {f"video.{role}": image for role in CAMERAS}, observation(vector, "hold position")
            )
            reply = client.get_action()
            for key in ("q_l_chunk", "q_r_chunk"):
                rows = np.asarray(reply.get(key), dtype=np.float64)
                if rows.shape != (HORIZON, 7) or not np.isfinite(rows).all():
                    raise ValueError("malformed YAM action chunk")
    except Exception as exc:
        # Do not echo SDK payloads, credential errors, or returned motion values.
        raise SystemExit(f"Probe failed: {type(exc).__name__}") from None
    print("One synthetic inference returned two finite 48 by 7 chunks. No hardware connected.")


if __name__ == "__main__":
    main()
