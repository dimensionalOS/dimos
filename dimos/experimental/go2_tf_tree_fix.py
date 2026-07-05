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

"""Fix the tf tree of go2 recordings made on andrew/feat/navigation-dev.

Those recordings contain two competing pose chains:

    world->base_link (from go2 odom)
    world->body (from pointlio)
        base_link->front_camera->{mid360_link, camera_optical}
        base_link->camera_link->camera_optical (go2 driver)

This rewrites them into a single tree rooted at the pointlio estimate:

    world->go2_base_link->go2_camera_link->go2_camera_optical
    world->mid360_link (from pointlio, was world->body)
        mid360_link->base_link->front_camera->camera_optical

The pointlio_odometry child frame is renamed body->mid360_link to match.

Usage: uv run python -m dimos.experimental.go2_tf_tree_fix <recording.db>
(edits the db in place)
"""

import argparse
import sqlite3

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

GO2_CHAIN_RENAMES = {
    "base_link": "go2_base_link",
    "camera_link": "go2_camera_link",
    "camera_optical": "go2_camera_optical",
}


def rewrite_tf_message(message: TFMessage, latest_base_to_front: list[Transform | None]) -> bool:
    changed = False
    for transform in message.transforms:
        if transform.frame_id == "base_link" and transform.child_frame_id == "front_camera":
            latest_base_to_front[0] = transform

    new_transforms = []
    for transform in message.transforms:
        edge = (transform.frame_id, transform.child_frame_id)
        if edge == ("world", "base_link"):
            transform.child_frame_id = "go2_base_link"
            changed = True
        elif edge == ("world", "body"):
            transform.child_frame_id = "mid360_link"
            changed = True
        elif edge in (("base_link", "camera_link"), ("camera_link", "camera_optical")):
            transform.frame_id = GO2_CHAIN_RENAMES[transform.frame_id]
            transform.child_frame_id = GO2_CHAIN_RENAMES[transform.child_frame_id]
            changed = True
        elif edge == ("front_camera", "mid360_link"):
            base_to_front = latest_base_to_front[0]
            if base_to_front is None:
                raise RuntimeError("saw front_camera->mid360_link before base_link->front_camera")
            base_to_mid = base_to_front.to_matrix() @ transform.to_matrix()
            transform = Transform.from_matrix(base_to_mid).inverse()
            transform.frame_id = "mid360_link"
            transform.child_frame_id = "base_link"
            transform.ts = base_to_front.ts
            changed = True
        new_transforms.append(transform)
    message.transforms = new_transforms
    return changed


def fix_database(db_path: str) -> None:
    connection = sqlite3.connect(db_path)

    latest_base_to_front: list[Transform | None] = [None]
    tf_updates = 0
    for row_id, data in connection.execute("SELECT id, data FROM tf_blob ORDER BY id"):
        message = TFMessage.lcm_decode(bytes(data))
        if rewrite_tf_message(message, latest_base_to_front):
            connection.execute(
                "UPDATE tf_blob SET data = ? WHERE id = ?", (message.lcm_encode(), row_id)
            )
            tf_updates += 1
    print(f"rewrote {tf_updates} tf messages")

    odom_updates = 0
    for row_id, data in connection.execute("SELECT id, data FROM pointlio_odometry_blob"):
        odometry = Odometry.lcm_decode(bytes(data))
        if odometry.child_frame_id == "body":
            odometry.child_frame_id = "mid360_link"
            connection.execute(
                "UPDATE pointlio_odometry_blob SET data = ? WHERE id = ?",
                (odometry.lcm_encode(), row_id),
            )
            odom_updates += 1
    print(f"renamed child frame on {odom_updates} pointlio_odometry messages")

    connection.commit()
    integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
    connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
    connection.close()
    print(f"integrity_check: {integrity}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("db_path", help="recording .db to fix in place")
    arguments = parser.parse_args()
    fix_database(arguments.db_path)


if __name__ == "__main__":
    main()
