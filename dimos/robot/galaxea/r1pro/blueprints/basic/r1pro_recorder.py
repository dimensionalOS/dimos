#!/usr/bin/env python3
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

"""The R1 Pro with its sensors and Point-LIO, and nothing that plans.

Two blueprints, one shape:

``r1pro-recorder``
    Every sensor at full rate, for a dataset to score a depth change against or
    a drive to look at afterwards.

``r1pro-calibration-recorder``
    The head stereo calibration capture: both eyes and both infos at 30 fps,
    Point-LIO's cloud and pose, tf -- and no wrist cameras, whose JPEG decode
    would be taking CPU from the frames this recording exists for.

For collecting data rather than driving on it. The nav stack is the expensive
part of ``r1pro-kronknav`` -- stereo matching, a raytracing voxel map, the MLS
planner and two controllers -- and none of it is wanted when the point is to
write sensor data to disk. Leaving it out gives the recorder the Orin's CPU,
which matters: with the full stack running, the head cloud in the last capture
fell ten seconds behind and dropped a tenth of its frames.

Point-LIO is in because odometry is not a derived quantity you can recover
afterwards, and the robot is hung off it rather than off the wheels: with
``publish_odom=False`` and :class:`R1ProLioMountTf`, ``base_link`` has exactly
one parent and every camera frame in the recording is placed by lidar-inertial
odometry. Leaving the wheel edge in would mean every offline camera pose came
from a speed the controller was *commanded*, which is the thing this recording
exists to stop trusting. Stereo depth can be rematched from the recorded eyes
at any time with any parameters -- that is exactly how the head's calibration
is fitted -- but where the robot *was* when each frame was taken cannot be
reconstructed from the frames, and every measurement that compares a camera
with a lidar needs it.

What is deliberately absent: `StereoCloud`. Its output is a pure function of
the two recorded eyes and the matcher's parameters, so recording it would be
writing down an answer that is about to change.

Recording a stereo calibration capture, on the robot::

    dimos run r1pro-calibration-recorder \\
        --record sqlite --record-engine rust \\
        --record-topics head_left_color,head_right_color,head_left_info,head_right_info,\\
pointlio_lidar,pointlio_odometry,tf

How to drive it: slowly, for 60-120 seconds, with the floor and at least one
wall in the head's view at 1-6 m the whole time. Put in a couple of gentle
turns -- a turn is what separates a yaw error from a baseline error, and a
straight line cannot -- and keep people from walking through the frame, since
the fit assumes the scene held still between one eye's exposure and the
other's. The resulting ``.db`` is what ``calibrate_stereo.py`` consumes; check
it first with ``python -m dimos.robot.galaxea.r1pro.recording_rates <db>
--require head_left_color=28 --require head_right_color=28``.

The full-rate recorder::

    dimos run r1pro-recorder \\
        --record sqlite --record-engine rust \\
        --record-topics head_left_color,head_right_color,head_left_info,head_right_info,\\
pointlio_lidar,pointlio_odometry,lidar,tf

``pointlio_odometry`` is the one to record: it is the pose estimate, and it is
the thing that cannot be recomputed later. ``pointlio_lidar`` is Point-LIO's
deskewed scan. ``lidar`` is the vendor driver's copy and on this robot it will
be **empty**: a Livox streams to the last host that asked, so the vendor's
``livox_ros_driver2`` stops seeing the sensor the moment Point-LIO opens its own
connection. Record it if you like, but do not expect rows in it.

**Rates.** ``pointcloud_freq`` and ``odom_freq`` are set high enough that the
output rate limit never decides anything: Point-LIO publishes once per state it
actually solves, and the estimator's own rate is the only thing throttling the
recording. That is what "full speed" means here -- not a higher number, but no
number in the way.

**Timestamps.** Point-LIO used to stamp its cloud and its odometry with the
moment they were *published* rather than the moment the state describes, so on
a run where it fell behind, its stamps ran ahead of the world -- 22 seconds on
2026-09-16 -- and every camera frame paired with tf by timestamp got a pose from
22 s earlier. Fixed in the module (see ``pointlio/cpp/publish_stamp.hpp``), which
means **the native binary has to be rebuilt on the robot** before this blueprint
is used: ``nix build .#pointlio_native`` in
``dimos/hardware/sensors/lidar/pointlio/cpp``. It also now logs
``pointlio is behind the world`` with the lag whenever the estimator trails the
wall clock by more than a second -- worth watching in the run's output, because
that lag is invisible in the data itself.

The engine has to be ``rust``: the robot ships sqlite 3.37.2 and the python
recorder writes JSONB, which needs 3.45, so it can neither write nor read its
own recordings there.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import r1pro_control
from dimos.robot.galaxea.r1pro.lio import (
    LIDAR_FRAME,
    ODOM_FRAME,
    R1ProLioMountTf,
    R1ProPointLio,
)
from dimos.visualization.vis_module import vis_module

# The rate the head cameras actually arrive at is ~28 Hz, so a 30 Hz cap is
# "every frame" while still being a cap the connection can reason about.
CALIBRATION_COLOR_HZ = 30.0

_rerun_config = {
    "tf_axes": 0.35,
    # A recording run is not a run you watch closely -- it is one you check is
    # alive -- so the viewer gets whatever is left after the recorder has what
    # it needs. (The uplink used to be the reason: it was 1.5-2.9 Mbit/s before
    # the USB wifi stick and measures 49.7 Mbit/s now, so the cap is about the
    # Orin's CPU rather than the link.)
    "max_hz": {"world/pointlio_lidar": 0.2},
    "visual_override": {
        "world/head_left_color": None,
        "world/head_right_color": None,
        "world/wrist_left_color": None,
        "world/wrist_right_color": None,
    },
}


def _sensors(*, color_publish_hz: float, enable_wrist_color: bool | None = None) -> Blueprint:
    """The robot, its cameras, and lidar odometry. Nothing downstream.

    ``publish_odom=False`` drops the wheel odometry's ``odom -> base_link``
    edge; :class:`R1ProLioMountTf` supplies ``lidar_pointlio_link ->
    base_link`` in its place, so the whole robot -- and every camera frame --
    hangs off Point-LIO. It also republishes ``base_link ->
    lidar_chassis_left_link``, which ``publish_odom=False`` takes away with it.

    Point-LIO gets a frame of its own (:data:`LIDAR_FRAME`) rather than the
    vendor URDF's ``lidar_chassis_left_link``: the connection already publishes
    ``base_link -> lidar_chassis_left_link`` off the URDF, so Point-LIO
    publishing ``odom -> lidar_chassis_left_link`` would give that frame two
    parents, and which one a consumer picks up depends on arrival order.
    """
    return autoconnect(
        r1pro_control(
            color_publish_hz=color_publish_hz,
            publish_odom=False,
            enable_wrist_color=enable_wrist_color,
        ),
        R1ProLioMountTf.blueprint(),
        R1ProPointLio.blueprint(
            sensor_frame_id=LIDAR_FRAME,
            frame_id=ODOM_FRAME,
            # Publish every state the estimator solves. These are output rate
            # limits, not sensor rates, and since the module refuses to publish
            # the same state twice, setting them above the estimator's own rate
            # takes them out of the decision entirely.
            pointcloud_freq=100.0,
            odom_freq=100.0,
        ),
        vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config),
    ).remappings(
        [
            # `R1ProConnection` publishes `lidar` and `odometry` too -- the
            # vendor driver's copy of the same physical scan, and wheel
            # odometry. Left alone, both producers land on one stream and the
            # recording interleaves them with no way to tell which row came
            # from which. Renamed, the recording has both and they stay apart.
            (R1ProPointLio, "lidar", "pointlio_lidar"),
            (R1ProPointLio, "odometry", "pointlio_odometry"),
        ]
    )


# No rate cap at all, rather than the connection's 30: the whole point of this
# blueprint is to write down what the camera saw.
r1pro_recorder = _sensors(color_publish_hz=0.0).global_config(n_workers=3, transport="zenoh")

# The head at the rate it publishes, and the wrists off: their JPEG decode is
# CPU spent on frames the calibration never looks at.
r1pro_calibration_recorder = _sensors(
    color_publish_hz=CALIBRATION_COLOR_HZ, enable_wrist_color=False
).global_config(n_workers=3, transport="zenoh")
