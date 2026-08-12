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

"""Generate LCM byte fixtures using DiMOS's own Python bindings.

The Kotlin codec in `android/app/src/main/java/com/dimensional/mini4pro/zenoh/`
is hand-written, so nothing about it is proven by reading it. What proves it is
byte-equality against an encoder we did not write: `lcm_encode()` from the
generated bindings at `dimos-lcm`, which is literally what DiMOS calls
(`dimos/protocol/pubsub/encoders.py:104`).

This script builds one instance of every type in `docs/zenoh-topics.md` with
**distinctive, non-default values** -- no zeros, no empty strings, no identity
quaternions -- because a fixture full of zeros hides exactly the bugs that
matter: swapped field order, wrong endianness, a float read as a double. Where a
degenerate case is itself worth pinning (an empty `Path`, a battery with no
cells, an empty string) it gets its own *extra* fixture rather than replacing the
interesting one.

Output goes to `android/app/src/test/resources/lcm/`, one `.bin` per fixture,
plus `MANIFEST.txt` recording the size and SHA-256 of each so a regeneration
shows up as a reviewable diff.

Usage:

    python3 tools/lcmfixtures/generate.py [--dimos-lcm ~/coding/dimos-lcm] [--check]

`--check` regenerates into a temporary directory and reports any difference
without writing, which is what you want when you suspect the bindings moved.

No third-party dependencies: the generated Python bindings use only `struct` and
`io`, so a bare interpreter is enough.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import os
import sys
import tempfile

# --- The values ------------------------------------------------------------
#
# Every one of these is mirrored by hand in the Kotlin tests. That duplication is
# the point: if the Kotlin side mistypes a number, the bytes stop matching. A
# fixture that carried its own field values in JSON would let a shared typo pass.

SEC = 1753622400  # 2025-07-27T12:00:00Z, comfortably inside int32
NSEC = 987654321  # nine digits, so a truncation to milliseconds is visible
SEQ = 4242

# Multi-byte UTF-8 on purpose: 'µ' and '°' are two bytes each, the battery
# emoji is four. A codec that length-prefixes character counts rather than byte
# counts fails here and nowhere else.
UTF8_TEXT = '{"code":9,"title":"battery µ 41.5°C 🔋","level":"WARNING"}'


def cov36():
    """A 6x6 that is different in every cell and negative in a third of them."""
    return [round(i * 0.25 - 3.0, 6) for i in range(36)]


def cov9():
    """A 3x3, likewise."""
    return [round(i * 1.5 - 5.0, 6) for i in range(9)]


def build(mods):
    """Return an ordered list of (fixture name, message object)."""
    Time = mods["std_msgs.Time"]
    BuiltinTime = mods["builtin_interfaces.Time"]
    Header = mods["std_msgs.Header"]
    StdString = mods["std_msgs.String"]
    StdFloat32 = mods["std_msgs.Float32"]
    Point = mods["geometry_msgs.Point"]
    Vector3 = mods["geometry_msgs.Vector3"]
    Quaternion = mods["geometry_msgs.Quaternion"]
    Pose = mods["geometry_msgs.Pose"]
    PoseWithCovariance = mods["geometry_msgs.PoseWithCovariance"]
    Twist = mods["geometry_msgs.Twist"]
    TwistStamped = mods["geometry_msgs.TwistStamped"]
    TwistWithCovariance = mods["geometry_msgs.TwistWithCovariance"]
    PoseStamped = mods["geometry_msgs.PoseStamped"]
    PointStamped = mods["geometry_msgs.PointStamped"]
    Odometry = mods["nav_msgs.Odometry"]
    Path = mods["nav_msgs.Path"]
    NavSatStatus = mods["sensor_msgs.NavSatStatus"]
    NavSatFix = mods["sensor_msgs.NavSatFix"]
    Imu = mods["sensor_msgs.Imu"]
    BatteryState = mods["sensor_msgs.BatteryState"]

    def time():
        t = Time()
        t.sec = SEC
        t.nsec = NSEC
        return t

    def header(frame_id="odom", seq=SEQ):
        h = Header()
        h.seq = seq
        h.stamp = time()
        h.frame_id = frame_id
        return h

    def point(x=12.5, y=-4.25, z=31.125):
        p = Point()
        p.x, p.y, p.z = x, y, z
        return p

    def vector3(x=-31.5, y=0.125, z=64.25):
        v = Vector3()
        v.x, v.y, v.z = x, y, z
        return v

    def quat():
        # A real rotation, none of whose components is 0 or 1: roughly
        # yaw 60 deg, pitch -20 deg, roll 15 deg.
        q = Quaternion()
        q.x, q.y, q.z, q.w = 0.20056212, -0.09442657, 0.47555491, 0.85078055
        return q

    def pose(p=None):
        o = Pose()
        o.position = p if p is not None else point()
        o.orientation = quat()
        return o

    def pose_with_cov():
        pc = PoseWithCovariance()
        pc.pose = pose()
        pc.covariance = cov36()
        return pc

    def twist():
        t = Twist()
        t.linear = vector3(3.5, -1.25, 0.75)
        t.angular = vector3(-0.125, 0.0625, -0.5)
        return t

    def twist_stamped():
        # The `setpoint` channel's type. The frame is the one the channel publishes in and
        # the angular values exercise both signs, because the yaw-rate conversion's failure
        # mode is a silent negation.
        ts = TwistStamped()
        ts.header = header("drone/world", 77)
        ts.twist = twist()
        return ts

    def twist_with_cov():
        tc = TwistWithCovariance()
        tc.twist = twist()
        tc.covariance = [round(v * -1.0, 6) for v in cov36()]
        return tc

    def pose_stamped(frame_id="odom", seq=SEQ, p=None):
        ps = PoseStamped()
        ps.header = header(frame_id, seq)
        ps.pose = pose(p)
        return ps

    def nav_sat_status(status=1, service=11):
        s = NavSatStatus()
        s.status = status
        s.service = service
        return s

    def nav_sat_fix(status=None):
        f = NavSatFix()
        f.header = header("datum/3")
        f.status = status if status is not None else nav_sat_status()
        f.latitude = 37.9838096  # Athens, southern hemisphere would flip the sign
        f.longitude = 23.7275383
        f.altitude = -12.75  # below the datum, so the sign is exercised
        f.position_covariance = cov9()
        f.position_covariance_type = 2  # DIAGONAL_KNOWN
        return f

    fixtures = []

    t = time()
    fixtures.append(("std_msgs_Time", t))

    bt = BuiltinTime()
    bt.sec = SEC
    bt.nanosec = NSEC
    fixtures.append(("builtin_interfaces_Time", bt))

    fixtures.append(("std_msgs_Header", header()))

    s = StdString()
    s.data = UTF8_TEXT
    fixtures.append(("std_msgs_String_utf8", s))

    s_empty = StdString()
    s_empty.data = ""
    fixtures.append(("std_msgs_String_empty", s_empty))

    # The `wind` channel's whole message. 9.1 is landing14's measured peak — the value whose
    # invisibility motivated the channel — and it is deliberately not float-exact, so a codec
    # that widened to double and re-narrowed differently would fail here.
    f32 = StdFloat32()
    f32.data = 9.1
    fixtures.append(("std_msgs_Float32", f32))

    fixtures.append(("geometry_msgs_Point", point()))
    fixtures.append(("geometry_msgs_Vector3", vector3()))
    fixtures.append(("geometry_msgs_Quaternion", quat()))
    fixtures.append(("geometry_msgs_Pose", pose()))
    fixtures.append(("geometry_msgs_PoseWithCovariance", pose_with_cov()))
    fixtures.append(("geometry_msgs_Twist", twist()))
    fixtures.append(("geometry_msgs_TwistStamped", twist_stamped()))
    fixtures.append(("geometry_msgs_TwistWithCovariance", twist_with_cov()))
    fixtures.append(("geometry_msgs_PoseStamped", pose_stamped()))

    pts = PointStamped()
    pts.header = header("base_link", 9)
    pts.point = point(-100.5, 250.25, 8.0)
    fixtures.append(("geometry_msgs_PointStamped", pts))

    od = Odometry()
    od.header = header("odom")
    od.child_frame_id = "base_link"
    od.pose = pose_with_cov()
    od.twist = twist_with_cov()
    fixtures.append(("nav_msgs_Odometry", od))

    # Three poses, each distinguishable, so an off-by-one in the loop shows up as
    # a value mismatch rather than a length mismatch.
    path = Path()
    path.header = header("odom", 11)
    path.poses = [
        pose_stamped("odom", 1, point(1.5, -2.5, 3.5)),
        pose_stamped("odom", 2, point(-10.25, 20.75, 30.125)),
        pose_stamped("odom", 3, point(0.0625, 0.125, -0.25)),
    ]
    path.poses_length = len(path.poses)
    fixtures.append(("nav_msgs_Path_three", path))

    empty_path = Path()
    empty_path.header = header("odom", 12)
    empty_path.poses = []
    empty_path.poses_length = 0
    fixtures.append(("nav_msgs_Path_empty", empty_path))

    fixtures.append(("sensor_msgs_NavSatStatus", nav_sat_status()))
    # status = -1 (STATUS_NO_FIX). int8 is signed; a byte-vs-int slip reads 255.
    fixtures.append(("sensor_msgs_NavSatStatus_nofix", nav_sat_status(-1, 4)))

    fixtures.append(("sensor_msgs_NavSatFix", nav_sat_fix()))

    imu = Imu()
    imu.header = header("base_link")
    imu.orientation = quat()
    imu.orientation_covariance = cov9()
    imu.angular_velocity = vector3(0.25, -0.5, 1.75)
    imu.angular_velocity_covariance = [round(v * 2.0, 6) for v in cov9()]
    imu.linear_acceleration = vector3(-0.75, 0.375, 9.80665)
    imu.linear_acceleration_covariance = [round(v * -0.5, 6) for v in cov9()]
    fixtures.append(("sensor_msgs_Imu", imu))

    bat = BatteryState()
    bat.header = header("base_link", 77)
    bat.voltage = 11.7
    bat.temperature = 41.5
    bat.current = -8.25  # negative while discharging, per ROS
    bat.charge = 1.35
    bat.capacity = 2.59
    bat.design_capacity = 2.59
    bat.percentage = 0.63  # 0..1, not 0..100
    bat.power_supply_status = 2  # DISCHARGING
    bat.power_supply_health = 2  # OVERHEAT -- the failure this airframe has
    bat.power_supply_technology = 3  # LIPO
    bat.present = True
    bat.cell_voltage = [3.91, 3.89, 3.9]
    bat.cell_temperature = [40.5, 42.5]
    bat.cell_voltage_length = len(bat.cell_voltage)
    bat.cell_temperature_length = len(bat.cell_temperature)
    bat.location = "bay µ1"  # multi-byte, inside a message that also has arrays
    bat.serial_number = "DJI-MINI4PRO-0001"
    fixtures.append(("sensor_msgs_BatteryState", bat))

    empty_bat = BatteryState()
    empty_bat.header = header("base_link", 78)
    empty_bat.voltage = 12.05
    empty_bat.temperature = 22.0
    empty_bat.current = -1.5
    empty_bat.charge = 2.0
    empty_bat.capacity = 2.59
    empty_bat.design_capacity = 2.59
    empty_bat.percentage = 0.87
    empty_bat.power_supply_status = 4  # FULL
    empty_bat.power_supply_health = 1  # GOOD
    empty_bat.power_supply_technology = 3
    empty_bat.present = False
    empty_bat.cell_voltage = []
    empty_bat.cell_temperature = []
    empty_bat.cell_voltage_length = 0
    empty_bat.cell_temperature_length = 0
    empty_bat.location = ""
    empty_bat.serial_number = ""
    fixtures.append(("sensor_msgs_BatteryState_nocells", empty_bat))

    # --- The frame tree, the intrinsics and the video ----------------------
    #
    # Added 2026-07-27 with the live publisher's parity work. Three of the six
    # carry a shape nothing else in this file has: `TFMessage` is an array with
    # no header at all, `CameraInfo` hoists `D_length` ahead of the header *and*
    # puts `height` before `width`, and `CompressedVideo` carries a bare
    # `builtin_interfaces.Time` rather than a `std_msgs.Header` and puts its
    # length field first with the body in the middle.

    Transform = mods["geometry_msgs.Transform"]
    TransformStamped = mods["geometry_msgs.TransformStamped"]
    TFMessage = mods["tf2_msgs.TFMessage"]
    RegionOfInterest = mods["sensor_msgs.RegionOfInterest"]
    CameraInfo = mods["sensor_msgs.CameraInfo"]
    CompressedVideo = mods["foxglove_msgs.CompressedVideo"]

    def transform(t=None):
        tr = Transform()
        # Deliberately a Vector3 where Pose has a Point: byte-identical, different
        # type, different fingerprint. A codec that reached for PoseCodec's body
        # would produce these very bytes behind the wrong eight.
        tr.translation = t if t is not None else vector3(0.08, 0.0, -0.035)
        tr.rotation = quat()
        return tr

    def transform_stamped(parent, child, t=None, seq=SEQ):
        ts = TransformStamped()
        ts.header = header(parent, seq)
        ts.child_frame_id = child
        ts.transform = transform(t)
        return ts

    fixtures.append(("geometry_msgs_Transform", transform()))
    fixtures.append(
        (
            "geometry_msgs_TransformStamped",
            transform_stamped("drone/base_link", "drone/camera"),
        )
    )

    # Three edges, each distinguishable, in the order the publisher emits them —
    # so an off-by-one in the loop shows up as a value mismatch rather than as a
    # length mismatch, exactly as nav_msgs_Path_three does.
    tf = TFMessage()
    tf.transforms = [
        transform_stamped("drone/world", "drone/base_link", point(12.5, -4.25, 31.125), 1),
        transform_stamped("drone/base_link", "drone/camera", vector3(0.08, 0.0, 0.0), 2),
        transform_stamped("drone/camera", "drone/camera_optical", vector3(0.0, 0.0, 0.0), 3),
    ]
    tf.transforms_length = len(tf.transforms)
    fixtures.append(("tf2_msgs_TFMessage_three", tf))

    # An empty tree is not a thing the publisher emits, and it is exactly the
    # shape a hoisted length field gets wrong: five bytes of fingerprint-plus-zero.
    empty_tf = TFMessage()
    empty_tf.transforms = []
    empty_tf.transforms_length = 0
    fixtures.append(("tf2_msgs_TFMessage_empty", empty_tf))

    roi = RegionOfInterest()
    roi.x_offset, roi.y_offset = 16, 32
    roi.height, roi.width = 540, 960
    roi.do_rectify = True
    fixtures.append(("sensor_msgs_RegionOfInterest", roi))

    # The message the publisher actually sends: the FITTED focal length at 1920
    # wide, distortion published as an absence, R the identity because an
    # unrectified monocular camera's rectification *is* the identity. `D` empty
    # is the case a length-before-header bug reads as garbage rather than as a
    # short array, so it is the interesting fixture and not the degenerate one.
    f = 0.75890625 * 1920
    ci = CameraInfo()
    ci.header = header("drone/camera_optical")
    ci.height, ci.width = 1080, 1920
    ci.distortion_model = ""
    ci.D = []
    ci.D_length = 0
    ci.K = [f, 0.0, 960.0, 0.0, f, 540.0, 0.0, 0.0, 1.0]
    ci.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    ci.P = [f, 0.0, 960.0, 0.0, 0.0, f, 540.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    ci.binning_x = ci.binning_y = 0
    ci.roi = RegionOfInterest()
    fixtures.append(("sensor_msgs_CameraInfo", ci))

    # And one with a non-empty D, so the array body's position -- after
    # distortion_model, far from its own length field -- is pinned too. These are
    # NOT values this project publishes; nothing has measured this lens.
    ci_d = CameraInfo()
    ci_d.header = header("drone/camera_optical", 43)
    ci_d.height, ci_d.width = 540, 960
    ci_d.distortion_model = "plumb_bob"
    ci_d.D = [-0.25, 0.0625, -0.001, 0.002, 0.0]
    ci_d.D_length = len(ci_d.D)
    ci_d.K = [728.55, 0.0, 480.0, 0.0, 728.55, 270.0, 0.0, 0.0, 1.0]
    ci_d.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    ci_d.P = [728.55, 0.0, 480.0, 0.0, 0.0, 728.55, 270.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    ci_d.binning_x, ci_d.binning_y = 2, 2
    ci_d.roi = roi
    fixtures.append(("sensor_msgs_CameraInfo_distorted", ci_d))

    # An access unit shaped like a real one: a four-byte Annex-B start code, an
    # SPS NAL, another start code, an IDR. Short, but every byte value matters --
    # a codec that wrote the body as a string, or length-prefixed it twice, or
    # dropped the trailing format string, fails on these bytes and on no others.
    cv = CompressedVideo()
    cv.timestamp = bt
    cv.frame_id = "drone/camera_optical"
    cv.data = bytes([0, 0, 0, 1, 0x67, 0x42, 0xC0, 0x1F, 0, 0, 0, 1, 0x65, 0x88, 0x84, 0x00])
    cv.data_length = len(cv.data)
    cv.format = "h264"
    fixtures.append(("foxglove_msgs_CompressedVideo", cv))

    # Zero bytes of payload: not something the publisher emits (it refuses a
    # non-positive length), and the case where a length-first field with a body
    # in the middle can silently swallow the format string.
    cv_empty = CompressedVideo()
    cv_empty.timestamp = bt
    cv_empty.frame_id = "drone/camera_optical"
    cv_empty.data = b""
    cv_empty.data_length = 0
    cv_empty.format = "h264"
    fixtures.append(("foxglove_msgs_CompressedVideo_empty", cv_empty))

    # --- The tag detections -----------------------------------------------
    #
    # Added 2026-07-28 with the `detections` channel. `Detection3D` is the only
    # type in this file whose hoisted array length has *more fields after the
    # array*: get `results_length` wrong and `bbox` decodes out of the tail of a
    # hypothesis and `id` out of `bbox`, rather than the message simply ending
    # early. So the interesting fixture is the one-result case the publisher
    # actually sends, and a zero-result one is carried beside it because that is
    # where a hoisted length with a trailing body goes wrong most quietly.
    #
    # The NaNs are not decoration. Nothing in this project solves a tag's
    # orientation or its 3D extent, and `BoundingBox3D` has no way to say so --
    # a size of (0,0,0) claims the object is a point. `struct.pack('>d')` writes
    # a quiet NaN and the Kotlin writer writes the same bits, which is the thing
    # worth pinning: a codec that routed NaN through a float, or normalised it,
    # or refused it, fails on these bytes and on no others.

    ObjectHypothesis = mods["vision_msgs.ObjectHypothesis"]
    ObjectHypothesisWithPose = mods["vision_msgs.ObjectHypothesisWithPose"]
    BoundingBox3D = mods["vision_msgs.BoundingBox3D"]
    Detection3D = mods["vision_msgs.Detection3D"]

    NAN = float("nan")

    def hypothesis(class_id="tag36h11:7", score=41.75):
        h = ObjectHypothesis()
        h.class_id = class_id
        h.score = score
        return h

    def hypothesis_with_pose(class_id="tag36h11:7", score=41.75, p=None):
        # A real detection's shape: the tag left of and above centre, 3.5 m down
        # the optical axis, with an orientation nothing solved.
        hp = ObjectHypothesisWithPose()
        hp.hypothesis = hypothesis(class_id, score)
        pose_ = Pose()
        pose_.position = p if p is not None else point(-0.375, 0.8125, 3.5)
        q = Quaternion()
        q.x = q.y = q.z = q.w = NAN
        pose_.orientation = q
        pc = PoseWithCovariance()
        pc.pose = pose_
        # ROS's own documented "covariance unknown", and the same all-zero matrix
        # `odom` and `pose` already publish. Deliberately not NaN: NaN is this
        # project's word for a quantity with no feed, and ROS has a word for this
        # one.
        pc.covariance = [0.0] * 36
        hp.pose = pc
        return hp

    def unsolved_box():
        b = BoundingBox3D()
        centre = Pose()
        centre.position = point(NAN, NAN, NAN)
        q = Quaternion()
        q.x = q.y = q.z = q.w = NAN
        centre.orientation = q
        b.center = centre
        b.size = vector3(NAN, NAN, NAN)
        return b

    fixtures.append(("vision_msgs_ObjectHypothesis", hypothesis()))
    fixtures.append(("vision_msgs_ObjectHypothesisWithPose", hypothesis_with_pose()))
    fixtures.append(("vision_msgs_BoundingBox3D", unsolved_box()))

    det = Detection3D()
    det.header = header("drone/camera_optical")
    det.results = [hypothesis_with_pose()]
    det.results_length = len(det.results)
    det.bbox = unsolved_box()
    det.id = "tag36h11:7"  # bare label, Ivan 2026-07-29: no suffix stuffing
    fixtures.append(("vision_msgs_Detection3D", det))

    # Two results, so an off-by-one in the loop shows up as a value mismatch --
    # and so the position of `bbox` and `id` after a *variable* number of
    # hypotheses is pinned by more than one length. Not a message the publisher
    # sends: one sighting is one object, and two hypotheses would mean two
    # candidate objects rather than one object described twice.
    det_two = Detection3D()
    det_two.header = header("drone/camera_optical", 7)
    det_two.results = [
        hypothesis_with_pose("tag36h11:7", 41.75, point(-0.375, 0.8125, 3.5)),
        hypothesis_with_pose("tag36h11:12", 18.5, point(0.125, -0.0625, 6.25)),
    ]
    det_two.results_length = len(det_two.results)
    det_two.bbox = unsolved_box()
    det_two.id = "tag36h11:7"
    fixtures.append(("vision_msgs_Detection3D_two", det_two))

    # The SOLVED variant, added 2026-07-28 with the pose solve: a real orientation in
    # `results[0]`, a bbox whose centre is the solved pose and whose size is the tag's flat
    # square -- (0.075, 0.075, 0.0), the zero z-extent being a measurement of the marker and
    # not a refused zero -- and the SAME bare id as the unsolved variant (Ivan 2026-07-29):
    # solvedness travels structurally, in the box and the orientation. The solved translation
    # (0.0625, -0.03125, 3.46875) deliberately differs from the apparent-size position so a
    # codec or encoder that wrote one where the other belongs fails on these bytes.
    def solved_box():
        b = BoundingBox3D()
        centre = Pose()
        centre.position = point(0.0625, -0.03125, 3.46875)
        centre.orientation = quat()
        b.center = centre
        b.size = vector3(0.075, 0.075, 0.0)
        return b

    def solved_hypothesis():
        # Same position as the unsolved fixture -- the solve must NOT move it -- with the
        # solved orientation where the NaNs were.
        hp = hypothesis_with_pose()
        hp.pose.pose.orientation = quat()
        return hp

    det_solved = Detection3D()
    det_solved.header = header("drone/camera_optical", 9)
    det_solved.results = [solved_hypothesis()]
    det_solved.results_length = 1
    det_solved.bbox = solved_box()
    det_solved.id = "tag36h11:7"  # solvedness travels structurally (real box vs NaN), not in the id
    fixtures.append(("vision_msgs_Detection3D_solved", det_solved))

    # No results at all: not something the encoder produces -- it refuses rather
    # than publishing an empty detection -- and exactly the shape where a length
    # field written after the header, or derived from the wrong list, decodes
    # `bbox` out of the header's tail.
    det_empty = Detection3D()
    det_empty.header = header("drone/camera_optical", 8)
    det_empty.results = []
    det_empty.results_length = 0
    det_empty.bbox = unsolved_box()
    det_empty.id = ""
    fixtures.append(("vision_msgs_Detection3D_empty", det_empty))

    # The type the channel is actually typed as, since 2026-07-28: ROS's own
    # convention for a detection topic, and what DiMOS consumes natively. Shape
    # is `nav_msgs.Path`'s exactly -- hoisted length, header, nothing after the
    # array -- one type further out, so there are two hoisted lengths nested on
    # one wire and the one-element fixture alone cannot tell them apart. Hence
    # the two- and zero-element variants below, for the same reason the inner
    # type has them.
    Detection3DArray = mods["vision_msgs.Detection3DArray"]

    def detection_array(dets, seq=SEQ):
        a = Detection3DArray()
        a.header = header("drone/camera_optical", seq)
        a.detections = dets
        a.detections_length = len(dets)
        return a

    # The message the publisher sends. Its header restates the element's, which
    # is what the encoder does and what a two-element array could not do.
    fixtures.append(("vision_msgs_Detection3DArray", detection_array([det])))

    # Two elements, so the *outer* count is pinned against a value the inner
    # count is not. Not a message the publisher sends today: `TagRecogniser`
    # reduces each frame to its largest tag before anything downstream sees one.
    fixtures.append(
        (
            "vision_msgs_Detection3DArray_two",
            detection_array([det, det_two], seq=7),
        )
    )

    # No detections at all: five bytes of fingerprint-plus-zero and a header,
    # and the shape a length written after the header reads as garbage. The
    # encoder never produces one -- it refuses a sighting rather than publishing
    # an empty array.
    fixtures.append(("vision_msgs_Detection3DArray_empty", detection_array([], seq=8)))

    # The solved element on the wire: what the channel carries when both gates pass.
    fixtures.append(("vision_msgs_Detection3DArray_solved", detection_array([det_solved], seq=9)))

    # ---- diagnostic_msgs: the `warnings` channel ---------------------------
    #
    # landing17's own wind warning, as the bus carries it. The values are that
    # flight's: DJI's `LEVEL_2` at t=111.078 with `windSpeedDmS` peaking at 142,
    # i.e. 14.2 m/s against a rated ~10.7. Multi-byte UTF-8 is deliberately NOT
    # used here -- the sentence is what QGC gets, and QGC gets ASCII -- but the
    # nested `values` list is, because the length field of a nested variable
    # array inside another variable array is exactly the byte a hand-written
    # codec gets wrong.
    KeyValue = mods["diagnostic_msgs.KeyValue"]
    DiagnosticStatus = mods["diagnostic_msgs.DiagnosticStatus"]
    DiagnosticArray = mods["diagnostic_msgs.DiagnosticArray"]

    def kv(key, value):
        k = KeyValue()
        k.key = key
        k.value = value
        return k

    def status(level, name, message, hardware_id, values):
        s = DiagnosticStatus()
        s.level = level
        s.name = name
        s.message = message
        s.hardware_id = hardware_id
        s.values = values
        s.values_length = len(values)
        return s

    def diagnostic_array(statuses, seq=SEQ):
        a = DiagnosticArray()
        a.header = header("drone/base_link", seq)
        a.status = statuses
        a.status_length = len(statuses)
        return a

    wind_status = status(
        2,  # ERROR -- DJI WARNING maps one step up, see warn/WarnLevel
        "wind/windWarning",
        "DJI: Strong wind 14.2 m/s",
        "",
        [
            kv("source", "wind"),
            kv("state", "LEVEL_2"),
            kv("level", "WARNING"),
            kv("change", "appeared"),
            kv("measurement", "14.2 m/s"),
            kv("forwarded", "true"),
        ],
    )
    fixtures.append(("diagnostic_msgs_KeyValue", kv("state", "LEVEL_2")))
    fixtures.append(("diagnostic_msgs_DiagnosticStatus", wind_status))
    fixtures.append(("diagnostic_msgs_DiagnosticArray", diagnostic_array([wind_status])))

    # A health warning with a hardware id and no values: the empty nested array,
    # which is where a length written after the strings reads as garbage rather
    # than as nothing.
    fixtures.append(
        (
            "diagnostic_msgs_DiagnosticArray_empty",
            diagnostic_array(
                [status(0, "health/0x1600A0", "DJI cleared: Aircraft overheating", "0/0", [])],
                seq=11,
            ),
        )
    )

    return fixtures


def load(dimos_lcm: str):
    """Import the generated bindings, returning {'pkg.Type': class}."""
    root = os.path.join(dimos_lcm, "generated", "python_lcm_msgs")
    if not os.path.isdir(os.path.join(root, "lcm_msgs")):
        raise SystemExit(f"no lcm_msgs package under {root}")
    sys.path.insert(0, root)
    names = [
        "std_msgs.Time",
        "builtin_interfaces.Time",
        "std_msgs.Header",
        "std_msgs.String",
        "std_msgs.Float32",
        "geometry_msgs.Point",
        "geometry_msgs.Vector3",
        "geometry_msgs.Quaternion",
        "geometry_msgs.Pose",
        "geometry_msgs.PoseWithCovariance",
        "geometry_msgs.Twist",
        "geometry_msgs.TwistStamped",
        "geometry_msgs.TwistWithCovariance",
        "geometry_msgs.PoseStamped",
        "geometry_msgs.PointStamped",
        "nav_msgs.Odometry",
        "nav_msgs.Path",
        "sensor_msgs.NavSatStatus",
        "sensor_msgs.NavSatFix",
        "sensor_msgs.Imu",
        "sensor_msgs.BatteryState",
        "geometry_msgs.Transform",
        "geometry_msgs.TransformStamped",
        "tf2_msgs.TFMessage",
        "sensor_msgs.RegionOfInterest",
        "sensor_msgs.CameraInfo",
        "foxglove_msgs.CompressedVideo",
        "vision_msgs.ObjectHypothesis",
        "vision_msgs.ObjectHypothesisWithPose",
        "vision_msgs.BoundingBox3D",
        "vision_msgs.Detection3D",
        "vision_msgs.Detection3DArray",
        "diagnostic_msgs.KeyValue",
        "diagnostic_msgs.DiagnosticStatus",
        "diagnostic_msgs.DiagnosticArray",
    ]
    out = {}
    for n in names:
        pkg, cls = n.split(".")
        out[n] = getattr(importlib.import_module(f"lcm_msgs.{pkg}.{cls}"), cls)
    return out, root


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(os.path.dirname(here))
    default_out = os.path.join(repo, "android", "app", "src", "test", "resources", "lcm")

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dimos-lcm", default=os.path.expanduser("~/coding/dimos-lcm"))
    ap.add_argument("--out", default=default_out)
    ap.add_argument("--check", action="store_true", help="compare, do not write")
    args = ap.parse_args()

    mods, root = load(args.dimos_lcm)
    fixtures = build(mods)

    out_dir = tempfile.mkdtemp(prefix="lcmfix-") if args.check else args.out
    os.makedirs(out_dir, exist_ok=True)

    manifest = [
        "# LCM byte fixtures",
        "#",
        "# Generated by tools/lcmfixtures/generate.py from the bindings at",
        f"#   {root}",
        "# using each type's own lcm_encode(). Do not edit these bytes by hand: a",
        "# hand-written fixture proves only that we agree with ourselves.",
        "#",
        "# name  bytes  sha256  fingerprint",
    ]
    for name, msg in fixtures:
        data = msg.lcm_encode()
        with open(os.path.join(out_dir, name + ".bin"), "wb") as f:
            f.write(data)
        manifest.append(
            "%-40s %6d  %s  %s"
            % (name, len(data), hashlib.sha256(data).hexdigest()[:16], data[:8].hex())
        )

    manifest_text = "\n".join(manifest) + "\n"
    with open(os.path.join(out_dir, "MANIFEST.txt"), "w") as f:
        f.write(manifest_text)

    if args.check:
        differs = []
        for name, _ in fixtures:
            a = os.path.join(args.out, name + ".bin")
            b = os.path.join(out_dir, name + ".bin")
            if not os.path.exists(a) or open(a, "rb").read() != open(b, "rb").read():
                differs.append(name)
        if differs:
            print("DIFFERS: " + ", ".join(differs))
            return 1
        print(f"{len(fixtures)} fixtures match {args.out}")
        return 0

    print(manifest_text)
    print(f"wrote {len(fixtures)} fixtures to {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
