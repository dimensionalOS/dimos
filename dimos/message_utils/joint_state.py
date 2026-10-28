# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Construct generated joint feedback from named configuration fields."""

from collections.abc import Mapping
from typing import Any

import numpy as np

from dimos.message_utils.time import header_now, time_from_seconds
from dimos.messages.sensor_msgs.msg import JointState
from dimos.messages.std_msgs.msg import Header


def from_mapping(value: Mapping[str, Any]) -> JointState:
    stamp = value.get("ts")
    header = (
        header_now(value.get("frame_id", ""))
        if stamp is None
        else Header(time_from_seconds(stamp), value.get("frame_id", ""))
    )
    return JointState(
        header=header,
        name=list(value.get("name", [])),
        position=np.array(
            value["position"] if value.get("position") is not None else [], dtype=np.float64
        ),
        velocity=np.array(
            value["velocity"] if value.get("velocity") is not None else [], dtype=np.float64
        ),
        effort=np.array(
            value["effort"] if value.get("effort") is not None else [], dtype=np.float64
        ),
    )
