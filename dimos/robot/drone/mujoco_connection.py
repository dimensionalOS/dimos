#!/usr/bin/env python3

import atexit
import base64
from collections.abc import Callable
import functools
import json
import pickle
import subprocess
import sys
import threading
import time
from typing import Any, TypeVar
import weakref

import numpy as np
from numpy.typing import NDArray
from reactivex import Observable
from reactivex.abc import ObserverBase, SchedulerBase
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

from dimos.robot.drone.type.odometry import Odometry

from dimos.simulation.mujoco.constants import (
    LAUNCHER_PATH,
    LIDAR_FPS,
    VIDEO_CAMERA_FOV,
    VIDEO_FPS,
    VIDEO_HEIGHT,
    VIDEO_WIDTH,
)

from dimos.simulation.mujoco.shared_memory import ShmWriter
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

