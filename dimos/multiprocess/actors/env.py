# Copyright 2025 Dimensional Inc.
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

import cv2
from dask.distributed import get_worker


def env():
    worker = get_worker()
    if worker:
        if not hasattr(worker, "env"):
            worker.env = {}
        return worker.env
    else:
        if not globals().get("env"):
            globals()["env"] = {}
        return globals()["env"]


def getenv(name: str, default_value: any = None):
    val = env().get(name)
    if val:
        return val

    print("Environment variable not set:", name)
    # check if default_value is function
    if callable(default_value):
        print("Using F default value for", name)
        val = default_value()

    print("Setting default value for", name, ":", val)
    env()[name] = val
    return val
