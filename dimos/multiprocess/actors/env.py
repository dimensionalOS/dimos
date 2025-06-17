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
import threading

from dask.distributed import get_worker


def env():
    worker = get_worker()
    if worker:
        print("Using worker environment", worker)
        if not hasattr(worker, "env"):
            worker.env = {}
        return worker.env
    else:
        print("Using global environment")
        if not globals().get("env"):
            globals()["env"] = {}
        return globals()["env"]


def getenv(name: str, default_value=None):
    e = env()
    val = e.get(name)
    if val:
        if isinstance(val, threading.Event):
            print("Event found, waiting for it to be released")
            val.wait()
            return e.get(name)
        return val

    print("Environment variable not set:", name)

    # Only use locking for expensive callable functions
    if callable(default_value):
        lock = threading.Event()
        e[name] = lock

        try:
            print("Using F default value for", name)
            computed_val = default_value()
            print("Setting default value for", name, ":", computed_val)
            e[name] = computed_val
            lock.set()
            return computed_val
        except Exception as e:
            # Clean up on failure
            e.pop(name, None)
            lock.set()
            raise e
    else:
        # For non-callable defaults, just set directly (no locking needed)
        print("Setting default value for", name, ":", default_value)
        env()[name] = default_value
        return default_value
