#!/usr/bin/env python3

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

import numpy as np
from dask.distributed import Client

client = Client(processes=False)


class ParameterServer:
    def __init__(self):
        self.data = dict()

    def put(self, key, value):
        self.data[key] = value

    def get(self, key):
        return self.data[key]


def train(params, lr=0.1):
    grad = 2 * (params - 1)  # gradient of (params - 1)**2
    new_params = params - lr * grad
    return new_params


ps_future = client.submit(ParameterServer, actor=True)
ps = ps_future.result()

ps.put("parameters", np.random.default_rng().random(1000))

print(ps.get("parameters").result())
for k in range(20):
    params = ps.get("parameters").result()
    new_params = train(params)
    ps.put("parameters", new_params)
    print(new_params.mean())
    # k=0: "0.5988202981316124"
    # k=10: "0.9569236575164062"
