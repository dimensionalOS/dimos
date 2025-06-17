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

import time

import lcm
from lcm_msgs.geometry_msgs import Twist, Vector3

msg2 = Vector3()
msg2.x = 1
msg2.y = 1
msg2.z = 1
msg3 = Twist()
msg3.linear = Vector3()
msg3.linear.x = 1
msg3.linear.y = 1
msg3.linear.z = 1

lc = lcm.LCM()
lc.publish("thing1_vector3#geometry_msgs.Vector3", msg2.encode())
lc.publish("thing1_twist#geometry_msgs.Twist", msg3.encode())

while True:
    msg2.x += 1
    msg2.y += 1
    msg2.z += 1
    lc.publish("thing1_vector3#geometry_msgs.Vector3", msg2.encode())
    msg3.linear.x += 1
    msg3.linear.y += 1
    msg3.linear.z += 1
    lc.publish("thing1_twist#geometry_msgs.Twist", msg3.encode())
    time.sleep(0.1)
    print("pub")
