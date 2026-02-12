# Copyright 2025-2026 Dimensional Inc.
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

import pickle
import time

from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped


def test_lcm_encode_decode() -> None:
    """Test encoding and decoding of TwistStamped to/from binary LCM format."""
    twist_source = TwistStamped(
        ts=time.time(),
        linear=(1.0, 2.0, 3.0),
        angular=(0.1, 0.2, 0.3),
    )
    binary_msg = twist_source.lcm_encode()
    twist_dest = TwistStamped.lcm_decode(binary_msg)

    assert isinstance(twist_dest, TwistStamped)
    assert twist_dest is not twist_source

    print(twist_source.linear)
    print(twist_source.angular)

    print(twist_dest.linear)
    print(twist_dest.angular)
    assert twist_dest == twist_source


def test_pickle_encode_decode() -> None:
    """Test encoding and decoding of TwistStamped to/from binary pickle format."""

    twist_source = TwistStamped(
        ts=time.time(),
        linear=(1.0, 2.0, 3.0),
        angular=(0.1, 0.2, 0.3),
    )
    binary_msg = pickle.dumps(twist_source)
    twist_dest = pickle.loads(binary_msg)
    assert isinstance(twist_dest, TwistStamped)
    assert twist_dest is not twist_source
    assert twist_dest == twist_source


if __name__ == "__main__":
    print("Running test_lcm_encode_decode...")
    test_lcm_encode_decode()
    print("test_lcm_encode_decode passed")

    print("Running test_pickle_encode_decode...")
    test_pickle_encode_decode()
    print("test_pickle_encode_decode passed")

    print("\nAll tests passed!")
