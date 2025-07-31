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
import pytest

from dimos.robot.sim import minecraft


def test_action_creation():
    """Test Action creation methods."""
    # Test no-op creation
    action = minecraft.Action()
    expected = np.array([0, 0, 0, 12, 12, 0, 0, 0], dtype=np.int32)
    assert np.array_equal(action.array, expected)

    # Test from array
    arr = np.array([1, 2, 0, 12, 12, 0, 0, 0], dtype=np.int32)
    action = minecraft.Action(arr)
    assert np.array_equal(action.array, arr)

    # Test classmethod
    action = minecraft.Action.noop()
    expected = np.array([0, 0, 0, 12, 12, 0, 0, 0], dtype=np.int32)
    assert np.array_equal(action.array, expected)


def test_movement_properties():
    """Test movement properties (forward/back, left/right)."""
    action = minecraft.Action()

    # Test forward/back boolean setters
    action.forward = True
    assert action.array[0] == 1
    assert action.forward
    assert not action.backward

    action.backward = True
    assert action.array[0] == 2
    assert action.backward
    assert not action.forward

    action.forward = False
    assert action.array[0] == 0
    assert not action.forward
    assert not action.backward

    # Test left/right boolean setters
    action.left = True
    assert action.array[1] == 1
    assert action.left
    assert not action.right

    action.right = True
    assert action.array[1] == 2
    assert action.right
    assert not action.left

    action.left = False
    assert action.array[1] == 0
    assert not action.left
    assert not action.right


def test_jump_sneak_sprint():
    """Test jump/sneak/sprint properties."""
    action = minecraft.Action()

    # Only one can be active at a time
    action.jump = True
    assert action.array[2] == 1
    assert action.jump
    assert not action.sneak
    assert not action.sprint

    action.sneak = True
    assert action.array[2] == 2
    assert not action.jump
    assert action.sneak
    assert not action.sprint

    action.sprint = True
    assert action.array[2] == 3
    assert not action.jump
    assert not action.sneak
    assert action.sprint


def test_camera_properties():
    """Test camera pitch and yaw properties."""
    action = minecraft.Action()

    # Test pitch
    action.camera_pitch = 0
    assert action.array[3] == 12  # Middle value
    assert abs(action.camera_pitch) < 0.1

    action.camera_pitch = 180
    assert action.array[3] == 24
    assert abs(action.camera_pitch - 180) < 1

    action.camera_pitch = -180
    assert action.array[3] == 0
    assert abs(action.camera_pitch + 180) < 1

    # Test yaw
    action.camera_yaw = 0
    assert action.array[4] == 12
    assert abs(action.camera_yaw) < 0.1

    action.camera_yaw = 90
    assert action.array[4] == 18
    assert abs(action.camera_yaw - 90) < 1

    action.camera_yaw = -90
    assert action.array[4] == 6
    assert abs(action.camera_yaw + 90) < 1


def test_functional_actions():
    """Test functional action properties."""
    action = minecraft.Action()

    # Test use
    action.use = True
    assert action.array[5] == 1
    assert action.use

    # Test attack
    action.attack = True
    assert action.array[5] == 3
    assert action.attack
    assert not action.use  # Only one functional action at a time

    # Test drop
    action.drop = True
    assert action.array[5] == 2
    assert action.drop

    # Test craft
    action.craft(100)
    assert action.array[5] == 4
    assert action.array[6] == 100

    # Test craft with clipping
    action.craft(300)
    assert action.array[6] == 243  # Clipped to max

    # Test equip
    action.equip(10)
    assert action.array[5] == 5
    assert action.array[7] == 10

    # Test place
    action.place(20)
    assert action.array[5] == 6
    assert action.array[7] == 20

    # Test destroy
    action.destroy(35)
    assert action.array[5] == 7
    assert action.array[7] == 35

    # Test destroy with clipping
    action.destroy(50)
    assert action.array[7] == 35  # Clipped to max


def test_action_string_representation():
    """Test string representation of actions."""
    # Test noop
    action = minecraft.Action()
    assert str(action) == "Action(noop)"

    # Test movement
    action.forward = True
    action.left = True
    assert str(action) == "Action(forward, left)"

    # Test with camera
    action = minecraft.Action()
    action.backward = True
    action.camera_yaw = 45
    assert str(action) == "Action(back, yaw=45.0°)"

    # Test functional actions
    action = minecraft.Action()
    action.attack = True
    assert str(action) == "Action(attack)"

    action = minecraft.Action()
    action.craft(50)
    assert str(action) == "Action(craft[50])"

    # Test complex action
    action = minecraft.Action()
    action.forward = 1
    action.jump = True
    action.camera_pitch = -30
    action.use = True
    assert "forward" in str(action)
    assert "jump" in str(action)
    assert "pitch=-30.0°" in str(action)
    assert "use" in str(action)


def test_xyz_movement():
    """Test x, y, z movement setters."""
    action = minecraft.Action()

    # Test x (forward/back)
    action.x = 1
    assert action.array[0] == 1
    assert action.x == 1
    assert action.forward

    action.x = -1
    assert action.array[0] == 2
    assert action.x == -1
    assert action.backward

    action.x = 0
    assert action.array[0] == 0
    assert action.x == 0

    # Test y (left/right)
    action.y = -1  # left
    assert action.array[1] == 1
    assert action.y == -1
    assert action.left

    action.y = 1  # right
    assert action.array[1] == 2
    assert action.y == 1
    assert action.right

    action.y = 0
    assert action.array[1] == 0
    assert action.y == 0

    # Test z (jump/sneak)
    action.z = 1  # jump
    assert action.array[2] == 1
    assert action.z == 1
    assert action.jump

    action.z = -1  # sneak
    assert action.array[2] == 2
    assert action.z == -1
    assert action.sneak

    action.z = 0
    assert action.array[2] == 0
    assert action.z == 0


def test_action_bounds():
    """Test that action values stay within valid bounds."""
    action = minecraft.Action()

    # Test camera bounds
    action.camera_pitch = 1000  # Should clip to 180
    assert action.array[3] == 24

    action.camera_pitch = -1000  # Should clip to -180
    assert action.array[3] == 0

    # Test slot bounds
    action.equip(1000)
    assert action.array[7] == 35

    action.place(-10)
    assert action.array[7] == 0

    # Test craft bounds
    action.craft(1000)
    assert action.array[6] == 243

    action.craft(-10)
    assert action.array[6] == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
