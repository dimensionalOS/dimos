from dimos.types.position import Position
from dimos.types.vector import Vector
import numpy as np


def test_position_default_init():
    """Test that default initialization of Position() has zero vectors for pos and rot."""
    position = Position()

    # Check that pos is a zero vector
    assert isinstance(position.pos, Vector)
    assert position.pos.is_zero()
    assert position.pos.x == 0.0
    assert position.pos.y == 0.0
    assert position.pos.z == 0.0

    # Check that rot is a zero vector
    assert isinstance(position.rot, Vector)
    assert position.rot.is_zero()
    assert position.rot.x == 0.0
    assert position.rot.y == 0.0
    assert position.rot.z == 0.0

    assert position.is_zero()

    assert not position


def test_position_vector_init():
    """Test initialization with custom vectors."""
    pos = Vector(1.0, 2.0, 3.0)
    rot = Vector(4.0, 5.0, 6.0)

    position = Position(pos, rot)

    # Check pos vector
    assert position.pos is pos
    assert position.pos.x == 1.0
    assert position.pos.y == 2.0
    assert position.pos.z == 3.0

    # Check rot vector
    assert position.rot is rot
    assert position.rot.x == 4.0
    assert position.rot.y == 5.0
    assert position.rot.z == 6.0


def test_position_partial_init():
    """Test initialization with only one custom vector."""
    pos = Vector(1.0, 2.0, 3.0)
    assert pos

    # Only specify pos
    position1 = Position(pos)
    assert position1.pos is pos
    assert position1.pos.x == 1.0
    assert position1.pos.y == 2.0
    assert position1.pos.z == 3.0
    assert not position1.pos.is_zero()

    assert isinstance(position1.rot, Vector)
    assert position1.rot.is_zero()
    assert position1.rot.x == 0.0
    assert position1.rot.y == 0.0
    assert position1.rot.z == 0.0

    # Only specify rot
    rot = Vector(4.0, 5.0, 6.0)
    assert not rot.is_zero()
    position2 = Position(rot=rot)
    assert isinstance(position2.pos, Vector)
    assert not position2.pos

    assert position2.rot is rot
    assert position2.rot.x == 4.0
    assert position2.rot.y == 5.0
    assert position2.rot.z == 6.0
    assert position2.rot

