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


def test_position_vector_init():
    """Test initialization with custom vectors."""
    pos = Vector(1.0, 2.0, 3.0)
    rot = Vector(4.0, 5.0, 6.0)
    position = Position(pos=pos, rot=rot)

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
    assert not pos.is_zero()

    # Only specify pos
    position1 = Position(pos=pos)
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
    assert position2.pos.is_zero()
    assert position2.pos.x == 0.0
    assert position2.pos.y == 0.0
    assert position2.pos.z == 0.0

    assert position2.rot is rot
    assert position2.rot.x == 4.0
    assert position2.rot.y == 5.0
    assert position2.rot.z == 6.0
    assert not position2.rot.is_zero()


def test_position_repr_str():
    """Test the string representation of Position."""
    pos = Vector(1.0, 2.0, 3.0)
    rot = Vector(4.0, 5.0, 6.0)
    position = Position(pos=pos, rot=rot)

    # Test __repr__
    expected_repr = f"pos({pos}), rot({rot})"
    assert repr(position) == expected_repr

    # Test __str__ (should be the same as __repr__)
    assert str(position) == expected_repr


def test_position_with_zero_vectors():
    """Test Position with zero and non-zero vectors."""
    # Both vectors are zero
    position1 = Position()
    assert position1.pos.is_zero()
    assert position1.rot.is_zero()

    # Only pos is zero
    position2 = Position(rot=Vector(1.0, 2.0, 3.0))
    assert position2.pos.is_zero()
    assert not position2.rot.is_zero()

    # Only rot is zero
    position3 = Position(pos=Vector(4.0, 5.0, 6.0))
    assert not position3.pos.is_zero()
    assert position3.rot.is_zero()

    # Neither vector is zero
    position4 = Position(pos=Vector(7.0, 8.0, 9.0), rot=Vector(10.0, 11.0, 12.0))
    assert not position4.pos.is_zero()
    assert not position4.rot.is_zero()


def test_position_is_zero():
    """Test the is_zero method of Position class."""
    # Position with both vectors zero
    position1 = Position()
    assert position1.is_zero()

    # Position with only pos zero
    position2 = Position(rot=Vector(1.0, 2.0, 3.0))
    assert not position2.is_zero()

    # Position with only rot zero
    position3 = Position(pos=Vector(4.0, 5.0, 6.0))
    assert not position3.is_zero()

    # Position with neither vector zero
    position4 = Position(pos=Vector(7.0, 8.0, 9.0), rot=Vector(10.0, 11.0, 12.0))
    assert not position4.is_zero()

    # Position with almost-zero vectors (within tolerance)
    position5 = Position(pos=Vector(1e-10, 1e-10, 1e-10), rot=Vector(1e-10, 1e-10, 1e-10))
    assert position5.is_zero()

    # Position with one almost-zero vector and one actual zero
    position6 = Position(pos=Vector(1e-10, 1e-10, 1e-10), rot=Vector())
    assert position6.is_zero()


def test_position_bool_conversion():
    """Test boolean conversion of positions."""
    # Position with both vectors zero should be False
    position1 = Position()
    assert not position1

    # Position with only pos zero should be True
    position2 = Position(rot=Vector(1.0, 2.0, 3.0))
    assert position2

    # Position with only rot zero should be True
    position3 = Position(pos=Vector(4.0, 5.0, 6.0))
    assert bool(position3)

    # Position with neither vector zero should be True
    position4 = Position(pos=Vector(7.0, 8.0, 9.0), rot=Vector(10.0, 11.0, 12.0))
    assert bool(position4)

    # Position with almost-zero vectors should be False
    position5 = Position(pos=Vector(1e-10, 1e-10, 1e-10), rot=Vector(1e-10, 1e-10, 1e-10))
    assert not position5

    # Direct use in if statements
    if position1:
        assert False, "Zero position should be False in boolean context"
    else:
        pass  # Expected path

    if position4:
        pass  # Expected path
    else:
        assert False, "Non-zero position should be True in boolean context"


def test_position_custom_init():
    position = Position(pos=[1, 2, 3], rot=np.array([4, 5, 6]))

    # Check pos vector
    assert position.pos.x == 1.0
    assert position.pos.y == 2.0
    assert position.pos.z == 3.0

    # Check rot vector
    assert position.rot.x == 4.0
    assert position.rot.y == 5.0
    assert position.rot.z == 6.0
