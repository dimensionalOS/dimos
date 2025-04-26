import pytest
import numpy as np
from geometry_msgs.msg import Vector3
from dimos.types.vector import Vector, to_numpy, to_vector, to_tuple, to_list, is_2d, is_3d, x, y, z


class TestVectorInit:
    def test_init_with_components(self):
        v = Vector(1, 2, 3)
        assert np.array_equal(v.data, np.array([1.0, 2.0, 3.0]))
        
    def test_init_with_list(self):
        v = Vector([1, 2, 3])
        assert np.array_equal(v.data, np.array([1.0, 2.0, 3.0]))
        
    def test_init_with_numpy_array(self):
        v = Vector(np.array([1, 2, 3]))
        assert np.array_equal(v.data, np.array([1.0, 2.0, 3.0]))
        
    def test_init_with_vector3(self):
        v3 = Vector3()
        v3.x, v3.y, v3.z = 1.0, 2.0, 3.0
        v = Vector(v3)
        assert np.array_equal(v.data, np.array([1.0, 2.0, 3.0]))


class TestVectorProperties:
    def test_properties(self):
        v = Vector(1, 2, 3)
        assert v.x == 1.0
        assert v.y == 2.0
        assert v.z == 3.0
        assert v.dim == 3
        assert v.yaw == 1.0
        assert v.tuple == (1.0, 2.0, 3.0)
        
    def test_length_methods(self):
        v = Vector(3, 4, 0)
        assert v.length() == 5.0
        assert v.length_squared() == 25.0
        
    def test_conversion_methods(self):
        v = Vector(1, 2, 3)
        assert v.to_list() == [1.0, 2.0, 3.0]
        assert v.to_tuple() == (1.0, 2.0, 3.0)
        assert np.array_equal(v.to_numpy(), np.array([1.0, 2.0, 3.0]))


class TestVectorOperations:
    def test_basic_operations(self):
        v1 = Vector(1, 2, 3)
        v2 = Vector(4, 5, 6)
        
        # Addition
        assert v1 + v2 == Vector(5, 7, 9)
        assert v1 + [4, 5, 6] == Vector(5, 7, 9)
        
        # Subtraction
        assert v1 - v2 == Vector(-3, -3, -3)
        assert v1 - [4, 5, 6] == Vector(-3, -3, -3)
        
        # Scalar multiplication
        assert v1 * 2 == Vector(2, 4, 6)
        assert 2 * v1 == Vector(2, 4, 6)
        
        # Division
        assert v1 / 2 == Vector(0.5, 1.0, 1.5)
        
        # Negation
        assert -v1 == Vector(-1, -2, -3)
        
    def test_vector_operations(self):
        v1 = Vector(1, 2, 3)
        v2 = Vector(4, 5, 6)
        
        # Dot product
        assert v1.dot(v2) == 32
        assert v1.dot([4, 5, 6]) == 32
        
        # Cross product
        assert v1.cross(v2) == Vector(-3, 6, -3)
        assert v1.cross([4, 5, 6]) == Vector(-3, 6, -3)
        
        # Distance
        assert v1.distance(v2) == pytest.approx(5.196152)
        assert v1.distance_squared(v2) == 27.0
        
        # Angle
        assert v1.angle(v2) == pytest.approx(0.2257261)
        
    def test_cross_product_errors(self):
        v1 = Vector(1, 2)  # 2D vector
        v2 = Vector(3, 4, 5)  # 3D vector
        
        with pytest.raises(ValueError):
            v1.cross(v2)  # Should fail for 2D vector
            
        with pytest.raises(ValueError):
            v2.cross([1, 2])  # Should fail for mismatched dimensions


class TestVectorNormalization:
    def test_normalize(self):
        v = Vector(3, 4, 0)
        normalized = v.normalize()
        assert normalized == Vector(0.6, 0.8, 0.0)
        assert normalized.length() == pytest.approx(1.0)
        
    def test_normalize_zero_vector(self):
        v = Vector(0, 0, 0)
        normalized = v.normalize()
        assert normalized == Vector(0, 0, 0)


class TestVectorProjection:
    def test_projection(self):
        v = Vector(3, 3, 0)
        onto = Vector(1, 0, 0)  # x-axis
        projection = v.project(onto)
        assert projection == Vector(3, 0, 0)


class TestVectorClassMethods:
    def test_zeros(self):
        v = Vector.zeros(3)
        assert v == Vector(0, 0, 0)
        
    def test_ones(self):
        v = Vector.ones(2)
        assert v == Vector(1, 1)
        
    def test_unit_vectors(self):
        assert Vector.unit_x() == Vector(1, 0, 0)
        assert Vector.unit_y() == Vector(0, 1, 0)
        assert Vector.unit_z() == Vector(0, 0, 1)
        
        # Test with different dimensions
        assert Vector.unit_x(2) == Vector(1, 0)
        assert Vector.unit_y(2) == Vector(0, 1)
        assert Vector.unit_z(2) == Vector(0, 0)  # z component is 0 for 2D


class TestVectorConversionFunctions:
    def test_to_numpy(self):
        # Test Vector conversion
        v = Vector(1, 2, 3)
        assert np.array_equal(to_numpy(v), np.array([1.0, 2.0, 3.0]))
        
        # Test Vector3 conversion
        v3 = Vector3()
        v3.x, v3.y, v3.z = 1.0, 2.0, 3.0
        assert np.array_equal(to_numpy(v3), np.array([1.0, 2.0, 3.0]))
        
        # Test list conversion
        assert np.array_equal(to_numpy([1, 2, 3]), np.array([1.0, 2.0, 3.0]))
        
    def test_to_vector(self):
        assert to_vector([1, 2, 3]) == Vector(1, 2, 3)
        assert to_vector(Vector(1, 2, 3)) == Vector(1, 2, 3)
        
    def test_to_tuple(self):
        assert to_tuple(Vector(1, 2, 3)) == (1.0, 2.0, 3.0)
        assert to_tuple([1, 2, 3]) == (1.0, 2.0, 3.0)
        
    def test_to_list(self):
        assert to_list(Vector(1, 2, 3)) == [1.0, 2.0, 3.0]
        assert to_list((1, 2, 3)) == [1.0, 2.0, 3.0]


class TestVectorDimensionChecks:
    def test_is_2d(self):
        assert is_2d(Vector(1, 2)) is True
        assert is_2d(Vector(1, 2, 3)) is False
        assert is_2d([1, 2]) is True
        assert is_2d([1, 2, 3]) is False
        
    def test_is_3d(self):
        assert is_3d(Vector(1, 2, 3)) is True
        assert is_3d(Vector(1, 2)) is False
        assert is_3d([1, 2, 3]) is True
        assert is_3d([1, 2]) is False
        
        # Test Vector3
        v3 = Vector3()
        assert is_3d(v3) is True


class TestVectorComponentExtraction:
    def test_component_extraction(self):
        v = Vector(1, 2, 3)
        assert x(v) == 1.0
        assert y(v) == 2.0
        assert z(v) == 3.0
        
        # Test with list
        assert x([4, 5, 6]) == 4.0
        assert y([4, 5, 6]) == 5.0
        assert z([4, 5, 6]) == 6.0
        
        # Test with Vector3
        v3 = Vector3()
        v3.x, v3.y, v3.z = 7.0, 8.0, 9.0
        assert x(v3) == 7.0
        assert y(v3) == 8.0
        assert z(v3) == 9.0
        
        # Test with partial vector
        assert x([1]) == 1.0
        assert y([1]) == 0.0
        assert z([1]) == 0.0
