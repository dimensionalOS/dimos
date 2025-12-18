import numpy as np
import pytest

from dimos.models.depth.metric3d import Metric3D
from dimos.msgs.sensor_msgs import Image
from dimos.utils.data import get_data


@pytest.fixture
def sample_intrinsics() -> list[float]:
    """Sample camera intrinsics [fx, fy, cx, cy]."""
    return [500.0, 500.0, 320.0, 240.0]


@pytest.mark.gpu
def test_metric3d_init(sample_intrinsics: list[float]) -> None:
    """Test Metric3D initialization."""
    model = Metric3D(camera_intrinsics=sample_intrinsics)
    assert model.config.camera_intrinsics == sample_intrinsics
    assert model.config.gt_depth_scale == 256.0
    assert model.device == "cuda"


@pytest.mark.gpu
def test_metric3d_update_intrinsic(sample_intrinsics: list[float]) -> None:
    """Test updating camera intrinsics."""
    model = Metric3D(camera_intrinsics=sample_intrinsics)

    new_intrinsics = [600.0, 600.0, 400.0, 300.0]
    model.update_intrinsic(new_intrinsics)
    assert model.intrinsic == new_intrinsics


@pytest.mark.gpu
def test_metric3d_update_intrinsic_invalid(sample_intrinsics: list[float]) -> None:
    """Test that invalid intrinsics raise an error."""
    model = Metric3D(camera_intrinsics=sample_intrinsics)

    with pytest.raises(ValueError, match="Intrinsic must be a list"):
        model.update_intrinsic([1.0, 2.0])  # Only 2 values


@pytest.mark.gpu
def test_metric3d_infer_depth(sample_intrinsics: list[float]) -> None:
    """Test depth inference on a sample image."""
    model = Metric3D(camera_intrinsics=sample_intrinsics)
    model.start()

    # Load test image
    image = Image.from_file(get_data("cafe.jpg")).to_rgb()
    rgb_array = image.data

    # Run inference
    depth_map = model.infer_depth(rgb_array)

    # Verify output
    assert isinstance(depth_map, np.ndarray)
    assert depth_map.shape[:2] == rgb_array.shape[:2]  # Same spatial dimensions
    assert depth_map.dtype in [np.float32, np.float64]
    assert depth_map.min() >= 0  # Depth should be non-negative

    print(f"Depth map shape: {depth_map.shape}")
    print(f"Depth range: [{depth_map.min():.2f}, {depth_map.max():.2f}]")

    model.stop()


@pytest.mark.gpu
def test_metric3d_multiple_inferences(sample_intrinsics: list[float]) -> None:
    """Test multiple depth inferences."""
    model = Metric3D(camera_intrinsics=sample_intrinsics)
    model.start()

    image = Image.from_file(get_data("cafe.jpg")).to_rgb()
    rgb_array = image.data

    # Run multiple inferences
    depths = []
    for _ in range(3):
        depth = model.infer_depth(rgb_array)
        depths.append(depth)

    # Results should be consistent
    for i in range(1, len(depths)):
        assert np.allclose(depths[0], depths[i], rtol=1e-5)

    model.stop()
