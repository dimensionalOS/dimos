import numpy as np
from numpy.typing import NDArray

class VoxelRayMap:
    """Voxel map with raycast clearing of dynamic objects."""

    def __init__(
        self,
        *,
        voxel_size: float,
        max_range: float,
        ray_subsample: int = 1,
        shadow_depth: float = 0.2,
        grace_depth: float = 0.2,
        min_health: int = -2,
        max_health: int = 1,
    ) -> None: ...
    def add_frame(
        self,
        points: NDArray[np.float32],
        origin: tuple[float, float, float],
    ) -> None:
        """Update the map with a frame of lidar points. Shape (N, 3) float32."""
        ...

    def global_map(self) -> NDArray[np.float32]:
        """Return the centers of all healthy voxels as (M, 3) float32."""
        ...

    def local_map(
        self,
        origin: tuple[float, float, float],
        radius: float,
        z_min: float,
        z_max: float,
    ) -> NDArray[np.float32]:
        """Return healthy voxels inside the cylinder around origin as (M, 3) float32."""
        ...

    def voxel_count(self) -> int:
        """Number of healthy voxels currently in the map."""
        ...

    def clear(self) -> None:
        """Reset the map to empty."""
        ...

    def __len__(self) -> int: ...
    def __repr__(self) -> str: ...

__all__ = ["VoxelRayMap"]
