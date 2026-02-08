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

"""
Mesh Utilities for Drake

Provides utilities for preparing URDF files for use with Drake:
- Xacro processing
- Mesh format conversion (DAE/STL to OBJ)
- Package path resolution

Example:
    urdf_path = prepare_urdf_for_drake(
        urdf_path="/path/to/robot.xacro",
        package_paths={"robot_description": "/path/to/robot_description"},
        xacro_args={"use_sim": "true"},
        convert_meshes=True,
    )
"""

from __future__ import annotations

import hashlib
from pathlib import Path
import re
import shutil
import tempfile
from typing import TYPE_CHECKING

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

logger = setup_logger()

# Cache directory for processed URDFs
_CACHE_DIR = Path(tempfile.gettempdir()) / "dimos_urdf_cache"


def prepare_urdf_for_drake(
    urdf_path: Path | str,
    package_paths: dict[str, Path] | None = None,
    xacro_args: dict[str, str] | None = None,
    convert_meshes: bool = False,
) -> str:
    """Prepare a URDF/xacro file for use with Drake.

    This function:
    1. Processes xacro files if needed
    2. Resolves package:// URIs in mesh paths
    3. Optionally converts DAE/STL meshes to OBJ format

    Args:
        urdf_path: Path to URDF or xacro file
        package_paths: Dict mapping package names to filesystem paths
        xacro_args: Arguments to pass to xacro processor
        convert_meshes: Convert DAE/STL meshes to OBJ for Drake compatibility

    Returns:
        Path to the prepared URDF file (may be cached)
    """
    urdf_path = Path(urdf_path)
    package_paths = package_paths or {}
    xacro_args = xacro_args or {}

    # Generate cache key
    cache_key = _generate_cache_key(urdf_path, package_paths, xacro_args, convert_meshes)
    cache_path = _CACHE_DIR / cache_key / urdf_path.stem
    cache_path.mkdir(parents=True, exist_ok=True)
    cached_urdf = cache_path / f"{urdf_path.stem}.urdf"

    # Check cache
    if cached_urdf.exists():
        logger.debug(f"Using cached URDF: {cached_urdf}")
        return str(cached_urdf)

    # Process xacro if needed
    if urdf_path.suffix in (".xacro", ".urdf.xacro"):
        urdf_content = _process_xacro(urdf_path, package_paths, xacro_args)
    else:
        urdf_content = urdf_path.read_text()

    # Strip transmission blocks (Drake doesn't need them, and they can cause issues)
    urdf_content = _strip_transmission_blocks(urdf_content)

    # Resolve package:// URIs
    urdf_content = _resolve_package_uris(urdf_content, package_paths, cache_path)

    # Convert meshes if requested
    if convert_meshes:
        urdf_content = _convert_meshes(urdf_content, cache_path)

    # Write processed URDF
    cached_urdf.write_text(urdf_content)
    logger.info(f"Prepared URDF cached at: {cached_urdf}")

    return str(cached_urdf)


def _generate_cache_key(
    urdf_path: Path,
    package_paths: dict[str, Path],
    xacro_args: dict[str, str],
    convert_meshes: bool,
) -> str:
    """Generate a cache key for the URDF configuration.

    Includes a version number to invalidate cache when processing logic changes.
    """
    # Include file modification time
    mtime = urdf_path.stat().st_mtime if urdf_path.exists() else 0

    # Version number to invalidate cache when processing logic changes
    # Increment this when adding new processing steps (e.g., stripping transmission blocks)
    processing_version = "v2"

    key_data = f"{processing_version}:{urdf_path}:{mtime}:{sorted(package_paths.items())}:{sorted(xacro_args.items())}:{convert_meshes}"
    return hashlib.md5(key_data.encode()).hexdigest()[:16]


def _process_xacro(
    xacro_path: Path,
    package_paths: dict[str, Path],
    xacro_args: dict[str, str],
) -> str:
    """Process xacro file to URDF."""
    try:
        import xacro  # type: ignore[import-not-found,import-untyped]
    except ImportError:
        raise ImportError(
            "xacro is required for processing .xacro files. Install with: pip install xacro"
        )

    # Create a custom substitution_args_context that resolves $(find pkg) to our paths
    # This avoids requiring ROS package discovery
    from xacro import substitution_args

    # Store original function
    original_find = substitution_args._find

    def custom_find(resolved: str, a: str, args: list[str], context: dict[str, str]) -> str:
        """Custom $(find pkg) handler that uses our package_paths."""
        pkg_name = args[0] if args else ""
        if pkg_name in package_paths:
            pkg_path = str(Path(package_paths[pkg_name]).resolve())
            return resolved.replace(f"$({a})", pkg_path)
        # Fall back to original behavior
        return str(original_find(resolved, a, args, context))

    # Monkey-patch the find function temporarily
    substitution_args._find = custom_find

    try:
        # Process xacro with our mappings
        doc = xacro.process_file(
            str(xacro_path),
            mappings=xacro_args,
        )
        return str(doc.toprettyxml(indent="  "))
    finally:
        # Restore original function
        substitution_args._find = original_find


def _strip_transmission_blocks(urdf_content: str) -> str:
    """Remove transmission blocks from URDF content.

    Drake doesn't need transmission blocks (they're for Gazebo/ROS control),
    and they can cause parsing errors if they contain malformed actuator names.

    Args:
        urdf_content: URDF XML content as string

    Returns:
        URDF content with transmission blocks removed
    """
    # Pattern to match <transmission>...</transmission> blocks and self-closing <transmission/>
    # Uses non-greedy matching and handles nested tags
    pattern = r"<transmission[^>]*(?:/>|>.*?</transmission>)"

    # Remove transmission blocks (with flags for multiline and dotall)
    result = re.sub(pattern, "", urdf_content, flags=re.DOTALL | re.MULTILINE)

    # Also remove any standalone <gazebo> blocks that might reference transmissions
    # (some URDFs have gazebo plugins that reference transmissions)
    gazebo_pattern = r"<gazebo>.*?<plugin[^>]*gazebo_ros_control[^>]*>.*?</plugin>.*?</gazebo>"
    result = re.sub(gazebo_pattern, "", result, flags=re.DOTALL | re.MULTILINE)

    return result


def _resolve_package_uris(
    urdf_content: str,
    package_paths: dict[str, Path],
    output_dir: Path,
) -> str:
    """Resolve package:// URIs to filesystem paths."""
    # Pattern for package:// URIs (handles both single and double quotes)
    # Note: Use triple quotes so \s is correctly interpreted as whitespace, not literal 's'
    pattern = r"""package://([^/]+)/(.+?)(["'<>\s])"""

    def replace_uri(match: re.Match[str]) -> str:
        pkg_name = match.group(1)
        rel_path = match.group(2)
        suffix = match.group(3)

        if pkg_name in package_paths:
            # Ensure absolute path for proper resolution
            pkg_path = Path(package_paths[pkg_name]).resolve()
            full_path = pkg_path / rel_path
            if full_path.exists():
                return f"{full_path}{suffix}"
            else:
                logger.warning(f"File not found: {full_path}")

        # Return original if not found
        return match.group(0)

    return re.sub(pattern, replace_uri, urdf_content)


def _convert_meshes(urdf_content: str, output_dir: Path) -> str:
    """Convert DAE/STL meshes to OBJ format for Drake compatibility."""
    try:
        import trimesh
    except ImportError:
        logger.warning("trimesh not installed, skipping mesh conversion")
        return urdf_content

    mesh_dir = output_dir / "meshes"
    mesh_dir.mkdir(exist_ok=True)

    # Find mesh file references
    pattern = r'filename="([^"]+\.(dae|stl|DAE|STL))"'

    converted: dict[str, str] = {}

    def convert_mesh(match: re.Match[str]) -> str:
        original_path = match.group(1)

        if original_path in converted:
            return f'filename="{converted[original_path]}"'

        try:
            # Load mesh
            mesh = trimesh.load(original_path, force="mesh")

            # Generate output path
            mesh_name = Path(original_path).stem
            obj_path = mesh_dir / f"{mesh_name}.obj"

            # Export as OBJ (trimesh.export returns None, ignore)
            mesh.export(str(obj_path), file_type="obj")  # type: ignore[no-untyped-call]
            logger.debug(f"Converted mesh: {original_path} -> {obj_path}")

            converted[original_path] = str(obj_path)
            return f'filename="{obj_path}"'

        except Exception as e:
            logger.warning(f"Failed to convert mesh {original_path}: {e}")
            return match.group(0)

    return re.sub(pattern, convert_mesh, urdf_content)


def pointcloud_to_convex_hull_obj(
    points: NDArray[np.float64],
    output_path: Path | str | None = None,
    *,
    voxel_size: float = 0.005,
    min_points: int = 4,
) -> str | None:
    """Compute convex hull from point cloud and save as OBJ file.

    Points are centered at origin so the mesh is in local frame.
    The caller sets the obstacle pose to place it in the world.

    Args:
        points: Nx3 numpy array of 3D points (world frame)
        output_path: Where to save OBJ. If None, uses a temp file.
        voxel_size: Downsample voxel size in meters (0 to skip)
        min_points: Minimum points required for convex hull

    Returns:
        Path to OBJ file, or None if hull computation fails
    """
    import numpy as np

    if points.shape[0] < min_points:
        logger.warning(f"Too few points ({points.shape[0]}) for convex hull")
        return None

    try:
        import open3d as o3d  # type: ignore[import-untyped]
    except ImportError:
        logger.warning("open3d not installed, cannot compute convex hull")
        return None

    # Center at origin so mesh is in local frame
    centered = points - points.mean(axis=0)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(centered.astype(np.float64))

    if voxel_size > 0 and len(pcd.points) > 100:
        pcd = pcd.voxel_down_sample(voxel_size)

    if len(pcd.points) < min_points:
        logger.warning(f"Too few points after downsample ({len(pcd.points)})")
        return None

    try:
        hull, _ = pcd.compute_convex_hull()
    except Exception as e:
        logger.warning(f"Convex hull computation failed: {e}")
        return None

    if output_path is None:
        hull_dir = _CACHE_DIR / "convex_hulls"
        hull_dir.mkdir(parents=True, exist_ok=True)
        output_path = hull_dir / f"hull_{id(points):x}.obj"

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        o3d.io.write_triangle_mesh(str(output_path), hull)
        logger.debug(
            f"Convex hull: {len(hull.vertices)} verts, {len(hull.triangles)} faces -> {output_path}"
        )
        return str(output_path)
    except Exception as e:
        logger.warning(f"Failed to write convex hull OBJ: {e}")
        return None


def clear_cache() -> None:
    """Clear the URDF cache directory."""
    if _CACHE_DIR.exists():
        shutil.rmtree(_CACHE_DIR)
        logger.info(f"Cleared URDF cache: {_CACHE_DIR}")
