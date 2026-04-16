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

"""Rerun-based robot visualizer using Pinocchio for kinematics."""

from __future__ import annotations

from pathlib import Path

import pinocchio
import rerun as rr

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Try to use hpp-fcl types if available (usually bundled with pinocchio)
try:
    import hppfcl
except ImportError:
    hppfcl = None


class RobotVisualizer:
    """Logs a robot URDF and updates its joints in Rerun.

    Uses Pinocchio to parse the URDF and compute forward kinematics for
    nested entity path transforms in Rerun.
    """

    def __init__(
        self,
        urdf_path: str | Path,
        entity_path: str = "world/robot",
        package_paths: dict[str, Path] | None = None,
        xacro_args: dict[str, str] | None = None,
    ) -> None:
        """Initialize the visualizer.

        Args:
            urdf_path: Path to URDF or xacro file.
            entity_path: Rerun entity path for the robot root.
            package_paths: Mapping of package names to filesystem paths for meshes.
            xacro_args: Arguments for xacro processing.
        """
        self.urdf_path = Path(urdf_path)
        self.root_entity_path = entity_path

        # Prepare URDF (resolves xacro and package:// URIs, converts DAE/STL to OBJ)
        from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake

        self.resolved_urdf = prepare_urdf_for_drake(
            self.urdf_path, package_paths=package_paths, xacro_args=xacro_args,
            convert_meshes=True
        )

        # Load Pinocchio model
        try:
            self.model = pinocchio.buildModelFromUrdf(self.resolved_urdf)
            self.visual_model = pinocchio.buildGeomFromUrdf(
                self.model, self.resolved_urdf, pinocchio.VISUAL
            )
        except Exception as e:
            logger.error(f"Failed to load URDF with Pinocchio: {e}")
            raise

        self.data = self.model.createData()

        # Map Pinocchio joint IDs to Rerun entity paths
        self.joint_to_entity: dict[int, str] = {}
        for i, name in enumerate(self.model.names):
            if i == 0:  # Universe
                self.joint_to_entity[i] = self.root_entity_path
            else:
                parent_id = self.model.parents[i]
                parent_path = self.joint_to_entity[parent_id]
                # Avoid "universe" in the path if possible, but Pinocchio joint 0 is universe
                self.joint_to_entity[i] = f"{parent_path}/{name}"

        self._log_static_visuals()

    def _log_static_visuals(self) -> None:
        """Log visual geometries to Rerun once at startup."""
        for i, geom in enumerate(self.visual_model.geometryObjects):
            joint_id = geom.parentJoint
            # Use geometry name if unique, otherwise append index
            geom_name = geom.name or f"visual_{i}"
            entity_path = f"{self.joint_to_entity[joint_id]}/visuals/{geom_name}"

            # Log the placement relative to the joint
            pos = geom.placement.translation
            quat = pinocchio.Quaternion(geom.placement.rotation)
            rr.log(
                entity_path,
                rr.Transform3D(
                    translation=pos,
                    rotation=rr.Quaternion(xyzw=[quat.x, quat.y, quat.z, quat.w]),
                ),
                static=True,
            )

            # Log the geometry itself
            self._log_geometry(entity_path, geom)

    def _log_geometry(self, entity_path: str, geom: pinocchio.GeometryObject) -> None:
        """Log a single geometry object to Rerun."""
        # Convert Pinocchio's 4-float rgba (0-1) to Rerun's 4-uint8 rgba
        color = None
        if hasattr(geom, "meshColor") and geom.meshColor is not None:
            # pinocchio 3.x uses meshColor for rgba
            color = [int(x * 255) for x in geom.meshColor]
        # Some versions might use overrideMaterial or similar, but meshColor is common

        if hppfcl and isinstance(geom.geometry, hppfcl.Box):
            half_size = geom.geometry.halfSide
            rr.log(entity_path, rr.Boxes3D(half_sizes=[half_size], colors=[color] if color else None), static=True)
        elif hppfcl and isinstance(geom.geometry, hppfcl.Sphere):
            # Using half_sizes as a list-of-vectors to log a single sphere correctly
            radius = geom.geometry.radius
            rr.log(
                entity_path,
                rr.Ellipsoids3D(
                    half_sizes=[[radius] * 3],
                    colors=[color] if color else None
                ),
                static=True
            )
        elif hppfcl and isinstance(geom.geometry, hppfcl.Cylinder):
            # Cylinder not directly supported, skip or log debug
            logger.debug(f"Cylinder primitive not yet supported in Rerun logging for {entity_path}")
        elif hasattr(geom, "meshPath") and geom.meshPath and not geom.meshPath == "BOX":
            # Log as Asset3D
            mesh_path = Path(geom.meshPath)
            if mesh_path.exists():
                rr.log(entity_path, rr.Asset3D(path=mesh_path), static=True)
            else:
                logger.warning(f"Mesh file not found: {mesh_path} for {entity_path}")

    def update_joints(self, joint_state: JointState) -> None:
        """Update joint positions and log new transforms to Rerun.

        Args:
            joint_state: The current joint state.
        """
        # Build configuration vector q
        q = pinocchio.neutral(self.model)
        
        # JointState might have fewer joints than the model, or in different order
        for name, pos in zip(joint_state.name, joint_state.position):
            joint_id = -1
            # 1. Try exact match
            if self.model.existJointName(name):
                joint_id = self.model.getJointId(name)
            # 2. Try stripping prefixes (e.g., arm/joint1 -> joint1, arm_joint1 -> joint1)
            else:
                for sep in ["/", "_"]:
                    if sep in name:
                        base_name = name.split(sep, 1)[-1]
                        if self.model.existJointName(base_name):
                            joint_id = self.model.getJointId(base_name)
                            break
            
            if joint_id == -1:
                continue

            # For 1-DOF joints (revolute, prismatic), idx_q is the index in q
            idx = self.model.joints[joint_id].idx_q
            if idx >= 0:
                q[idx] = pos

        # Compute FK (we need liMi for local transforms)
        pinocchio.forwardKinematics(self.model, self.data, q)

        # Log local transforms to Rerun
        # Skip joint 0 (universe)
        for i in range(1, self.model.njoints):
            entity_path = self.joint_to_entity[i]
            # liMi is the transform from parent to child joint
            m = self.data.liMi[i]
            pos = m.translation
            quat = pinocchio.Quaternion(m.rotation)
            rr.log(
                entity_path,
                rr.Transform3D(
                    translation=pos,
                    rotation=rr.Quaternion(xyzw=[quat.x, quat.y, quat.z, quat.w]),
                ),
            )
