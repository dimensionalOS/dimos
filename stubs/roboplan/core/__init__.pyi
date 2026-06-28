"""Partial stubs matching RoboPlan's bundled pybind stubs."""

from collections.abc import Sequence
import os

import numpy as np
from numpy.typing import NDArray

__version__: str

class JointConfiguration:
    def __init__(
        self,
        joint_names: Sequence[str] = ...,
        positions: NDArray[np.float64] = ...,
    ) -> None: ...
    @property
    def joint_names(self) -> list[str]: ...
    @joint_names.setter
    def joint_names(self, arg: Sequence[str], /) -> None: ...
    @property
    def positions(self) -> NDArray[np.float64]: ...
    @positions.setter
    def positions(self, arg: NDArray[np.float64], /) -> None: ...

class JointGroupInfo:
    @property
    def joint_names(self) -> list[str]: ...

class JointPath:
    @property
    def joint_names(self) -> list[str]: ...
    @property
    def positions(self) -> list[NDArray[np.float64]]: ...

class Box:
    def __init__(self, x: float, y: float, z: float) -> None: ...

class Sphere:
    def __init__(self, radius: float) -> None: ...

class Cylinder:
    def __init__(self, radius: float, length: float) -> None: ...

class Mesh:
    def __init__(
        self,
        filename: str | os.PathLike[str],
        scale: NDArray[np.float64] = ...,
    ) -> None: ...

class Scene:
    def __init__(
        self,
        name: str,
        urdf_path: str | os.PathLike[str],
        srdf_path: str | os.PathLike[str],
        package_paths: Sequence[str | os.PathLike[str]] = ...,
        yaml_config_path: str | os.PathLike[str] = ...,
    ) -> None: ...
    def hasCollisions(self, q: NDArray[np.float64], debug: bool = ...) -> bool: ...
    def toFullJointPositions(
        self, group_name: str, q: NDArray[np.float64]
    ) -> NDArray[np.float64]: ...
    def forwardKinematics(
        self, q: NDArray[np.float64], frame_name: str, base_frame: str = ...
    ) -> NDArray[np.float64]: ...
    def computeFrameJacobian(
        self, q: NDArray[np.float64], frame_name: str, local: bool = ...
    ) -> NDArray[np.float64]: ...
    def getJointGroupInfo(self, name: str) -> JointGroupInfo: ...
    def getPositionLimitVectors(
        self, group_name: str = ..., collapsed: bool = ...
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]: ...
    def addBoxGeometry(
        self,
        name: str,
        parent_frame: str,
        box: Box,
        tform: NDArray[np.float64],
        color: NDArray[np.float64],
    ) -> None: ...
    def addSphereGeometry(
        self,
        name: str,
        parent_frame: str,
        sphere: Sphere,
        tform: NDArray[np.float64],
        color: NDArray[np.float64],
    ) -> None: ...
    def addCylinderGeometry(
        self,
        name: str,
        parent_frame: str,
        cylinder: Cylinder,
        tform: NDArray[np.float64],
        color: NDArray[np.float64],
    ) -> None: ...
    def addMeshGeometry(
        self,
        name: str,
        parent_frame: str,
        mesh: Mesh,
        tform: NDArray[np.float64],
        color: NDArray[np.float64],
    ) -> None: ...
    def updateGeometryPlacement(
        self, name: str, parent_frame: str, tform: NDArray[np.float64]
    ) -> None: ...
    def removeGeometry(self, name: str) -> None: ...
    def setCollisions(self, body1: str, body2: str, enable: bool) -> None: ...

def hasCollisionsAlongPath(
    scene: Scene,
    q_start: NDArray[np.float64],
    q_end: NDArray[np.float64],
    max_step_size: float,
    bisection: bool = ...,
    check_endpoints: bool = ...,
) -> bool: ...
