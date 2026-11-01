# Copyright 2026 Dimensional Inc.
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

"""Numerical arm limits and gravity calculations."""

from __future__ import annotations

import importlib

import numpy as np
from numpy.typing import NDArray

from dimos.robot.assets.model import RobotModel
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PhysicalModel:
    """Numerical arm model: exact coordinates, bounded feedback and gravity."""

    def __init__(
        self,
        robot_model: RobotModel,
        joint_names: list[str],
        *,
        gravity_comp: bool,
        feedback_margin_rad: float = 0.05,
    ) -> None:
        self._gravity_comp = gravity_comp
        self._margin = feedback_margin_rad
        self._names = tuple(joint_names)
        # Physical model support is only needed when a native session is opened.
        pinocchio = importlib.import_module("pinocchio")
        self._pinocchio = pinocchio
        model = pinocchio.buildModelFromXML(robot_model.load().xml)
        locked = [
            index
            for index, name in enumerate(model.names)
            if index and str(name) not in self._names
        ]
        if locked:
            model = pinocchio.buildReducedModel(model, locked, pinocchio.neutral(model))
        count = len(self._names)
        if model.nq != count or model.nv != count:
            raise ValueError("physical model dimensions do not match angular joints")
        if tuple(str(name) for name in model.names[1:]) != self._names:
            raise ValueError("physical model joint order does not match angular joints")
        self._lower = np.asarray(model.lowerPositionLimit, dtype=np.float64)
        self._upper = np.asarray(model.upperPositionLimit, dtype=np.float64)
        if (
            self._lower.shape != (count,)
            or self._upper.shape != (count,)
            or not np.isfinite([self._lower, self._upper]).all()
            or np.any(self._lower >= self._upper)
        ):
            raise ValueError("physical model contains invalid arm position limits")
        self._model = model
        self._data = model.createData()
        self._warned: set[tuple[str, str]] = set()

    def gravity(self, positions: NDArray[np.float64]) -> NDArray[np.float64]:
        if positions.shape != (len(self._names),) or not np.isfinite(positions).all():
            raise ValueError("arm feedback contains non-finite or invalid positions")
        gravity = (
            np.asarray(
                self._pinocchio.computeGeneralizedGravity(self._model, self._data, positions),
                dtype=np.float64,
            )
            if self._gravity_comp
            else np.zeros(len(self._names))
        )
        if gravity.shape != positions.shape or not np.isfinite(gravity).all():
            raise ValueError("gravity compensation produced invalid torques")
        return gravity

    def normalize(self, positions: list[float]) -> list[float]:
        q = np.asarray(positions, dtype=np.float64)
        if q.shape != self._lower.shape or not np.isfinite(q).all():
            raise RuntimeError("arm feedback contains non-finite or invalid positions")
        for index, name in enumerate(self._names):
            lower, upper, value = self._lower[index], self._upper[index], q[index]
            if lower <= value <= upper:
                continue
            boundary = "lower" if value < lower else "upper"
            limit = lower if value < lower else upper
            if abs(value - limit) > self._margin + 1e-12:
                raise RuntimeError(
                    f"{name} reported {value} outside [{lower}, {upper}] by more than {self._margin} rad"
                )
            if (name, boundary) not in self._warned:
                logger.warning(
                    "Clamping Damiao joint feedback at position limit",
                    joint=name,
                    value=value,
                    limit=limit,
                )
                self._warned.add((name, boundary))
            q[index] = limit
        return [float(value) for value in q]
