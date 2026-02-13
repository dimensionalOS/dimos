"""Simulation engines for manipulator backends."""

from __future__ import annotations

from typing import Literal

from dimos.simulation.engines.base import SimulationEngine
from dimos.simulation.engines.mujoco_engine import MujocoEngine

EngineType = Literal["mujoco"]

_ENGINES: dict[EngineType, type[SimulationEngine]] = {
    "mujoco": MujocoEngine,
}


def get_engine(engine_name: EngineType) -> type[SimulationEngine]:
    return _ENGINES[engine_name]


__all__ = [
    "EngineType",
    "SimulationEngine",
    "get_engine",
]
