"""A scene with three standing places for a G1."""

from pathlib import Path

from dimos.robot.assets.model import UsdModel

THREE_G1_SCENE = UsdModel.from_file(Path(__file__).parent / "scene.usda")
