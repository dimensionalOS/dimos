# Copyright 2025 Dimensional Inc.
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


import genesis as gs  # type: ignore

from ..base.simulator_base import SimulatorBase


class GenesisSimulator(SimulatorBase):
    """Genesis simulator implementation."""

    def __init__(
        self,
        headless: bool = True,
        open_usd: str | None = None,  # Keep for compatibility
        entities: list[dict[str, str | dict]] | None = None,  # type: ignore[type-arg]
    ) -> None:
        """Initialize the Genesis simulation.

        Args:
            headless: Whether to run without visualization
            open_usd: Path to USD file (for Isaac)
            entities: List of entity configurations to load. Each entity is a dict with:
                     - type: str ('mesh', 'urdf', 'mjcf', 'primitive')
                     - path: str (file path for mesh/urdf/mjcf)
                     - params: dict (parameters for primitives or loading options)
        """
        super().__init__(headless, open_usd, entities)

        # Initialize Genesis
        gs.init()

        # Create scene with viewer options
        self.scene = gs.Scene(
            show_viewer=not headless,
            viewer_options=gs.options.ViewerOptions(
                res=(1280, 960),
                camera_pos=(3.5, 0.0, 2.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=40,
                max_FPS=60,
            ),
            vis_options=gs.options.VisOptions(
                show_world_frame=True,
                world_frame_size=1.0,
                show_link_frame=False,
                show_cameras=False,
                plane_reflection=True,
                ambient_light=(0.1, 0.1, 0.1),
            ),
            renderer=gs.renderers.Rasterizer(),
        )

        # Handle USD parameter for compatibility
        if open_usd:
            print(f"[Warning] USD files not supported in Genesis. Ignoring: {open_usd}")

        # Load entities if provided
        if entities:
            self._load_entities(entities)

        # Don't build scene yet - let stream add camera first
        self.is_built = False

    def _load_entities(self, entities: list[dict[str, str | dict]]):  # type: ignore[no-untyped-def, type-arg]
        """Load multiple entities into the scene."""
        for entity in entities:
            entity_type = entity.get("type", "").lower()  # type: ignore[union-attr]
            path = entity.get("path", "")
            params = entity.get("params", {})

            try:
                if entity_type == "mesh":
                    mesh = gs.morphs.Mesh(
                        file=path,  # Explicit file argument
                        **params,
                    )
                    self.scene.add_entity(mesh)
                    print(f"[Genesis] Added mesh from {path}")

                elif entity_type == "urdf":
                    robot = gs.morphs.URDF(
                        file=path,  # Explicit file argument
                        **params,
                    )
                    self.scene.add_entity(robot)
                    print(f"[Genesis] Added URDF robot from {path}")

                elif entity_type == "mjcf":
                    mujoco = gs.morphs.MJCF(
                        file=path,  # Explicit file argument
                        **params,
                    )
                    self.scene.add_entity(mujoco)
                    print(f"[Genesis] Added MJCF model from {path}")

                elif entity_type == "primitive":
                    shape_type = params.pop("shape", "plane")  # type: ignore[union-attr]
                    if shape_type == "plane":
                        morph = gs.morphs.Plane(**params)
                    elif shape_type == "box":
                        morph = gs.morphs.Box(**params)
                    elif shape_type == "sphere":
                        morph = gs.morphs.Sphere(**params)
                    else:
                        raise ValueError(f"Unsupported primitive shape: {shape_type}")

                    # Add position if not specified
                    if "pos" not in params:
                        if shape_type == "plane":
                            morph.pos = [0, 0, 0]
                        else:
                            morph.pos = [0, 0, 1]  # Lift objects above ground

                    self.scene.add_entity(morph)
                    print(f"[Genesis] Added {shape_type} at position {morph.pos}")

                else:
                    raise ValueError(f"Unsupported entity type: {entity_type}")

            except Exception as e:
                print(f"[Warning] Failed to load entity {entity}: {e!s}")

    def add_entity(self, entity_type: str, path: str = "", **params) -> None:  # type: ignore[no-untyped-def]
        """Add a single entity to the scene.

        Args:
            entity_type: Type of entity ('mesh', 'urdf', 'mjcf', 'primitive')
            path: File path for mesh/urdf/mjcf entities
            **params: Additional parameters for entity creation
        """
        self._load_entities([{"type": entity_type, "path": path, "params": params}])

    def get_stage(self):  # type: ignore[no-untyped-def]
        """Get the current stage/scene."""
        return self.scene

    def build(self) -> None:
        """Build the scene if not already built."""
        if not self.is_built:
            self.scene.build()
            self.is_built = True

    def close(self) -> None:
        """Close the simulation."""
        # Genesis handles cleanup automatically
        pass
