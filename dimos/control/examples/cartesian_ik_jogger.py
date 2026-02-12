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

"""Pygame-based cartesian jogger for CartesianIKTask.

Publishes PoseStamped commands to the coordinator via LCM.
The frame_id is used as the task name for routing.

Keyboard controls for jogging robot end-effector in world frame:
    W/S: +X/-X (forward/backward)
    A/D: -Y/+Y (left/right)
    Q/E: +Z/-Z (up/down)
    R/F: +Roll/-Roll
    T/G: +Pitch/-Pitch
    Y/H: +Yaw/-Yaw
    SPACE: Reset to home pose
    ESC: Quit

Usage:
    python -m dimos.control.examples.cartesian_ik_jogger
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any

import numpy as np

try:
    import pygame
except ImportError:
    print("pygame not installed. Install with: pip install pygame")
    raise


@dataclass
class JogState:
    """Current jogging state."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def copy(self) -> JogState:
        return JogState(
            x=self.x,
            y=self.y,
            z=self.z,
            roll=self.roll,
            pitch=self.pitch,
            yaw=self.yaw,
        )

    @classmethod
    def from_fk(cls, model_path: str, ee_joint_id: int) -> JogState:
        """Create JogState from forward kinematics at zero configuration.

        This ensures the initial pose is reachable by the robot.
        """
        import pinocchio  # type: ignore[import-untyped]

        # Load model
        if model_path.endswith(".xml"):
            model = pinocchio.buildModelFromMJCF(model_path)
        else:
            model = pinocchio.buildModelFromUrdf(model_path)

        data = model.createData()

        # Compute FK at zero configuration
        q_zero = np.zeros(model.nq)
        pinocchio.forwardKinematics(model, data, q_zero)

        # Get EE pose
        ee_pose = data.oMi[ee_joint_id]
        position = ee_pose.translation
        rotation = ee_pose.rotation

        # Convert rotation matrix to RPY
        rpy = pinocchio.rpy.matrixToRpy(rotation)

        print("Initial EE pose from FK at q=0:")
        print(f"  Position: x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}")
        print(
            f"  Orientation: roll={np.degrees(rpy[0]):.1f}°, pitch={np.degrees(rpy[1]):.1f}°, yaw={np.degrees(rpy[2]):.1f}°"
        )

        return cls(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
            roll=float(rpy[0]),
            pitch=float(rpy[1]),
            yaw=float(rpy[2]),
        )

    def to_pose_stamped(self, task_name: str) -> Any:
        """Convert to PoseStamped for LCM publishing.

        Args:
            task_name: Task name to use as frame_id for routing
        """
        from dimos.msgs.geometry_msgs import PoseStamped
        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        position = Vector3(self.x, self.y, self.z)
        orientation = Quaternion.from_euler(Vector3(self.roll, self.pitch, self.yaw))

        return PoseStamped(
            ts=time.time(),
            frame_id=task_name,  # Used for task routing
            position=position,
            orientation=orientation,
        )


# Jog speeds
LINEAR_SPEED = 0.05  # m/s
ANGULAR_SPEED = 0.5  # rad/s

# Position limits (workspace bounds) - will be updated based on initial pose
X_LIMITS = (-0.5, 0.5)
Y_LIMITS = (-0.5, 0.5)
Z_LIMITS = (-0.2, 0.6)

# Task name for routing (must match blueprint config)
TASK_NAME = "cartesian_ik_arm"


def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, value))


def _get_piper_model_path() -> str:
    """Get path to Piper MJCF model."""
    from dimos.utils.data import get_data

    piper_path = get_data("piper_description")
    return str(piper_path / "mujoco_model" / "piper_no_gripper_description.xml")


def run_jogger_ui(model_path: str | None = None, ee_joint_id: int = 6) -> None:
    """Run the pygame-based cartesian jogger UI.

    This is ONLY the UI - it publishes PoseStamped to LCM.
    The coordinator must be running separately to receive commands.

    Args:
        model_path: Path to robot model (MJCF/URDF) for computing initial FK pose.
                   If None, uses Piper model.
        ee_joint_id: End-effector joint ID in the model
    """
    from dimos.core.transport import LCMTransport
    from dimos.msgs.geometry_msgs import PoseStamped

    # Use Piper model if not specified
    if model_path is None:
        model_path = _get_piper_model_path()

    print("Starting Cartesian IK Jogger UI...")
    print("Publishing to /coordinator/cartesian_command")
    print("(Coordinator must be running separately to receive commands)")

    # Create LCM publisher for sending cartesian commands
    transport: LCMTransport[PoseStamped] = LCMTransport(
        "/coordinator/cartesian_command", PoseStamped
    )

    # Initialize pygame
    pygame.init()
    screen = pygame.display.set_mode((600, 400))
    pygame.display.set_caption("Cartesian IK Jogger")
    font = pygame.font.Font(None, 28)
    clock = pygame.time.Clock()

    # Initial pose from forward kinematics at zero configuration
    # This ensures we start at a pose that's reachable from q=[0,0,0,0,0,0]
    home_pose = JogState.from_fk(model_path, ee_joint_id)
    current_pose = home_pose.copy()

    # Send initial pose via LCM
    transport.publish(current_pose.to_pose_stamped(TASK_NAME))

    running = True
    last_time = time.perf_counter()

    print("\nControls:")
    print("  W/S: +X/-X (forward/backward)")
    print("  A/D: -Y/+Y (left/right)")
    print("  Q/E: +Z/-Z (up/down)")
    print("  R/F: +Roll/-Roll")
    print("  T/G: +Pitch/-Pitch")
    print("  Y/H: +Yaw/-Yaw")
    print("  SPACE: Reset to home")
    print("  ESC: Quit")
    print()

    while running:
        dt = time.perf_counter() - last_time
        last_time = time.perf_counter()

        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    current_pose = home_pose.copy()
                    print("Reset to home pose")

        # Get pressed keys for continuous jogging
        keys = pygame.key.get_pressed()

        # Linear motion
        if keys[pygame.K_w]:
            current_pose.x += LINEAR_SPEED * dt
        if keys[pygame.K_s]:
            current_pose.x -= LINEAR_SPEED * dt
        if keys[pygame.K_a]:
            current_pose.y -= LINEAR_SPEED * dt
        if keys[pygame.K_d]:
            current_pose.y += LINEAR_SPEED * dt
        if keys[pygame.K_q]:
            current_pose.z += LINEAR_SPEED * dt
        if keys[pygame.K_e]:
            current_pose.z -= LINEAR_SPEED * dt

        # Angular motion
        if keys[pygame.K_r]:
            current_pose.roll += ANGULAR_SPEED * dt
        if keys[pygame.K_f]:
            current_pose.roll -= ANGULAR_SPEED * dt
        if keys[pygame.K_t]:
            current_pose.pitch += ANGULAR_SPEED * dt
        if keys[pygame.K_g]:
            current_pose.pitch -= ANGULAR_SPEED * dt
        if keys[pygame.K_y]:
            current_pose.yaw += ANGULAR_SPEED * dt
        if keys[pygame.K_h]:
            current_pose.yaw -= ANGULAR_SPEED * dt

        # Clamp to workspace limits
        current_pose.x = clamp(current_pose.x, *X_LIMITS)
        current_pose.y = clamp(current_pose.y, *Y_LIMITS)
        current_pose.z = clamp(current_pose.z, *Z_LIMITS)

        # Publish pose via LCM (frame_id = task name for routing)
        transport.publish(current_pose.to_pose_stamped(TASK_NAME))

        # Draw UI
        screen.fill((30, 30, 30))

        # Title
        title = font.render("Cartesian IK Jogger", True, (255, 255, 255))
        screen.blit(title, (200, 20))

        # Position display
        y_offset = 70
        pos_text = (
            f"Position: X={current_pose.x:.3f}  Y={current_pose.y:.3f}  Z={current_pose.z:.3f}"
        )
        pos_surf = font.render(pos_text, True, (100, 255, 100))
        screen.blit(pos_surf, (50, y_offset))

        # Orientation display
        y_offset += 30
        ori_text = f"Orientation: R={np.degrees(current_pose.roll):.1f}°  P={np.degrees(current_pose.pitch):.1f}°  Y={np.degrees(current_pose.yaw):.1f}°"
        ori_surf = font.render(ori_text, True, (100, 200, 255))
        screen.blit(ori_surf, (50, y_offset))

        # Controls
        y_offset += 50
        controls = [
            ("W/S", "+X/-X (forward/back)"),
            ("A/D", "-Y/+Y (left/right)"),
            ("Q/E", "+Z/-Z (up/down)"),
            ("R/F", "+Roll/-Roll"),
            ("T/G", "+Pitch/-Pitch"),
            ("Y/H", "+Yaw/-Yaw"),
            ("SPACE", "Reset to home"),
            ("ESC", "Quit"),
        ]

        for key, desc in controls:
            text = f"{key}: {desc}"
            surf = font.render(text, True, (180, 180, 180))
            screen.blit(surf, (50, y_offset))
            y_offset += 25

        # Active keys indicator
        y_offset += 20
        active_keys = []
        if keys[pygame.K_w]:
            active_keys.append("W")
        if keys[pygame.K_s]:
            active_keys.append("S")
        if keys[pygame.K_a]:
            active_keys.append("A")
        if keys[pygame.K_d]:
            active_keys.append("D")
        if keys[pygame.K_q]:
            active_keys.append("Q")
        if keys[pygame.K_e]:
            active_keys.append("E")

        if active_keys:
            active_text = f"Active: {' '.join(active_keys)}"
            active_surf = font.render(active_text, True, (255, 255, 0))
            screen.blit(active_surf, (50, y_offset))

        pygame.display.flip()
        clock.tick(50)  # 50 Hz update rate

    # Cleanup
    print("Jogger UI stopped.")
    pygame.quit()


def main() -> None:
    """Run the jogger UI standalone.

    Note: This only runs the UI. The coordinator must be started separately:
        Terminal 1: dimos run coordinator-cartesian-ik-mock
        Terminal 2: python -m dimos.control.examples.cartesian_ik_jogger
    """
    run_jogger_ui()


if __name__ == "__main__":
    main()
