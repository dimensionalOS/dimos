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

"""Pygame visualization for SimpleRobot."""

import math
import threading


def run_visualization(robot, window_size=(800, 800), meters_per_pixel=0.02):
    """Run pygame visualization for a robot. Call from a thread."""
    import pygame

    pygame.init()
    screen = pygame.display.set_mode(window_size)
    pygame.display.set_caption("Simple Robot")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)

    # Colors
    BG = (30, 30, 40)
    GRID = (50, 50, 60)
    ROBOT = (100, 200, 255)
    ARROW = (255, 150, 100)
    TEXT = (200, 200, 200)

    w, h = window_size
    cx, cy = w // 2, h // 2

    while robot._running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                robot._running = False
                break
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    robot._running = False
                    break

        screen.fill(BG)

        # Draw grid (1 meter spacing)
        grid_spacing = int(1.0 / meters_per_pixel)
        for x in range(0, w, grid_spacing):
            pygame.draw.line(screen, GRID, (x, 0), (x, h))
        for y in range(0, h, grid_spacing):
            pygame.draw.line(screen, GRID, (0, y), (w, y))

        # Robot position in screen coords
        rx = cx + int(robot.x / meters_per_pixel)
        ry = cy - int(robot.y / meters_per_pixel)

        # Draw robot body
        pygame.draw.circle(screen, ROBOT, (rx, ry), 20)

        # Draw direction arrow
        ax = rx + int(30 * math.cos(robot.theta))
        ay = ry - int(30 * math.sin(robot.theta))
        pygame.draw.line(screen, ARROW, (rx, ry), (ax, ay), 3)

        # Arrowhead
        for sign in [-1, 1]:
            hx = ax - int(10 * math.cos(robot.theta + sign * 2.5))
            hy = ay + int(10 * math.sin(robot.theta + sign * 2.5))
            pygame.draw.line(screen, ARROW, (ax, ay), (hx, hy), 3)

        # Info text
        info = [
            f"Position: ({robot.x:.2f}, {robot.y:.2f}) m",
            f"Heading: {math.degrees(robot.theta):.1f}°",
            f"Velocity: {robot.linear_vel.x:.2f} m/s",
            f"Angular: {math.degrees(robot.angular_vel.z):.1f}°/s",
        ]
        for i, text in enumerate(info):
            screen.blit(font.render(text, True, TEXT), (10, 10 + i * 25))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


def start_visualization(robot, **kwargs):
    """Start visualization in a background thread."""
    thread = threading.Thread(target=run_visualization, args=(robot,), kwargs=kwargs, daemon=True)
    thread.start()
    return thread
