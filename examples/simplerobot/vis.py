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

    BG = (30, 30, 40)
    GRID = (50, 50, 60)
    ROBOT = (100, 200, 255)
    ARROW = (255, 150, 100)
    TEXT = (200, 200, 200)

    w, h = window_size
    cx, cy = w // 2, h // 2
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q):
                running = False

        pose, vel = robot._pose, robot._vel

        screen.fill(BG)

        # Grid (1m spacing)
        grid_spacing = int(1.0 / meters_per_pixel)
        for x in range(0, w, grid_spacing):
            pygame.draw.line(screen, GRID, (x, 0), (x, h))
        for y in range(0, h, grid_spacing):
            pygame.draw.line(screen, GRID, (0, y), (w, y))

        # Robot position in screen coords
        rx = cx + int(pose.x / meters_per_pixel)
        ry = cy - int(pose.y / meters_per_pixel)

        # Robot body
        pygame.draw.circle(screen, ROBOT, (rx, ry), 20)

        # Direction arrow
        ax = rx + int(45 * math.cos(pose.yaw))
        ay = ry - int(45 * math.sin(pose.yaw))
        pygame.draw.line(screen, ARROW, (rx, ry), (ax, ay), 3)
        for sign in [-1, 1]:
            hx = ax - int(10 * math.cos(pose.yaw + sign * 0.5))
            hy = ay + int(10 * math.sin(pose.yaw + sign * 0.5))
            pygame.draw.line(screen, ARROW, (ax, ay), (hx, hy), 3)

        # Info text
        info = [
            f"Position: ({pose.x:.2f}, {pose.y:.2f}) m",
            f"Heading: {math.degrees(pose.yaw):.1f}°",
            f"Velocity: {vel.linear.x:.2f} m/s",
            f"Angular: {math.degrees(vel.angular.z):.1f}°/s",
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
