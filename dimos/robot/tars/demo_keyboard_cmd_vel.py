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

"""Main-thread WASD teleop publishing /cmd_vel (works on macOS, unlike KeyboardTeleop).

    dimos run coordinator-tars-sim            # terminal 1
    python -m dimos.robot.tars.demo_keyboard_cmd_vel   # terminal 2

W/S forward/back, A/D turn, Space stop, Esc quit.
"""

from __future__ import annotations

import pygame

from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.Twist import Twist

VX, WZ, HZ = 0.5, 0.4, 20  # m/s, rad/s (half-size TARS tops out ~0.55 m/s)


def main() -> None:
    tx = make_transport("/cmd_vel", Twist)
    pygame.init()
    screen = pygame.display.set_mode((360, 120))
    pygame.display.set_caption("TARS cmd_vel")
    font = pygame.font.Font(None, 28)
    clock = pygame.time.Clock()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
            ):
                running = False
        keys = pygame.key.get_pressed()
        vx = VX * (keys[pygame.K_w] - keys[pygame.K_s])
        wz = WZ * (keys[pygame.K_a] - keys[pygame.K_d])
        if keys[pygame.K_SPACE]:
            vx = wz = 0.0
        tx.broadcast(None, Twist(linear=[vx, 0, 0], angular=[0, 0, wz]))
        screen.fill((20, 20, 24))
        screen.blit(
            font.render(f"vx {vx:+.2f} m/s   wz {wz:+.2f} rad/s", True, (220, 220, 220)), (20, 45)
        )
        pygame.display.flip()
        clock.tick(HZ)
    pygame.quit()


if __name__ == "__main__":
    main()
