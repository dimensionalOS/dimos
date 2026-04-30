# Viewing & Navigation

When running DimSim in headed mode (default), you get a browser-based 3D view of the simulation. This page covers the controls for navigating the scene while the robot operates.

## God View (Free Camera)

In dimos mode, the camera is detached from the robot — you get a free-flying "god view" to observe the scene from any angle while the robot navigates autonomously.

Click anywhere in the browser to capture the mouse pointer. Press `Escape` to release it.

<!-- TODO: Add god view screenshot -->
![God view of the scene](assets/god-view.png)

## Keyboard Controls

| Key | Action |
|-----|--------|
| **W** | Move forward |
| **A** | Move left |
| **S** | Move backward |
| **D** | Move right |
| **Space** | Move up (fly) |
| **Shift** | Move down (fly) |
| **G** | Toggle ghost mode (fly through walls) |
| **Mouse** | Look around |

## Ghost Mode

Press **G** to toggle ghost mode. In ghost mode, the camera passes through walls and objects — useful for quickly reaching any part of the scene. Press **G** again to return to normal collision mode.

## Robot View

The robot's sensor feed (color image, depth, LiDAR) is now only visible in rerun, not in the browser. The browser shows the 3D world from your camera angle while rerun shows what the robot sees.
