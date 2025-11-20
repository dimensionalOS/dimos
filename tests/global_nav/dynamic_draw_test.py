import numpy as np
from vectortypes import Vector
from draw import Drawer
from astar import astar
import random
import matplotlib.pyplot as plt
from costmap import Costmap

# Load the costmap
costmap = Costmap.from_pickle("costmapMsg.pickle")

# Create a smudged version of the costmap for better planning
smudged_costmap = costmap.smudge(
    kernel_size=10, iterations=10, threshold=80, preserve_unknown=False
)

# Create a drawer that we'll reuse
drawer = Drawer(
    figsize=(12, 10),
    title="A* Path Planning - Navigating to Random Points",
    show_axes=True,
    show_colorbar=False,
    dark_mode=True,
)

# Start position
start = Vector(0.0, 0.0)

# Configure interactive mode for real-time updates
plt.ion()

update_interval = 1

# Precompute valid cells for more efficient sampling
# Create a mask of valid cells (cost < 50 and not unknown)
valid_cells_mask = (costmap.grid < 50) & (costmap.grid >= 0)
valid_y_indices, valid_x_indices = np.where(valid_cells_mask)

try:
    while True:
        # Generate a random goal by sampling directly from valid cells
        # Pick a random index from the valid cells
        idx = random.randint(0, len(valid_y_indices) - 1)

        # Get grid coordinates from the random selection
        grid_y, grid_x = valid_y_indices[idx], valid_x_indices[idx]

        # Convert to world coordinates
        goal = costmap.grid_to_world(Vector(grid_x, grid_y))

        print(f"Planning path from {start} to {goal}")

        # Plan path using A*
        path = astar(
            smudged_costmap,
            start=start,
            goal=goal,
            cost_threshold=50,
        )

        if path:
            print(f"Path found with {len(path)} waypoints")

            # Resample the path for smoother visualization
            resampled_path = path.resample(0.5)

            # Clear previous plot
            drawer.clear(title="A* Path Planning")
            drawer.draw(
                smudged_costmap,
                (
                    costmap,
                    {"transparent_empty": True, "cmap": "Greys", "alpha": 0.7},
                ),
                (path, {"color": "yellow", "linewidth": 1.5}),
                (resampled_path, {"color": "green", "linewidth": 2}),
                (start, {"color": "blue", "markersize": 100, "show_text": True}),
                (goal, {"color": "red", "markersize": 100, "show_text": True}),
            )

            plt.draw()
            drawer.fig.canvas.flush_events()
            plt.pause(update_interval)

            # Update start to be the previous goal for continuous movement
            start = goal
        else:
            print(f"No path found to goal {goal}")
            # Try a different goal next time but keep the same start

    # Keep the plot open after all iterations
    print("\nPath planning demo completed.")

except KeyboardInterrupt:
    print("\nInterrupted by user")
    plt.ioff()

except Exception as e:
    print(f"Error during navigation demo: {e}")
    raise
