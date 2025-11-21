import math
import heapq
from typing import Optional
from dimos.types.path import Path
from dimos.types.vector import VectorLike
from dimos.types.costmap import Costmap


def astar(
    costmap: Costmap,
    goal: VectorLike,
    start: VectorLike = (0.0, 0.0),
    cost_threshold: int = 60,
    allow_diagonal: bool = True,
) -> Optional[Path]:
    """
    A* path planning algorithm from start to goal position.

    Args:
        costmap: Costmap object containing the environment
        goal: Goal position as any vector-like object
        start: Start position as any vector-like object (default: origin [0,0])
        cost_threshold: Cost threshold above which a cell is considered an obstacle
        allow_diagonal: Whether to allow diagonal movements

    Returns:
        Path object containing waypoints, or None if no path found
    """
    # Convert world coordinates to grid coordinates directly using vector-like inputs
    start_vector = costmap.world_to_grid(start)
    goal_vector = costmap.world_to_grid(goal)

    print("RUNNING ASTAR", costmap, "\n", goal, "\n", start)
    # Check if start or goal is out of bounds or in an obstacle
    if not (0 <= start_vector.x < costmap.width and 0 <= start_vector.y < costmap.height) or not (
        0 <= goal_vector.x < costmap.width and 0 <= goal_vector.y < costmap.height
    ):
        print("Start or goal position is out of bounds")
        return None

    # Check if start or goal is in an obstacle
    if (
        costmap.grid[int(start_vector.y), int(start_vector.x)] >= cost_threshold
        or costmap.grid[int(goal_vector.y), int(goal_vector.x)] >= cost_threshold
    ):
        print("Start or goal position is in an obstacle")
        return None

    # Define possible movements (8-connected grid)
    if allow_diagonal:
        # 8-connected grid: horizontal, vertical, and diagonal movements
        directions = [
            (0, 1),
            (1, 0),
            (0, -1),
            (-1, 0),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ]
    else:
        # 4-connected grid: only horizontal and vertical movements
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    # Cost for each movement (straight vs diagonal)
    movement_costs = [1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414] if allow_diagonal else [1.0, 1.0, 1.0, 1.0]

    # A* algorithm implementation
    open_set = []  # Priority queue for nodes to explore
    closed_set = set()  # Set of explored nodes

    # Convert Vector objects to tuples for dictionary keys
    start_tuple = (int(start_vector.x), int(start_vector.y))
    goal_tuple = (int(goal_vector.x), int(goal_vector.y))

    # Dictionary to store cost from start and parents for each node
    g_score = {start_tuple: 0}
    parents = {}

    # Heuristic function (Euclidean distance)
    def heuristic(x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # Start with the starting node
    f_score = g_score[start_tuple] + heuristic(start_tuple[0], start_tuple[1], goal_tuple[0], goal_tuple[1])
    heapq.heappush(open_set, (f_score, start_tuple))

    while open_set:
        # Get the node with the lowest f_score
        _, current = heapq.heappop(open_set)
        current_x, current_y = current

        # Check if we've reached the goal
        if current == goal_tuple:
            # Reconstruct the path
            waypoints = []
            while current in parents:
                world_point = costmap.grid_to_world(current)
                waypoints.append(world_point)
                current = parents[current]

            # Add the start position
            start_world_point = costmap.grid_to_world(start_tuple)
            waypoints.append(start_world_point)

            # Reverse the path (start to goal)
            waypoints.reverse()

            # Add the goal position if it's not already included
            goal_point = costmap.grid_to_world(goal_tuple)

            if not waypoints or waypoints[-1].distance(goal_point) > 1e-5:
                waypoints.append(goal_point)

            return Path(waypoints)

        # Add current node to closed set
        closed_set.add(current)

        # Explore neighbors
        for i, (dx, dy) in enumerate(directions):
            neighbor_x, neighbor_y = current_x + dx, current_y + dy
            neighbor = (neighbor_x, neighbor_y)

            # Check if the neighbor is valid
            if not (0 <= neighbor_x < costmap.width and 0 <= neighbor_y < costmap.height):
                continue

            # Check if the neighbor is already explored
            if neighbor in closed_set:
                continue

            # Check if the neighbor is an obstacle
            if costmap.grid[neighbor_y, neighbor_x] >= cost_threshold:
                continue

            # Calculate g_score for the neighbor
            tentative_g_score = g_score[current] + movement_costs[i]

            # Get the current g_score for the neighbor or set to infinity if not yet explored
            neighbor_g_score = g_score.get(neighbor, float("inf"))

            # If this path to the neighbor is better than any previous one
            if tentative_g_score < neighbor_g_score:
                # Update the neighbor's scores and parent
                parents[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic(neighbor_x, neighbor_y, goal_tuple[0], goal_tuple[1])

                # Add the neighbor to the open set with its f_score
                heapq.heappush(open_set, (f_score, neighbor))

    # If we get here, no path was found
    return None


if __name__ == "__main__":
    from costmap import Costmap

    # Load the costmap
    costmap = Costmap.from_pickle("costmapMsg.pickle")

    # Create a smudged version of the costmap for better planning
    smudged_costmap = costmap.smudge()

    # Test different types of inputs for goal position
    start = Vector(0.0, 0.0)  # Define a single position
    goal = Vector(5.0, -7.0)  # Define a single position

    print("A* navigating\nfrom\n", start, "\nto\n", goal, "\non\n", smudged_costmap)

    # Try each type of input
    path = astar(smudged_costmap, start=start, goal=goal, cost_threshold=50)

    print("result\n", path)
