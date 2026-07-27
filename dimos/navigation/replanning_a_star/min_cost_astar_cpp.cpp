// Copyright 2025 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace py = pybind11;

// Movement directions (8-connected grid)
// Order: right, down, left, up, down-right, down-left, up-right, up-left
constexpr int DX[8] = {0, 1, 0, -1, 1, 1, -1, -1};
constexpr int DY[8] = {1, 0, -1, 0, 1, -1, 1, -1};

// Movement costs: straight = 1.0, diagonal = sqrt(2) ≈ 1.42
constexpr double STRAIGHT_COST = 1.0;
constexpr double DIAGONAL_COST = 1.42;
constexpr double MOVE_COSTS[8] = {
    STRAIGHT_COST, STRAIGHT_COST, STRAIGHT_COST, STRAIGHT_COST,
    DIAGONAL_COST, DIAGONAL_COST, DIAGONAL_COST, DIAGONAL_COST
};

constexpr int8_t COST_UNKNOWN = -1;
constexpr int8_t COST_FREE = 0;

// Pack coordinates into a single 64-bit key for fast hashing
inline uint64_t pack_coords(int x, int y) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32) |
           static_cast<uint64_t>(static_cast<uint32_t>(y));
}

// Unpack coordinates from 64-bit key
inline std::pair<int, int> unpack_coords(uint64_t key) {
    return {static_cast<int>(key >> 32), static_cast<int>(key & 0xFFFFFFFF)};
}

// Octile distance heuristic - optimal for 8-connected grids with diagonal movement
inline double heuristic(int x1, int y1, int x2, int y2) {
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    // Octile distance: straight moves + diagonal adjustment
    return (dx + dy) + (DIAGONAL_COST - 2 * STRAIGHT_COST) * std::min(dx, dy);
}

// Reconstruct path from goal to start using parent map
inline std::vector<std::pair<int, int>> reconstruct_path(
    const std::unordered_map<uint64_t, uint64_t>& parents,
    uint64_t goal_key,
    int start_x,
    int start_y
) {
    std::vector<std::pair<int, int>> path;
    uint64_t node = goal_key;

    while (parents.count(node)) {
        auto [x, y] = unpack_coords(node);
        path.emplace_back(x, y);
        node = parents.at(node);
    }

    path.emplace_back(start_x, start_y);
    std::reverse(path.begin(), path.end());
    return path;
}

// Priority queue node: (priority_objective, priority_dist, x, y)
struct Node {
    double objective;
    double dist;
    int x;
    int y;

    // Min-heap comparison: lower values have higher priority
    bool operator>(const Node& other) const {
        if (objective != other.objective) return objective > other.objective;
        return dist > other.dist;
    }
};

/**
 * A* pathfinding algorithm optimized for costmap grids.
 *
 * @param grid 2D numpy array of int8 values (height x width)
 * @param start_x Starting X coordinate in grid cells
 * @param start_y Starting Y coordinate in grid cells
 * @param goal_x Goal X coordinate in grid cells
 * @param goal_y Goal Y coordinate in grid cells
 * @param cost_threshold Cells with value >= this are obstacles (default: 100)
 * @param unknown_penalty Cost multiplier for unknown cells (default: 0.8)
 * @param distance_weight Weight for geometric movement distance (default: 0.0)
 * @param cell_cost_weight Weight for normalized cell costs (default: 1.0)
 * @return Vector of (x, y) grid coordinates from start to goal, empty if no path
 */
std::vector<std::pair<int, int>> min_cost_astar_cpp(
    py::array_t<int8_t, py::array::c_style | py::array::forcecast> grid,
    int start_x,
    int start_y,
    int goal_x,
    int goal_y,
    int cost_threshold = 100,
    double unknown_penalty = 0.8,
    double distance_weight = 0.0,
    double cell_cost_weight = 1.0
) {
    if (cost_threshold <= 0) {
        throw std::invalid_argument("cost_threshold must be positive");
    }
    if (distance_weight < 0.0 || cell_cost_weight < 0.0) {
        throw std::invalid_argument("A* objective weights must be non-negative");
    }

    // Get buffer info for direct array access
    auto buf = grid.unchecked<2>();
    const int height = static_cast<int>(buf.shape(0));
    const int width = static_cast<int>(buf.shape(1));

    // Bounds check for goal
    if (goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height) {
        return {};
    }

    // Bounds check for start
    if (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height) {
        return {};
    }

    const uint64_t start_key = pack_coords(start_x, start_y);
    const uint64_t goal_key = pack_coords(goal_x, goal_y);

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;

    std::unordered_set<uint64_t> closed_set;
    closed_set.reserve(width * height / 4);  // Pre-allocate

    // Parent tracking for path reconstruction
    std::unordered_map<uint64_t, uint64_t> parents;
    parents.reserve(width * height / 4);

    // Score tracking (weighted objective and distance tiebreaker)
    std::unordered_map<uint64_t, double> objective_score;
    std::unordered_map<uint64_t, double> dist_score;
    objective_score.reserve(width * height / 4);
    dist_score.reserve(width * height / 4);

    // Initialize start node
    objective_score[start_key] = 0.0;
    dist_score[start_key] = 0.0;
    double h = heuristic(start_x, start_y, goal_x, goal_y);
    open_set.push({distance_weight * h, h, start_x, start_y});

    while (!open_set.empty()) {
        Node current = open_set.top();
        open_set.pop();

        const int cx = current.x;
        const int cy = current.y;
        const uint64_t current_key = pack_coords(cx, cy);

        if (closed_set.count(current_key)) {
            continue;
        }

        if (current_key == goal_key) {
            return reconstruct_path(parents, current_key, start_x, start_y);
        }

        closed_set.insert(current_key);

        const double current_objective = objective_score[current_key];
        const double current_dist = dist_score[current_key];

        // Explore all 8 neighbors
        for (int i = 0; i < 8; ++i) {
            const int nx = cx + DX[i];
            const int ny = cy + DY[i];

            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                continue;
            }

            const uint64_t neighbor_key = pack_coords(nx, ny);

            if (closed_set.count(neighbor_key)) {
                continue;
            }

            // Get cell value (note: grid is [y, x] in row-major order)
            const int8_t val = buf(ny, nx);

            if (val >= cost_threshold) {
                continue;
            }

            double cell_cost;
            if (val == COST_UNKNOWN) {
                cell_cost = cost_threshold * unknown_penalty;
                if (cell_cost >= cost_threshold) {
                    continue;
                }
            } else if (val == COST_FREE) {
                cell_cost = 0.0;
            } else {
                cell_cost = static_cast<double>(val);
            }

            const double tentative_dist = current_dist + MOVE_COSTS[i];
            const double tentative_objective = current_objective
                + distance_weight * MOVE_COSTS[i]
                + cell_cost_weight * (cell_cost / static_cast<double>(cost_threshold));

            // Get existing scores (infinity if not yet visited)
            auto objective_it = objective_score.find(neighbor_key);
            auto dist_it = dist_score.find(neighbor_key);
            const double n_objective = (objective_it != objective_score.end())
                ? objective_it->second
                : INFINITY;
            const double n_dist = (dist_it != dist_score.end()) ? dist_it->second : INFINITY;

            // Raw distance remains the tiebreaker for legacy zero-distance weighting.
            if (tentative_objective < n_objective ||
                (tentative_objective == n_objective && tentative_dist < n_dist)) {

                // Update parent and scores
                parents[neighbor_key] = current_key;
                objective_score[neighbor_key] = tentative_objective;
                dist_score[neighbor_key] = tentative_dist;

                // Cell costs are non-negative, so the geometric term is admissible.
                const double h_dist = heuristic(nx, ny, goal_x, goal_y);
                const double priority_cost = tentative_objective + distance_weight * h_dist;
                const double priority_dist = tentative_dist + h_dist;

                open_set.push({priority_cost, priority_dist, nx, ny});
            }
        }
    }

    return {};
}

PYBIND11_MODULE(min_cost_astar_ext, m) {
    m.doc() = "C++ implementation of A* pathfinding for costmap grids";

    m.def("min_cost_astar_cpp", &min_cost_astar_cpp,
          "A* pathfinding on a costmap grid.\n\n"
          "Args:\n"
          "    grid: 2D numpy array of int8 values (height x width)\n"
          "    start_x: Starting X coordinate in grid cells\n"
          "    start_y: Starting Y coordinate in grid cells\n"
          "    goal_x: Goal X coordinate in grid cells\n"
          "    goal_y: Goal Y coordinate in grid cells\n"
          "    cost_threshold: Cells >= this value are obstacles (default: 100)\n"
          "    unknown_penalty: Cost multiplier for unknown cells (default: 0.8)\n"
          "    distance_weight: Weight for geometric distance (default: 0.0)\n"
          "    cell_cost_weight: Weight for normalized cell costs (default: 1.0)\n\n"
          "Returns:\n"
          "    List of (x, y) grid coordinates from start to goal, or empty list if no path",
          py::arg("grid"),
          py::arg("start_x"),
          py::arg("start_y"),
          py::arg("goal_x"),
          py::arg("goal_y"),
          py::arg("cost_threshold") = 100,
          py::arg("unknown_penalty") = 0.8,
          py::arg("distance_weight") = 0.0,
          py::arg("cell_cost_weight") = 1.0);
}
