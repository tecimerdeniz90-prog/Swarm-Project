#include "kinodynamic_astar.h"
#include <cmath>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include <limits>

KinodynamicAStar::KinodynamicAStar(float dt, float v_max, float a_max, float w)
    : dt_(dt), v_max_(v_max), a_max_(a_max), w_(w) {}

KinodynamicAStar::~KinodynamicAStar() {
    for (auto node : allocated_nodes_) {
        delete node;
    }
}

float KinodynamicAStar::calculateHeuristic(const KinodynamicState& state, const Point3D& goal) const {
    float dx = state.x - goal.x;
    float dy = state.y - goal.y;
    float dz = state.z - goal.z;
    float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    
    // Time heuristic: minimum time needed to reach goal at maximum velocity
    return dist / v_max_;
}

int KinodynamicAStar::countConflicts(const KinodynamicState& state,
                                     const std::vector<std::vector<Point3D>>& other_paths,
                                     const CollisionDetector& detector) const {
    int conflicts = 0;
    for (const auto& path : other_paths) {
        if (state.step < static_cast<int>(path.size())) {
            const auto& other_pt = path[state.step];
            if (detector.checkAgentCollision(state.x, state.y, state.z, other_pt.x, other_pt.y, other_pt.z)) {
                conflicts++;
            }
        } else if (!path.empty()) {
            // If the other agent has completed its path, it stays at its destination
            const auto& other_pt = path.back();
            if (detector.checkAgentCollision(state.x, state.y, state.z, other_pt.x, other_pt.y, other_pt.z)) {
                conflicts++;
            }
        }
    }
    return conflicts;
}

bool KinodynamicAStar::violatesConstraints(const KinodynamicState& state,
                                          const std::vector<SpaceTimeConstraint>& constraints,
                                          const CollisionDetector& detector) const {
    for (const auto& c : constraints) {
        if (c.step == state.step) {
            if (detector.checkAgentCollision(state.x, state.y, state.z, c.x, c.y, c.z)) {
                return true;
            }
        }
    }
    return false;
}

bool KinodynamicAStar::checkTransition(const KinodynamicState& start, const KinodynamicState& end,
                                       const VoxelMap& map, const CollisionDetector& detector) const {
    // Interpolate 5 points to check obstacle collision along the trajectory segment
    const int num_steps = 5;
    for (int i = 1; i <= num_steps; ++i) {
        float t = static_cast<float>(i) / num_steps;
        float x = start.x + t * (end.x - start.x);
        float y = start.y + t * (end.y - start.y);
        float z = start.z + t * (end.z - start.z);
        if (detector.checkObstacleCollision(map, x, y, z)) {
            return false;
        }
    }
    return true;
}

bool KinodynamicAStar::search(const Point3D& start, const Point3D& goal,
                              const VoxelMap& map, const CollisionDetector& detector,
                              const std::vector<SpaceTimeConstraint>& constraints,
                              const std::vector<std::vector<Point3D>>& other_paths,
                              std::vector<Point3D>& path) {
    // Clear previous search allocations
    for (auto node : allocated_nodes_) {
        delete node;
    }
    allocated_nodes_.clear();

    // Start state
    KinodynamicState start_state = { start.x, start.y, start.z, 0.0f, 0.0f, 0.0f, 0 };
    
    // Check if start is valid
    if (detector.checkObstacleCollision(map, start.x, start.y, start.z)) {
        std::cerr << "Error: Start position is in collision with obstacles." << std::endl;
        return false;
    }

    float start_h = calculateHeuristic(start_state, goal);
    int start_conf = countConflicts(start_state, other_paths, detector);
    PathNode* start_node = new PathNode(start_state, nullptr, 0.0f, start_h, start_conf);
    allocated_nodes_.push_back(start_node);

    std::vector<PathNode*> open_list;
    std::unordered_set<KinodynamicState, StateHash, StateEqual> closed_list;

    open_list.push_back(start_node);

    // Goal thresholds
    const float dist_threshold = 0.5f;
    const float vel_threshold = 0.5f;
    const int max_iterations = 20000;
    int iterations = 0;

    while (!open_list.empty() && iterations++ < max_iterations) {
        // Find f_min
        float f_min = std::numeric_limits<float>::max();
        for (const auto node : open_list) {
            if (node->f < f_min) {
                f_min = node->f;
            }
        }

        // Gather FOCAL list: open_list nodes with f <= f_min * w
        std::vector<size_t> focal_indices;
        float focal_bound = f_min * w_;
        for (size_t i = 0; i < open_list.size(); ++i) {
            if (open_list[i]->f <= focal_bound) {
                focal_indices.push_back(i);
            }
        }

        // Pick best node from FOCAL based on: conflicts (secondary priority) then h (tie-breaker)
        size_t best_focal_idx = 0;
        float min_conflicts = std::numeric_limits<float>::max();
        float min_h = std::numeric_limits<float>::max();
        
        for (size_t i = 0; i < focal_indices.size(); ++i) {
            size_t open_idx = focal_indices[i];
            PathNode* node = open_list[open_idx];
            if (node->conflicts < min_conflicts) {
                min_conflicts = node->conflicts;
                min_h = node->h;
                best_focal_idx = open_idx;
            } else if (node->conflicts == min_conflicts && node->h < min_h) {
                min_h = node->h;
                best_focal_idx = open_idx;
            }
        }

        // Pop best node
        PathNode* curr = open_list[best_focal_idx];
        open_list.erase(open_list.begin() + best_focal_idx);

        if (iterations % 2000 == 0) {
            std::cout << "[A* Debug] Iter: " << iterations 
                      << " | Pos: (" << curr->state.x << ", " << curr->state.y << ", " << curr->state.z << ")"
                      << " | g: " << curr->g << " | h: " << curr->h << " | f: " << curr->f 
                      << " | Open: " << open_list.size() << " | Closed: " << closed_list.size() << std::endl;
        }

        // Add to closed list
        closed_list.insert(curr->state);

        // Check if goal reached
        float dist_to_goal = std::sqrt(std::pow(curr->state.x - goal.x, 2) +
                                       std::pow(curr->state.y - goal.y, 2) +
                                       std::pow(curr->state.z - goal.z, 2));
        float vel_mag = std::sqrt(curr->state.vx * curr->state.vx +
                                  curr->state.vy * curr->state.vy +
                                  curr->state.vz * curr->state.vz);

        if (dist_to_goal < dist_threshold && vel_mag < vel_threshold) {
            // Reconstruct path
            path.clear();
            PathNode* temp = curr;
            while (temp != nullptr) {
                path.push_back({ temp->state.x, temp->state.y, temp->state.z });
                temp = temp->parent;
            }
            std::reverse(path.begin(), path.end());
            return true;
        }

        // Expand node
        // Discrete control inputs for accelerations in each axis
        std::vector<float> acc_options = { -a_max_, 0.0f, a_max_ };

        for (float ax : acc_options) {
            for (float ay : acc_options) {
                for (float az : acc_options) {
                    // Calculate next state
                    KinodynamicState next_state;
                    next_state.vx = curr->state.vx + ax * dt_;
                    next_state.vy = curr->state.vy + ay * dt_;
                    next_state.vz = curr->state.vz + az * dt_;
                    next_state.x = curr->state.x + curr->state.vx * dt_ + 0.5f * ax * dt_ * dt_;
                    next_state.y = curr->state.y + curr->state.vy * dt_ + 0.5f * ay * dt_ * dt_;
                    next_state.z = curr->state.z + curr->state.vz * dt_ + 0.5f * az * dt_ * dt_;
                    next_state.step = curr->state.step + 1;

                    // Speed limit check
                    float speed_sq = next_state.vx * next_state.vx +
                                     next_state.vy * next_state.vy +
                                     next_state.vz * next_state.vz;
                    if (speed_sq > v_max_ * v_max_) {
                        continue;
                    }

                    // Map bounds & obstacle collision check
                    if (detector.checkObstacleCollision(map, next_state.x, next_state.y, next_state.z)) {
                        continue;
                    }

                    // Space-time constraints check
                    if (violatesConstraints(next_state, constraints, detector)) {
                        continue;
                    }

                    // Segment collision check
                    if (!checkTransition(curr->state, next_state, map, detector)) {
                        continue;
                    }

                    // Closed list check
                    if (closed_list.find(next_state) != closed_list.end()) {
                        continue;
                    }

                    // Calculate step cost: Control effort + direction changes + time cost
                    float u_mag_sq = ax * ax + ay * ay + az * az;
                    float cos_theta = 1.0f;
                    float v_mag_curr = std::sqrt(curr->state.vx * curr->state.vx + curr->state.vy * curr->state.vy + curr->state.vz * curr->state.vz);
                    float v_mag_next = std::sqrt(next_state.vx * next_state.vx + next_state.vy * next_state.vy + next_state.vz * next_state.vz);
                    if (v_mag_curr > 0.001f && v_mag_next > 0.001f) {
                        cos_theta = (curr->state.vx * next_state.vx + curr->state.vy * next_state.vy + curr->state.vz * next_state.vz) / (v_mag_curr * v_mag_next);
                    }
                    float dir_change_penalty = 1.0f - cos_theta;

                    // Weights: lambda1 = 0.1, lambda2 = 1.0, lambda3 = 1.0
                    float step_cost = (0.1f * u_mag_sq + 1.0f * dir_change_penalty + 1.0f) * dt_;
                    float child_g = curr->g + step_cost;
                    float child_h = calculateHeuristic(next_state, goal);
                    int child_conf = curr->conflicts + countConflicts(next_state, other_paths, detector);

                    // Check if already in open list with better g
                    auto open_it = std::find_if(open_list.begin(), open_list.end(), [&](PathNode* n) {
                        return StateEqual()(n->state, next_state);
                    });

                    if (open_it != open_list.end()) {
                        if (child_g < (*open_it)->g) {
                            (*open_it)->g = child_g;
                            (*open_it)->f = child_g + (*open_it)->h;
                            (*open_it)->parent = curr;
                            (*open_it)->conflicts = child_conf;
                        }
                    } else {
                        PathNode* child = new PathNode(next_state, curr, child_g, child_h, child_conf);
                        allocated_nodes_.push_back(child);
                        open_list.push_back(child);
                    }
                }
            }
        }
    }

    std::cerr << "Warning: Kinodynamic A* failed to find a path (Iterations: " << iterations << ")." << std::endl;
    return false;
}
