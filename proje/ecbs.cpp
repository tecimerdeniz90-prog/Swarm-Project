#include "ecbs.h"
#include <iostream>
#include <limits>
#include <algorithm>

ECBS::ECBS(float w_high, float w_low)
    : w_high_(w_high), w_low_(w_low) {}

ECBS::~ECBS() {
    for (auto node : allocated_nodes_) {
        delete node;
    }
}

int ECBS::countAllConflicts(const std::vector<std::vector<Point3D>>& paths, const CollisionDetector& detector) const {
    int num_agents = paths.size();
    int conflicts = 0;
    for (int i = 0; i < num_agents; ++i) {
        for (int j = i + 1; j < num_agents; ++j) {
            if (paths[i].empty() || paths[j].empty()) continue;
            size_t max_size = std::max(paths[i].size(), paths[j].size());
            for (size_t t = 0; t < max_size; ++t) {
                Point3D pi = (t < paths[i].size()) ? paths[i][t] : paths[i].back();
                Point3D pj = (t < paths[j].size()) ? paths[j][t] : paths[j].back();
                if (detector.checkAgentCollision(pi.x, pi.y, pi.z, pj.x, pj.y, pj.z)) {
                    conflicts++;
                }
            }
        }
    }
    return conflicts;
}

bool ECBS::findFirstConflict(const std::vector<std::vector<Point3D>>& paths, const CollisionDetector& detector,
                             int& agent_i, int& agent_j, int& step, Point3D& pi, Point3D& pj) const {
    int num_agents = paths.size();
    for (int i = 0; i < num_agents; ++i) {
        for (int j = i + 1; j < num_agents; ++j) {
            if (paths[i].empty() || paths[j].empty()) continue;
            size_t max_size = std::max(paths[i].size(), paths[j].size());
            for (size_t t = 0; t < max_size; ++t) {
                Point3D pt_i = (t < paths[i].size()) ? paths[i][t] : paths[i].back();
                Point3D pt_j = (t < paths[j].size()) ? paths[j][t] : paths[j].back();
                if (detector.checkAgentCollision(pt_i.x, pt_i.y, pt_i.z, pt_j.x, pt_j.y, pt_j.z)) {
                    agent_i = i;
                    agent_j = j;
                    step = static_cast<int>(t);
                    pi = pt_i;
                    pj = pt_j;
                    return true;
                }
            }
        }
    }
    return false;
}

bool ECBS::plan(const std::vector<Point3D>& starts, const std::vector<Point3D>& goals,
                const VoxelMap& map, const CollisionDetector& detector,
                std::vector<std::vector<Point3D>>& solutions) {
    int M = starts.size();
    if (M == 0) return true;

    // Allocate root node
    CTNode* root = new CTNode(M);
    allocated_nodes_.push_back(root);

    KinodynamicAStar planner(0.5f, 2.0f, 1.0f, w_low_);

    std::cout << "Planning initial paths for " << M << " agents..." << std::endl;
    for (int i = 0; i < M; ++i) {
        std::vector<Point3D> path;
        bool ok = planner.search(starts[i], goals[i], map, detector, {}, {}, path);
        if (!ok) {
            std::cerr << "ECBS Init Error: Failed to find path for agent " << i << std::endl;
            return false;
        }
        root->paths[i] = path;
    }

    // Compute root cost and conflicts
    root->cost = 0;
    for (int i = 0; i < M; ++i) {
        root->cost += root->paths[i].size();
    }
    root->conflicts = countAllConflicts(root->paths, detector);

    std::vector<CTNode*> open_list;
    open_list.push_back(root);

    const int max_ct_nodes = 500;
    int ct_nodes_expanded = 0;

    std::cout << "Starting high-level conflict tree search..." << std::endl;
    while (!open_list.empty() && ct_nodes_expanded++ < max_ct_nodes) {
        // Find minimum cost in OPEN
        float cost_min = std::numeric_limits<float>::max();
        for (const auto node : open_list) {
            if (node->cost < cost_min) {
                cost_min = node->cost;
            }
        }

        // Form FOCAL list: cost <= cost_min * w_high_
        std::vector<size_t> focal_indices;
        float focal_bound = cost_min * w_high_;
        for (size_t i = 0; i < open_list.size(); ++i) {
            if (open_list[i]->cost <= focal_bound) {
                focal_indices.push_back(i);
            }
        }

        // Select the CT node from FOCAL with minimum conflicts, tie-breaker: minimum cost
        size_t best_focal_idx = 0;
        int min_conflicts = std::numeric_limits<int>::max();
        float min_cost = std::numeric_limits<float>::max();

        for (size_t i = 0; i < focal_indices.size(); ++i) {
            size_t open_idx = focal_indices[i];
            CTNode* node = open_list[open_idx];
            if (node->conflicts < min_conflicts) {
                min_conflicts = node->conflicts;
                min_cost = node->cost;
                best_focal_idx = open_idx;
            } else if (node->conflicts == min_conflicts && node->cost < min_cost) {
                min_cost = node->cost;
                best_focal_idx = open_idx;
            }
        }

        // Pop best node
        CTNode* curr = open_list[best_focal_idx];
        open_list.erase(open_list.begin() + best_focal_idx);

        std::cout << "CT Node Expanded: " << ct_nodes_expanded 
                  << " | Cost: " << curr->cost 
                  << " | Conflicts: " << curr->conflicts << std::endl;

        // Check for conflicts
        int agent_i, agent_j, step;
        Point3D pi, pj;
        bool has_conflict = findFirstConflict(curr->paths, detector, agent_i, agent_j, step, pi, pj);

        if (!has_conflict) {
            solutions = curr->paths;
            std::cout << "Conflict-free solutions found! Total CT nodes expanded: " << ct_nodes_expanded << std::endl;
            return true;
        }

        std::cout << "  Conflict found between agent " << agent_i << " and " << agent_j 
                  << " at step " << step << std::endl;

        // Branch 1: Constraint for Agent i
        CTNode* child1 = new CTNode(M);
        child1->paths = curr->paths;
        child1->constraints = curr->constraints;
        
        // Agent i must avoid position pj (agent j's position) at time step
        child1->constraints[agent_i].push_back({ step, pj.x, pj.y, pj.z });
        
        // Re-plan agent i's path
        std::vector<std::vector<Point3D>> other_paths;
        for (int k = 0; k < M; ++k) {
            if (k != agent_i) {
                other_paths.push_back(child1->paths[k]);
            }
        }

        std::vector<Point3D> new_path_i;
        bool ok1 = planner.search(starts[agent_i], goals[agent_i], map, detector, 
                                  child1->constraints[agent_i], other_paths, new_path_i);
        if (ok1) {
            child1->paths[agent_i] = new_path_i;
            child1->cost = 0;
            for (int k = 0; k < M; ++k) child1->cost += child1->paths[k].size();
            child1->conflicts = countAllConflicts(child1->paths, detector);
            
            allocated_nodes_.push_back(child1);
            open_list.push_back(child1);
        } else {
            delete child1;
        }

        // Branch 2: Constraint for Agent j
        CTNode* child2 = new CTNode(M);
        child2->paths = curr->paths;
        child2->constraints = curr->constraints;
        
        // Agent j must avoid position pi (agent i's position) at time step
        child2->constraints[agent_j].push_back({ step, pi.x, pi.y, pi.z });

        // Re-plan agent j's path
        other_paths.clear();
        for (int k = 0; k < M; ++k) {
            if (k != agent_j) {
                other_paths.push_back(child2->paths[k]);
            }
        }

        std::vector<Point3D> new_path_j;
        bool ok2 = planner.search(starts[agent_j], goals[agent_j], map, detector, 
                                  child2->constraints[agent_j], other_paths, new_path_j);
        if (ok2) {
            child2->paths[agent_j] = new_path_j;
            child2->cost = 0;
            for (int k = 0; k < M; ++k) child2->cost += child2->paths[k].size();
            child2->conflicts = countAllConflicts(child2->paths, detector);

            allocated_nodes_.push_back(child2);
            open_list.push_back(child2);
        } else {
            delete child2;
        }
    }

    std::cerr << "Error: ECBS failed to find a conflict-free solution (CT limit reached)." << std::endl;
    return false;
}
