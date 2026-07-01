#ifndef ECBS_H
#define ECBS_H

#include "kinodynamic_astar.h"
#include "collision_detector.h"
#include "voxel_map.h"
#include <vector>

struct CTNode {
    std::vector<std::vector<Point3D>> paths;
    std::vector<std::vector<SpaceTimeConstraint>> constraints;
    float cost;
    int conflicts;

    CTNode(int num_agents) {
        paths.resize(num_agents);
        constraints.resize(num_agents);
        cost = 0.0f;
        conflicts = 0;
    }
};

class ECBS {
public:
    ECBS(float w_high = 1.2f, float w_low = 1.1f);
    ~ECBS();

    // Runs Enhanced Conflict-Based Search to coordinate multi-UAV paths
    bool plan(const std::vector<Point3D>& starts, const std::vector<Point3D>& goals,
              const VoxelMap& map, const CollisionDetector& detector,
              std::vector<std::vector<Point3D>>& solutions);

private:
    float w_high_;
    float w_low_;
    std::vector<CTNode*> allocated_nodes_;

    int countAllConflicts(const std::vector<std::vector<Point3D>>& paths, const CollisionDetector& detector) const;
    bool findFirstConflict(const std::vector<std::vector<Point3D>>& paths, const CollisionDetector& detector,
                           int& agent_i, int& agent_j, int& step, Point3D& pi, Point3D& pj) const;
};

#endif // ECBS_H
