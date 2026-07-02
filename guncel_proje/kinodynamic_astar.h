#ifndef KINODYNAMIC_ASTAR_H
#define KINODYNAMIC_ASTAR_H

#include "pcd_reader.h"
#include "voxel_map.h"
#include "collision_detector.h"
#include <vector>
#include <memory>
#include <cmath>

struct SpaceTimeConstraint {
    int step;
    float x, y, z;
};

struct KinodynamicState {
    float x, y, z;
    float vx, vy, vz;
    int step; // time step index
};

struct PathNode {
    KinodynamicState state;
    PathNode* parent;
    float g; // actual cost
    float h; // heuristic cost
    float f; // g + h
    int conflicts; // conflicts with other paths

    PathNode(const KinodynamicState& s, PathNode* p = nullptr, float g_val = 0.0f, float h_val = 0.0f, int conf = 0)
        : state(s), parent(p), g(g_val), h(h_val), f(g_val + h_val), conflicts(conf) {}
};

class KinodynamicAStar {
public:
    KinodynamicAStar(float dt = 0.5f, float v_max = 2.0f, float a_max = 1.0f, float w = 1.1f);
    ~KinodynamicAStar();

    // Runs Focal Search to find a kinodynamically feasible path
    bool search(const Point3D& start, const Point3D& goal,
                const VoxelMap& map, const CollisionDetector& detector,
                const std::vector<SpaceTimeConstraint>& constraints,
                const std::vector<std::vector<Point3D>>& other_paths,
                std::vector<Point3D>& path);

private:
    float dt_;
    float v_max_;
    float a_max_;
    float w_; // suboptimality factor

    std::vector<PathNode*> allocated_nodes_;

    float calculateHeuristic(const KinodynamicState& state, const Point3D& goal) const;
    int countConflicts(const KinodynamicState& state, const std::vector<std::vector<Point3D>>& other_paths, const CollisionDetector& detector) const;
    bool checkTransition(const KinodynamicState& start, const KinodynamicState& end, const VoxelMap& map, const CollisionDetector& detector) const;
    bool violatesConstraints(const KinodynamicState& state, const std::vector<SpaceTimeConstraint>& constraints, const CollisionDetector& detector) const;
    
    // Hash function and equality check for 3D state discretization in closed list
    struct StateHash {
        size_t operator()(const KinodynamicState& s) const {
            int ix = static_cast<int>(std::round(s.x / 0.4f));
            int iy = static_cast<int>(std::round(s.y / 0.4f));
            int iz = static_cast<int>(std::round(s.z / 0.4f));
            int ivx = static_cast<int>(std::round(s.vx / 0.4f));
            int ivy = static_cast<int>(std::round(s.vy / 0.4f));
            int ivz = static_cast<int>(std::round(s.vz / 0.4f));
            return (std::hash<int>()(ix) ^ (std::hash<int>()(iy) << 1) ^ (std::hash<int>()(iz) << 2) ^
                    (std::hash<int>()(ivx) << 3) ^ (std::hash<int>()(ivy) << 4) ^ (std::hash<int>()(ivz) << 5) ^
                    (std::hash<int>()(s.step) << 6));
        }
    };

    struct StateEqual {
        bool operator()(const KinodynamicState& a, const KinodynamicState& b) const {
            int ax = static_cast<int>(std::round(a.x / 0.4f));
            int ay = static_cast<int>(std::round(a.y / 0.4f));
            int az = static_cast<int>(std::round(a.z / 0.4f));
            int avx = static_cast<int>(std::round(a.vx / 0.4f));
            int avy = static_cast<int>(std::round(a.vy / 0.4f));
            int avz = static_cast<int>(std::round(a.vz / 0.4f));

            int bx = static_cast<int>(std::round(b.x / 0.4f));
            int by = static_cast<int>(std::round(b.y / 0.4f));
            int bz = static_cast<int>(std::round(b.z / 0.4f));
            int bvx = static_cast<int>(std::round(b.vx / 0.4f));
            int bvy = static_cast<int>(std::round(b.vy / 0.4f));
            int bvz = static_cast<int>(std::round(b.vz / 0.4f));

            return (ax == bx && ay == by && az == bz &&
                    avx == bvx && avy == bvy && avz == bvz &&
                    a.step == b.step);
        }
    };
};

#endif // KINODYNAMIC_ASTAR_H
