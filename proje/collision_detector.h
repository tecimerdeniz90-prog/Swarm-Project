#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include "voxel_map.h"

class CollisionDetector {
public:
    CollisionDetector(float rx = 0.15f, float ry = 0.15f, float rz = 1.5f);

    // Checks if a UAV at (x, y, z) collides with the voxel map obstacles
    bool checkObstacleCollision(const VoxelMap& map, float x, float y, float z) const;

    // Checks if two UAVs collide (uses ellipsoidal Minkowski sum)
    bool checkAgentCollision(float x1, float y1, float z1, float x2, float y2, float z2) const;

    // Getters for dimensions
    float rx() const { return rx_; }
    float ry() const { return ry_; }
    float rz() const { return rz_; }

private:
    float rx_;
    float ry_;
    float rz_;
};

#endif // COLLISION_DETECTOR_H
