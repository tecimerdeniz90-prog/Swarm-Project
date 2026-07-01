#include "collision_detector.h"
#include <cmath>
#include <iostream>
#include <algorithm>

CollisionDetector::CollisionDetector(float rx, float ry, float rz)
    : rx_(rx), ry_(ry), rz_(rz) {}

bool CollisionDetector::checkObstacleCollision(const VoxelMap& map, float x, float y, float z) const {
    // 1. Calculate bounding box of the ellipsoid
    float x_min = x - rx_;
    float x_max = x + rx_;
    float y_min = y - ry_;
    float y_max = y + ry_;
    float z_min = z - rz_;
    float z_max = z + rz_;

    // If bounding box exceeds map bounds, it is considered a collision
    if (map.isOutOfBounds(x_min, y_min, z_min) || map.isOutOfBounds(x_max, y_max, z_max)) {
        return true;
    }

    // 2. Iterate through all voxels within the bounding box
    float res = map.getResolution();
    Point3D map_min = map.getMinPt();

    int ix_min = static_cast<int>(std::floor((x_min - map_min.x) / res));
    int ix_max = static_cast<int>(std::ceil((x_max - map_min.x) / res));
    int iy_min = static_cast<int>(std::floor((y_min - map_min.y) / res));
    int iy_max = static_cast<int>(std::ceil((y_max - map_min.y) / res));
    int iz_min = static_cast<int>(std::floor((z_min - map_min.z) / res));
    int iz_max = static_cast<int>(std::ceil((z_max - map_min.z) / res));

    // Clamp to map index range
    ix_min = std::max(0, ix_min);
    ix_max = std::min(map.getDimX() - 1, ix_max);
    iy_min = std::max(0, iy_min);
    iy_max = std::min(map.getDimY() - 1, iy_max);
    iz_min = std::max(0, iz_min);
    iz_max = std::min(map.getDimZ() - 1, iz_max);

    // Padding for conservative collision checking
    float rx_pad = rx_ + res * 0.5f;
    float ry_pad = ry_ + res * 0.5f;
    float rz_pad = rz_ + res * 0.5f;

    float rx_sq = rx_pad * rx_pad;
    float ry_sq = ry_pad * ry_pad;
    float rz_sq = rz_pad * rz_pad;

    for (int ix = ix_min; ix <= ix_max; ++ix) {
        float vx = map_min.x + ix * res;
        float dx = vx - x;
        float term_x = (dx * dx) / rx_sq;
        if (term_x > 1.0f) continue;

        for (int iy = iy_min; iy <= iy_max; ++iy) {
            float vy = map_min.y + iy * res;
            float dy = vy - y;
            float term_y = term_x + (dy * dy) / ry_sq;
            if (term_y > 1.0f) continue;

            for (int iz = iz_min; iz <= iz_max; ++iz) {
                float vz = map_min.z + iz * res;
                float dz = vz - z;
                float dist_val = term_y + (dz * dz) / rz_sq;

                if (dist_val <= 1.0f) {
                    if (map.isOccupied(vx, vy, vz)) {
                        return true;
                    }
                }
            }
        }
    }

    return false;
}

bool CollisionDetector::checkAgentCollision(float x1, float y1, float z1, float x2, float y2, float z2) const {
    float dx = x1 - x2;
    float dy = y1 - y2;
    float dz = z1 - z2;

    float rx_double = 2.0f * rx_;
    float ry_double = 2.0f * ry_;
    float rz_double = 2.0f * rz_;

    float dist_val = (dx * dx) / (rx_double * rx_double) +
                     (dy * dy) / (ry_double * ry_double) +
                     (dz * dz) / (rz_double * rz_double);

    return dist_val <= 1.0f;
}
