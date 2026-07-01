#include "voxel_map.h"
#include <cmath>
#include <iostream>
#include <algorithm>

VoxelMap::VoxelMap(float resolution)
    : resolution_(resolution), dim_x_(0), dim_y_(0), dim_z_(0) {
    min_pt_ = {0, 0, 0};
    max_pt_ = {0, 0, 0};
}

bool VoxelMap::loadMap(const std::string& filepath) {
    std::vector<Point3D> points;
    if (!readPCD(filepath, points, min_pt_, max_pt_)) {
        return false;
    }

    // Add a small safety margin around bounds to avoid edge cases
    min_pt_.x -= resolution_;
    min_pt_.y -= resolution_;
    min_pt_.z -= resolution_;
    max_pt_.x += resolution_;
    max_pt_.y += resolution_;
    max_pt_.z += resolution_;

    dim_x_ = static_cast<int>(std::ceil((max_pt_.x - min_pt_.x) / resolution_)) + 1;
    dim_y_ = static_cast<int>(std::ceil((max_pt_.y - min_pt_.y) / resolution_)) + 1;
    dim_z_ = static_cast<int>(std::ceil((max_pt_.z - min_pt_.z) / resolution_)) + 1;

    std::cout << "Grid dimensions: " << dim_x_ << " x " << dim_y_ << " x " << dim_z_ << std::endl;
    
    // Safety check for memory allocation
    size_t total_cells = static_cast<size_t>(dim_x_) * dim_y_ * dim_z_;
    double memory_mb = static_cast<double>(total_cells) / (1024.0 * 1024.0);
    std::cout << "Allocating grid of size: " << memory_mb << " MB" << std::endl;

    if (memory_mb > 500.0) {
        std::cerr << "Warning: Map resolution is too fine or map is too large. Attempting to allocate anyway." << std::endl;
    }

    grid_.assign(total_cells, 0);

    for (const auto& pt : points) {
        int ix, iy, iz;
        posToIdx(pt.x, pt.y, pt.z, ix, iy, iz);
        if (ix >= 0 && ix < dim_x_ && iy >= 0 && iy < dim_y_ && iz >= 0 && iz < dim_z_) {
            grid_[gridIdx(ix, iy, iz)] = 1;
        }
    }

    std::cout << "Voxelization completed. Map loaded successfully." << std::endl;
    return true;
}

bool VoxelMap::isOutOfBounds(float x, float y, float z) const {
    int ix, iy, iz;
    posToIdx(x, y, z, ix, iy, iz);
    return (ix < 0 || ix >= dim_x_ || iy < 0 || iy >= dim_y_ || iz < 0 || iz >= dim_z_);
}

bool VoxelMap::isOccupied(float x, float y, float z) const {
    int ix, iy, iz;
    posToIdx(x, y, z, ix, iy, iz);
    if (ix < 0 || ix >= dim_x_ || iy < 0 || iy >= dim_y_ || iz < 0 || iz >= dim_z_) {
        return true; // Out of bounds is treated as occupied
    }
    return grid_[gridIdx(ix, iy, iz)] != 0;
}

void VoxelMap::posToIdx(float x, float y, float z, int& ix, int& iy, int& iz) const {
    ix = static_cast<int>(std::round((x - min_pt_.x) / resolution_));
    iy = static_cast<int>(std::round((y - min_pt_.y) / resolution_));
    iz = static_cast<int>(std::round((z - min_pt_.z) / resolution_));
}

size_t VoxelMap::gridIdx(int ix, int iy, int iz) const {
    return static_cast<size_t>(ix) * dim_y_ * dim_z_ + static_cast<size_t>(iy) * dim_z_ + iz;
}
