#ifndef VOXEL_MAP_H
#define VOXEL_MAP_H

#include "pcd_reader.h"
#include <vector>
#include <string>

class VoxelMap {
public:
    VoxelMap(float resolution = 0.2f);

    // Loads PCD file and builds the voxel grid
    bool loadMap(const std::string& filepath);

    // Checks if a 3D coordinate is out of bounds
    bool isOutOfBounds(float x, float y, float z) const;

    // Checks if a 3D coordinate is occupied
    bool isOccupied(float x, float y, float z) const;

    // Getters for dimensions and bounds
    float getResolution() const { return resolution_; }
    Point3D getMinPt() const { return min_pt_; }
    Point3D getMaxPt() const { return max_pt_; }
    
    int getDimX() const { return dim_x_; }
    int getDimY() const { return dim_y_; }
    int getDimZ() const { return dim_z_; }

private:
    float resolution_;
    Point3D min_pt_;
    Point3D max_pt_;
    
    int dim_x_;
    int dim_y_;
    int dim_z_;

    std::vector<uint8_t> grid_;

    // Helpers for conversion
    void posToIdx(float x, float y, float z, int& ix, int& iy, int& iz) const;
    size_t gridIdx(int ix, int iy, int iz) const;
};

#endif // VOXEL_MAP_H
