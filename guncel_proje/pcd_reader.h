#ifndef PCD_READER_H
#define PCD_READER_H

#include <string>
#include <vector>

struct Point3D {
    float x;
    float y;
    float z;
};

// Reads point cloud data from a binary PCD file.
// Calculates the bounding box (min_pt and max_pt).
bool readPCD(const std::string& filepath, std::vector<Point3D>& points, Point3D& min_pt, Point3D& max_pt);

#endif // PCD_READER_H
