#include "pcd_reader.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <limits>

bool readPCD(const std::string& filepath, std::vector<Point3D>& points, Point3D& min_pt, Point3D& max_pt) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open PCD file: " << filepath << std::endl;
        return false;
    }

    std::string line;
    int num_points = 0;
    bool is_binary = false;

    // Parse header
    while (std::getline(file, line)) {
        if (line.rfind("POINTS", 0) == 0) {
            std::stringstream ss(line);
            std::string temp;
            ss >> temp >> num_points;
        } else if (line.rfind("DATA", 0) == 0) {
            if (line.find("binary") != std::string::npos) {
                is_binary = true;
            }
            break;
        }
    }

    if (!is_binary) {
        std::cerr << "Error: Only binary PCD files are supported." << std::endl;
        return false;
    }

    if (num_points <= 0) {
        std::cerr << "Error: Invalid number of points in header: " << num_points << std::endl;
        return false;
    }

    points.reserve(num_points);
    min_pt = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    max_pt = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };

    // Point format: 4 floats + 4 bytes padding = 20 bytes total.
    struct RawPoint {
        float index;
        float x;
        float y;
        float z;
        uint8_t padding[4];
    };

    std::vector<RawPoint> raw_points(num_points);
    file.read(reinterpret_cast<char*>(raw_points.data()), num_points * sizeof(RawPoint));

    size_t read_bytes = file.gcount();
    size_t read_points = read_bytes / sizeof(RawPoint);

    for (size_t i = 0; i < read_points; ++i) {
        Point3D pt = { raw_points[i].x, raw_points[i].y, raw_points[i].z };
        points.push_back(pt);

        if (pt.x < min_pt.x) min_pt.x = pt.x;
        if (pt.x > max_pt.x) max_pt.x = pt.x;
        if (pt.y < min_pt.y) min_pt.y = pt.y;
        if (pt.y > max_pt.y) max_pt.y = pt.y;
        if (pt.z < min_pt.z) min_pt.z = pt.z;
        if (pt.z > max_pt.z) max_pt.z = pt.z;
    }

    std::cout << "Successfully loaded " << points.size() << " points from " << filepath << std::endl;
    std::cout << "Map bounding box: [" << min_pt.x << ", " << max_pt.x << "] x ["
              << min_pt.y << ", " << max_pt.y << "] x ["
              << min_pt.z << ", " << max_pt.z << "]" << std::endl;

    return true;
}
