#include "pcd_reader.h"
#include "voxel_map.h"
#include "collision_detector.h"
#include "kinodynamic_astar.h"
#include "ecbs.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

Point3D findFreePosition(const VoxelMap& map, const CollisionDetector& detector, Point3D target) {
    if (!detector.checkObstacleCollision(map, target.x, target.y, target.z)) {
        return target;
    }
    std::cout << "Adjusting point (" << target.x << ", " << target.y << ", " << target.z << ") to avoid obstacles..." << std::endl;
    for (float r = 0.2f; r <= 3.0f; r += 0.2f) {
        for (float dx = -r; dx <= r; dx += 0.2f) {
            for (float dy = -r; dy <= r; dy += 0.2f) {
                for (float dz = -r; dz <= r; dz += 0.2f) {
                    Point3D pt = { target.x + dx, target.y + dy, target.z + dz };
                    if (!detector.checkObstacleCollision(map, pt.x, pt.y, pt.z)) {
                        std::cout << "  Adjusted to (" << pt.x << ", " << pt.y << ", " << pt.z << ")" << std::endl;
                        return pt;
                    }
                }
            }
        }
    }
    return target;
}

int main() {
    std::cout << "====== Multi-UAV Collaborative Path Planner ======" << std::endl;

    std::string map_path = "multi-UAVs-path-planner-main/Map/10X30m.pcd";
    VoxelMap map(0.2f); // 0.2m voxel resolution
    
    if (!map.loadMap(map_path)) {
        std::cerr << "Failed to load map: " << map_path << std::endl;
        return 1;
    }

    CollisionDetector detector(0.15f, 0.15f, 0.5f); // 0.5m z-axis (downwash)

    // Define 3 agents with starting positions on one end and goals on the other end
    // Map bounding box: [-9.4, 0.5] x [-22.1, 17.6] x [-0.1, 6.0]
    std::vector<Point3D> raw_starts = {
        { -6.5f, -10.0f, 1.5f },
        { -3.8f, -10.0f, 2.4f },
        { -1.6f, -10.0f, 2.2f }
    };

    std::vector<Point3D> raw_goals = {
        { -9.0f, 10.0f, 2.0f },
        { -6.4f, 10.0f, 2.6f },
        { -5.8f, 10.0f, 2.8f }
    };

    std::vector<Point3D> starts;
    std::vector<Point3D> goals;
    for (size_t i = 0; i < raw_starts.size(); ++i) {
        starts.push_back(findFreePosition(map, detector, raw_starts[i]));
        goals.push_back(findFreePosition(map, detector, raw_goals[i]));
    }

    // High-level suboptimality w_high = 1.5, low-level w_low = 1.4
    ECBS ecbs(1.5f, 1.4f);
    std::vector<std::vector<Point3D>> solutions;

    bool success = ecbs.plan(starts, goals, map, detector, solutions);

    if (success) {
        std::cout << "Plan search succeeded! Writing output to paths.json..." << std::endl;

        // Manual JSON writing to avoid third-party dependencies
        std::ofstream json_file("paths.json");
        json_file << "{\n";
        json_file << "  \"map_file\": \"" << map_path << "\",\n";
        
        // Starts
        json_file << "  \"starts\": [\n";
        for (size_t i = 0; i < starts.size(); ++i) {
            json_file << "    [" << starts[i].x << ", " << starts[i].y << ", " << starts[i].z << "]"
                      << (i + 1 < starts.size() ? ",\n" : "\n");
        }
        json_file << "  ],\n";

        // Goals
        json_file << "  \"goals\": [\n";
        for (size_t i = 0; i < goals.size(); ++i) {
            json_file << "    [" << goals[i].x << ", " << goals[i].y << ", " << goals[i].z << "]"
                      << (i + 1 < goals.size() ? ",\n" : "\n");
        }
        json_file << "  ],\n";

        // Paths
        json_file << "  \"paths\": [\n";
        for (size_t i = 0; i < solutions.size(); ++i) {
            json_file << "    [\n";
            for (size_t t = 0; t < solutions[i].size(); ++t) {
                json_file << "      [" << solutions[i][t].x << ", " << solutions[i][t].y << ", " << solutions[i][t].z << "]"
                          << (t + 1 < solutions[i].size() ? ",\n" : "\n");
            }
            json_file << "    ]" << (i + 1 < solutions.size() ? ",\n" : "\n");
        }
        json_file << "  ]\n";
        json_file << "}\n";
        json_file.close();

        std::cout << "Paths written successfully to paths.json." << std::endl;
    } else {
        std::cerr << "Plan search failed." << std::endl;
    }

    return 0;
}
