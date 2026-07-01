cmake_minimum_required(VERSION 3.10)
project(MultiUAVPathPlanner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
set(SOURCES
    main.cpp
    pcd_reader.cpp
    voxel_map.cpp
    collision_detector.cpp
    kinodynamic_astar.cpp
    ecbs.cpp
)

# Header files (for IDE grouping)
set(HEADERS
    pcd_reader.h
    voxel_map.h
    collision_detector.h
    kinodynamic_astar.h
    ecbs.h
)

# Executable target
add_executable(planner ${SOURCES} ${HEADERS})
