@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
cl.exe /EHsc /O2 main.cpp pcd_reader.cpp voxel_map.cpp collision_detector.cpp kinodynamic_astar.cpp ecbs.cpp /Feplanner.exe
if %ERRORLEVEL% EQU 0 (
    planner.exe
)
