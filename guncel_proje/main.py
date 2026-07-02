import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

from voxel_map import VoxelMap
from collision_detector import CollisionDetector
from ecbs import ECBS

def main():
    print("====== Python Multi-UAV Collaborative Path Planner ======")
    
    # 1. Load Voxel Map
    map_path = "Map/10X30m.pcd"
    voxel_map = VoxelMap(resolution=0.2)
    if not voxel_map.load_map(map_path):
        print("Failed to load map.")
        return

    # 2. Setup Collision Detector (Downwash model)
    detector = CollisionDetector(rx=0.15, ry=0.15, rz=0.5)

    # 3. Setup Starts and Goals (verified to have high clearance)
    starts = [
        [-6.5, -10.0, 1.5],
        [-3.8, -10.0, 2.4],
        [-1.6, -10.0, 2.2]
    ]

    goals = [
        [-9.0, 10.0, 2.0],
        [-6.4, 10.0, 2.6],
        [-7.0, 10.0, 2.0]
    ]

    # Double check start positions are free
    for i in range(len(starts)):
        s_coll = detector.check_obstacle_collision(voxel_map, starts[i][0], starts[i][1], starts[i][2])
        g_coll = detector.check_obstacle_collision(voxel_map, goals[i][0], goals[i][1], goals[i][2])
        print(f"Agent {i}: Start {starts[i]} collides={s_coll} | Goal {goals[i]} collides={g_coll}")
        if s_coll or g_coll:
            return

    # 4. Plan Paths using ECBS
    ecbs = ECBS(w_high=1.5, w_low=1.4)
    solutions = ecbs.plan(starts, goals, voxel_map, detector)

    if solutions is None:
        print("Failed to find conflict-free paths.")
        return

    # Save paths to JSON
    output_data = {
        "map_file": map_path,
        "starts": starts,
        "goals": goals,
        "paths": solutions
    }
    with open("paths.json", "w") as f:
        json.dump(output_data, f, indent=2)
    print("Saved planned paths to paths.json.")

    # 5. Run Visualization
    visualize_paths(map_path, starts, goals, solutions)

def visualize_paths(map_path, starts, goals, solutions):
    # Read points directly from map_path
    import struct
    print("Loading map points for plotting...")
    points = []
    with open(map_path, 'rb') as f:
        while True:
            line = f.readline()
            if b"DATA binary" in line:
                break
        data = f.read()
        num_points = len(data) // 20
        for i in range(num_points):
            offset = i * 20
            idx, x, y, z = struct.unpack_from('ffff', data, offset)
            points.append([x, y, z])
    points = np.array(points)

    # Downsample points for plotting speed
    points_ds = points[::300]
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Multi-UAVs Cramped Environment Path Planning")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    # Plot map point cloud as obstacles
    ax.scatter(points_ds[:, 0], points_ds[:, 1], points_ds[:, 2], 
               c='gray', s=1, alpha=0.3, label='Obstacles (Point Cloud)')

    colors = ['cyan', 'magenta', 'lime', 'orange', 'yellow']
    
    # Plot starts, goals, and static paths
    for i, path in enumerate(solutions):
        c = colors[i % len(colors)]
        path = np.array(path)
        # Start
        ax.scatter(starts[i][0], starts[i][1], starts[i][2], c='green', marker='o', s=80, edgecolors='black')
        ax.text(starts[i][0], starts[i][1], starts[i][2] + 0.3, f"Start {i}", color='green', weight='bold')
        
        # Goal
        ax.scatter(goals[i][0], goals[i][1], goals[i][2], c='red', marker='*', s=120, edgecolors='black')
        ax.text(goals[i][0], goals[i][1], goals[i][2] + 0.3, f"Goal {i}", color='red', weight='bold')
        
        # Path line
        ax.plot(path[:, 0], path[:, 1], path[:, 2], c=c, linewidth=2, label=f"UAV {i} Path")

    ax.set_xlim(min(points_ds[:, 0]), max(points_ds[:, 0]))
    ax.set_ylim(min(points_ds[:, 1]), max(points_ds[:, 1]))
    ax.set_zlim(0, 6)

    plt.legend()
    
    # Animation setup
    paths = [np.array(p) for p in solutions]
    max_len = max(len(p) for p in paths)
    
    drone_dots = []
    for i in range(len(paths)):
        c = colors[i % len(colors)]
        dot, = ax.plot([], [], [], c=c, marker='o', markersize=8, markeredgecolor='black')
        drone_dots.append(dot)
        
    def update(frame):
        for i, path in enumerate(paths):
            idx = min(frame, len(path) - 1)
            pos = path[idx]
            drone_dots[i].set_data([pos[0]], [pos[1]])
            drone_dots[i].set_3d_properties([pos[2]])
        return drone_dots

    print("Starting path animation window...")
    print("Saving path animation to simulation.gif...")
    ani = FuncAnimation(fig, update, frames=max_len, interval=100, blit=True)
    ani.save('simulation.gif', writer='pillow', fps=10)
    print("Saved simulation.gif successfully!")

if __name__ == "__main__":
    main()
