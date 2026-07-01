import json
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

def read_pcd_binary(filepath):
    """Parses binary PCD format files to extract 3D point cloud coordinates."""
    print(f"Loading point cloud map: {filepath}...")
    with open(filepath, 'rb') as f:
        while True:
            line = f.readline()
            if not line:
                break
            if b"DATA binary" in line:
                break
        
        data = f.read()
        num_points = len(data) // 20
        points = []
        for i in range(num_points):
            offset = i * 20
            idx, x, y, z = struct.unpack_from('ffff', data, offset)
            points.append([x, y, z])
            
    print(f"Loaded {len(points)} points from PCD map.")
    return np.array(points)

def main():
    # Load paths output
    try:
        with open("paths.json", "r") as f:
            data = json.load(f)
    except FileNotFoundError:
        print("Error: paths.json not found. Run the C++ planner first to generate the path data.")
        return

    map_file = data["map_file"]
    starts = np.array(data["starts"])
    goals = np.array(data["goals"])
    paths = [np.array(path) for path in data["paths"]]

    # Load point cloud map
    points = read_pcd_binary(map_file)
    
    # Downsample points for matplotlib visualization speed
    downsample_rate = 300
    points_ds = points[::downsample_rate]
    print(f"Downsampled points to {len(points_ds)} for fast rendering.")

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Set labels and title
    ax.set_title("Multi-UAVs Cramped Environment Path Planning")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    # Plot map point cloud as obstacles (gray dots)
    ax.scatter(points_ds[:, 0], points_ds[:, 1], points_ds[:, 2], 
               c='gray', s=1, alpha=0.3, label='Obstacles (Point Cloud)')

    colors = ['cyan', 'magenta', 'lime', 'orange', 'yellow']
    
    # Plot starts, goals, and static paths
    for i, path in enumerate(paths):
        c = colors[i % len(colors)]
        # Start (Green marker)
        ax.scatter(starts[i, 0], starts[i, 1], starts[i, 2], c='green', marker='o', s=80, edgecolors='black')
        ax.text(starts[i, 0], starts[i, 1], starts[i, 2] + 0.3, f"Start {i}", color='green', weight='bold')
        
        # Goal (Red marker)
        ax.scatter(goals[i, 0], goals[i, 1], goals[i, 2], c='red', marker='*', s=120, edgecolors='black')
        ax.text(goals[i, 0], goals[i, 1], goals[i, 2] + 0.3, f"Goal {i}", color='red', weight='bold')
        
        # Path line
        ax.plot(path[:, 0], path[:, 1], path[:, 2], c=c, linewidth=2, label=f"UAV {i} Path")

    # Set viewport bounds based on starts and goals
    ax.set_xlim(min(points_ds[:, 0]), max(points_ds[:, 0]))
    ax.set_ylim(min(points_ds[:, 1]), max(points_ds[:, 1]))
    ax.set_zlim(0, 6)

    plt.legend()
    
    # Animation setup
    max_len = max(len(p) for p in paths)
    
    # Drone trackers (animated points)
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

    print("Starting path animation...")
    ani = FuncAnimation(fig, update, frames=max_len, interval=100, blit=True)
    plt.show()

if __name__ == "__main__":
    main()
