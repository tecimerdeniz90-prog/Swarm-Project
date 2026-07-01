import struct
import math

def load_pcd(file_path):
    points = []
    with open(file_path, 'rb') as f:
        while True:
            line = f.readline()
            if b"DATA binary" in line:
                break
        data = f.read()
        num_points = len(data) // 20
        for i in range(num_points):
            offset = i * 20
            idx, x, y, z = struct.unpack_from('ffff', data, offset)
            points.append((x, y, z))
    return points

def main():
    print("Loading PCD map...")
    points = load_pcd("Map/10X30m.pcd")
    print(f"Loaded {len(points)} points.")

    # Voxelize with a coarse resolution (e.g., 0.8m) to avoid Gazebo lag
    resolution = 0.8
    occupied = set()
    for p in points:
        gx = int(round(p[0] / resolution))
        gy = int(round(p[1] / resolution))
        gz = int(round(p[2] / resolution))
        occupied.add((gx, gy, gz))

    print(f"Coarse voxelization resulted in {len(occupied)} occupied cells.")

    # Generate Gazebo World XML
    world_xml = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="cramped_environment">
    <!-- ROS 2 State Plugin -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
    </plugin>

    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Obstacles representing the PCD Map -->
"""

    box_count = 0
    for idx, (gx, gy, gz) in enumerate(occupied):
        x = gx * resolution
        y = gy * resolution
        z = gz * resolution
        
        # Limit the number of boxes to avoid crashing Gazebo (max 400 boxes)
        # We also skip boxes below the ground (z < 0)
        if z < 0 or box_count > 400:
            continue
            
        world_xml += f"""    <model name="obs_{box_count}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{resolution:.2f} {resolution:.2f} {resolution:.2f}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <transparency>0.5</transparency>
          <geometry>
            <box>
              <size>{resolution:.2f} {resolution:.2f} {resolution:.2f}</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
      </link>
    </model>
"""
        box_count += 1

    world_xml += """  </world>
</sdf>
"""

    with open("cramped_env.world", "w") as f:
        f.write(world_xml)
    print(f"Successfully generated cramped_env.world with {box_count} obstacles.")

if __name__ == "__main__":
    main()
