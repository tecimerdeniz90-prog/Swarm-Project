import struct
import numpy as np
import math

class VoxelMap:
    def __init__(self, resolution=0.2):
        self.resolution = resolution
        self.min_pt = [float('inf'), float('inf'), float('inf')]
        self.max_pt = [float('-inf'), float('-inf'), float('-inf')]
        self.occupied_voxels = set()
        self.dim_x = 0
        self.dim_y = 0
        self.dim_z = 0

    def load_map(self, filepath):
        print(f"VoxelMap: Loading point cloud map: {filepath}...")
        try:
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
            points = np.array(points)
        except Exception as e:
            print(f"Error loading PCD file: {e}")
            return False

        # Calculate bounding box using NumPy for speed on the large array
        min_vals = np.min(points, axis=0) - self.resolution
        max_vals = np.max(points, axis=0) + self.resolution
        
        # Convert to normal python float lists for zero scalar overhead
        self.min_pt = [float(min_vals[0]), float(min_vals[1]), float(min_vals[2])]
        self.max_pt = [float(max_vals[0]), float(max_vals[1]), float(max_vals[2])]
        
        self.dim_x = int(math.ceil((self.max_pt[0] - self.min_pt[0]) / self.resolution)) + 1
        self.dim_y = int(math.ceil((self.max_pt[1] - self.min_pt[1]) / self.resolution)) + 1
        self.dim_z = int(math.ceil((self.max_pt[2] - self.min_pt[2]) / self.resolution)) + 1

        print(f"Grid bounds: {self.min_pt} to {self.max_pt}")
        print(f"Voxelizing {len(points)} points into grid ({self.dim_x} x {self.dim_y} x {self.dim_z})...")

        for pt in points:
            ix, iy, iz = self.pos_to_idx(pt[0], pt[1], pt[2])
            self.occupied_voxels.add((ix, iy, iz))

        print("Voxelization completed successfully.")
        return True

    def pos_to_idx(self, x, y, z):
        # Using built-in round is ~20x faster than np.round for scalar math
        ix = int(round((x - self.min_pt[0]) / self.resolution))
        iy = int(round((y - self.min_pt[1]) / self.resolution))
        iz = int(round((z - self.min_pt[2]) / self.resolution))
        return ix, iy, iz

    def idx_to_pos(self, ix, iy, iz):
        x = self.min_pt[0] + ix * self.resolution
        y = self.min_pt[1] + iy * self.resolution
        z = self.min_pt[2] + iz * self.resolution
        return x, y, z

    def is_out_of_bounds(self, x, y, z):
        ix, iy, iz = self.pos_to_idx(x, y, z)
        return (ix < 0 or ix >= self.dim_x or 
                iy < 0 or iy >= self.dim_y or 
                iz < 0 or iz >= self.dim_z)

    def is_occupied(self, x, y, z):
        if self.is_out_of_bounds(x, y, z):
            return True
        ix, iy, iz = self.pos_to_idx(x, y, z)
        return (ix, iy, iz) in self.occupied_voxels
