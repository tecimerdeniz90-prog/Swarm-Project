import numpy as np

class CollisionDetector:
    def __init__(self, rx=0.15, ry=0.15, rz=0.5):
        self.rx = rx
        self.ry = ry
        self.rz = rz

    def check_obstacle_collision(self, voxel_map, x, y, z):
        # Point-sampling approximation for fast O(1) checks in Python.
        # We sample 9 key points inside/on the boundary of the ellipsoid:
        # Center, 2 points along Z (up/down), and 4 points along X/Y (front/back/left/right).
        sample_offsets = [
            (0.0, 0.0, 0.0),
            (0.0, 0.0, self.rz * 0.5),
            (0.0, 0.0, self.rz),
            (0.0, 0.0, -self.rz * 0.5),
            (0.0, 0.0, -self.rz),
            (self.rx, 0.0, 0.0),
            (-self.rx, 0.0, 0.0),
            (0.0, self.ry, 0.0),
            (0.0, -self.ry, 0.0)
        ]
        
        for dx, dy, dz in sample_offsets:
            px, py, pz = x + dx, y + dy, z + dz
            if voxel_map.is_out_of_bounds(px, py, pz):
                return True
            ix, iy, iz = voxel_map.pos_to_idx(px, py, pz)
            if (ix, iy, iz) in voxel_map.occupied_voxels:
                return True
                
        return False

    def check_agent_collision(self, x1, y1, z1, x2, y2, z2):
        dx = x1 - x2
        dy = y1 - y2
        dz = z1 - z2

        rx_double = 2.0 * self.rx
        ry_double = 2.0 * self.ry
        rz_double = 2.0 * self.rz

        dist_val = (dx ** 2) / (rx_double ** 2) + \
                   (dy ** 2) / (ry_double ** 2) + \
                   (dz ** 2) / (rz_double ** 2)
        return dist_val <= 1.0
