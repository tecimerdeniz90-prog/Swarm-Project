import rclpy
from rclpy.node import Node
import json
import os
import time
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Point, Quaternion

class UAVSimNode(Node):
    def __init__(self):
        super().__init__('uav_sim_node')
        self.get_logger().info("Initializing UAV Gazebo Trajectory Controller...")
        
        # Paths file path inside WSL
        self.paths_path = "/mnt/c/Users/tecim/.gemini/antigravity-ide/scratch/multi-uav-path-planning/paths.json"
        
        # Load paths
        self.paths = []
        self.load_paths()
        
        # Step counter
        self.step_idx = 0
        self.max_steps = 0
        if self.paths:
            self.max_steps = max(len(p) for p in self.paths)
            self.get_logger().info(f"Loaded paths for {len(self.paths)} UAVs with max length of {self.max_steps} steps.")
            
        # Create client for Gazebo set_entity_state service
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.get_logger().info("Waiting for /gazebo/set_entity_state service...")
        self.client.wait_for_service()
        self.get_logger().info("Service connected.")
        
        # Run timer at 8Hz (0.12 seconds per step) for a more realistic speed
        self.timer = self.create_timer(0.12, self.timer_callback)
        
    def load_paths(self):
        # Wait a bit in case paths.json is being written
        for _ in range(5):
            if os.path.exists(self.paths_path):
                try:
                    with open(self.paths_path, 'r') as f:
                        data = json.load(f)
                        self.paths = data['paths']
                        return
                except Exception as e:
                    self.get_logger().error(f"Error reading paths file: {e}")
            time.sleep(1.0)
        self.get_logger().error(f"Could not load paths from {self.paths_path}")

    def timer_callback(self):
        if not self.paths:
            return
            
        # Loop simulation if all paths are finished
        if self.step_idx >= self.max_steps + 40: # 40 steps delay at the end
            self.get_logger().info("Simulation loop finished, resetting to start positions.")
            self.step_idx = 0
            
        for i, path in enumerate(self.paths):
            # Clamp step index to last point in path if finished
            idx = min(self.step_idx, len(path) - 1)
            pos = path[idx]
            
            # Send set entity state request
            req = SetEntityState.Request()
            req.state.name = f'uav_{i}'
            req.state.reference_frame = 'world'
            
            # Set position
            pose = Pose()
            pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
            # Neutral rotation
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            req.state.pose = pose
            
            # Call async service
            self.client.call_async(req)
            
        self.step_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = UAVSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
