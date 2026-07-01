import numpy as np
from kinodynamic_astar import KinodynamicAStar

class UAVAgent:
    def __init__(self, agent_id, start, goal, communication_range=5.0):
        self.agent_id = agent_id
        self.start = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.current_pos = np.array(start, dtype=float)
        self.current_vel = np.array([0.0, 0.0, 0.0], dtype=float)
        
        self.communication_range = communication_range
        self.planned_path = []  # Rotayı temsil eden [x, y, z] koordinat listesi
        self.cluster_id = None
        self.role = "PEER"  # "PEER" veya "LEADER" olabilir
        
        # Yerel Kinodynamic A* planlayıcısı
        self.planner = KinodynamicAStar(dt=0.5, v_max=2.0, a_max=1.0, w=1.4)

    def plan_initial_path(self, voxel_map, detector):
        """Diğer İHA'ları göz ardı ederek başlangıçtan hedefe ilk rotayı çizer."""
        path = self.planner.search(self.start, self.goal, voxel_map, detector, [], [])
        if path is not None:
            self.planned_path = path
            return True
        return False

    def get_position_at_step(self, step):
        """Belirli bir simülasyon adımındaki İHA konumunu döner."""
        if not self.planned_path:
            return self.current_pos
        if step < len(self.planned_path):
            return np.array(self.planned_path[step])
        return np.array(self.planned_path[-1])

    def get_remaining_path(self, current_step):
        """Mevcut adımdan itibaren kalan rotayı döner."""
        if not self.planned_path:
            return []
        if current_step < len(self.planned_path):
            return self.planned_path[current_step:]
        return [self.planned_path[-1]]

    def replan_locally(self, current_step, voxel_map, detector, neighbor_paths, constraints=None):
        """Komşu İHA'ları engel kabul ederek mevcut konumdan hedefe yeniden rota çizer."""
        if constraints is None:
            constraints = []
        
        start_pos = self.get_position_at_step(current_step)
        
        other_paths_formatted = []
        for n_path in neighbor_paths:
            if len(n_path) > current_step:
                other_paths_formatted.append(n_path[current_step:])
            elif len(n_path) > 0:
                other_paths_formatted.append([n_path[-1]])
        
        local_constraints = []
        for c in constraints:
            local_c = c.copy()
            local_c['step'] = max(0, c['step'] - current_step)
            local_constraints.append(local_c)

        new_path = self.planner.search(start_pos, self.goal, voxel_map, detector, local_constraints, other_paths_formatted)
        if new_path is not None:
            past_path = self.planned_path[:current_step]
            self.planned_path = past_path + new_path
            return True
        return False