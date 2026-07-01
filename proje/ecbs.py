import numpy as np
from kinodynamic_astar import KinodynamicAStar

class CTNode:
    def __init__(self, num_agents):
        self.paths = [[] for _ in range(num_agents)]
        self.constraints = [[] for _ in range(num_agents)]
        self.cost = 0.0
        self.conflicts = 0

class ECBS:
    def __init__(self, w_high=1.5, w_low=1.4):
        self.w_high = w_high
        self.w_low = w_low

    def count_all_conflicts(self, paths, detector):
        num_agents = len(paths)
        conflicts = 0
        for i in range(num_agents):
            for j in range(i + 1, num_agents):
                if not paths[i] or not paths[j]:
                    continue
                max_size = max(len(paths[i]), len(paths[j]))
                for t in range(max_size):
                    pi = paths[i][t] if t < len(paths[i]) else paths[i][-1]
                    pj = paths[j][t] if t < len(paths[j]) else paths[j][-1]
                    if detector.check_agent_collision(pi[0], pi[1], pi[2], pj[0], pj[1], pj[2]):
                        conflicts += 1
        return conflicts

    def find_first_conflict(self, paths, detector):
        num_agents = len(paths)
        for i in range(num_agents):
            for j in range(i + 1, num_agents):
                if not paths[i] or not paths[j]:
                    continue
                max_size = max(len(paths[i]), len(paths[j]))
                for t in range(max_size):
                    pi = paths[i][t] if t < len(paths[i]) else paths[i][-1]
                    pj = paths[j][t] if t < len(paths[j]) else paths[j][-1]
                    if detector.check_agent_collision(pi[0], pi[1], pi[2], pj[0], pj[1], pj[2]):
                        return i, j, t, pi, pj
        return None

    def plan(self, starts, goals, voxel_map, detector):
        M = len(starts)
        if M == 0:
            return []

        root = CTNode(M)
        planner = KinodynamicAStar(dt=0.5, v_max=2.0, a_max=1.0, w=self.w_low)

        print(f"ECBS: Planning initial paths for {M} agents...")
        for i in range(M):
            path = planner.search(starts[i], goals[i], voxel_map, detector, [], [])
            if path is None:
                print(f"ECBS: Failed to find initial path for agent {i}")
                return None
            root.paths[i] = path

        root.cost = sum(len(p) for p in root.paths)
        root.conflicts = self.count_all_conflicts(root.paths, detector)

        open_list = [root]
        max_ct_nodes = 500
        ct_nodes_expanded = 0

        while open_list and ct_nodes_expanded < max_ct_nodes:
            ct_nodes_expanded += 1
            
            cost_min = min(node.cost for node in open_list)
            
            focal_bound = cost_min * self.w_high
            focal_nodes = [(node, idx) for idx, node in enumerate(open_list) if node.cost <= focal_bound]
            
            curr, best_idx = min(focal_nodes, key=lambda x: (x[0].conflicts, x[0].cost))
            open_list.pop(best_idx)

            print(f"CT Node Expanded: {ct_nodes_expanded} | Cost: {curr.cost} | Conflicts: {curr.conflicts}")

            conflict = self.find_first_conflict(curr.paths, detector)
            if conflict is None:
                print(f"Conflict-free solution found after expanding {ct_nodes_expanded} nodes.")
                return curr.paths

            agent_i, agent_j, step, pi, pj = conflict
            print(f"  Conflict between agent {agent_i} and {agent_j} at step {step}")

            # Branch 1
            child1 = CTNode(M)
            child1.paths = [list(p) for p in curr.paths]
            child1.constraints = [list(c) for c in curr.constraints]
            child1.constraints[agent_i].append({'step': step, 'x': pj[0], 'y': pj[1], 'z': pj[2]})
            
            other_paths = [child1.paths[k] for k in range(M) if k != agent_i]
            new_path_i = planner.search(starts[agent_i], goals[agent_i], voxel_map, detector, 
                                        child1.constraints[agent_i], other_paths)
            if new_path_i is not None:
                child1.paths[agent_i] = new_path_i
                child1.cost = sum(len(p) for p in child1.paths)
                child1.conflicts = self.count_all_conflicts(child1.paths, detector)
                open_list.append(child1)

            # Branch 2
            child2 = CTNode(M)
            child2.paths = [list(p) for p in curr.paths]
            child2.constraints = [list(c) for c in curr.constraints]
            child2.constraints[agent_j].append({'step': step, 'x': pi[0], 'y': pi[1], 'z': pi[2]})
            
            other_paths = [child2.paths[k] for k in range(M) if k != agent_j]
            new_path_j = planner.search(starts[agent_j], goals[agent_j], voxel_map, detector, 
                                        child2.constraints[agent_j], other_paths)
            if new_path_j is not None:
                child2.paths[agent_j] = new_path_j
                child2.cost = sum(len(p) for p in child2.paths)
                child2.conflicts = self.count_all_conflicts(child2.paths, detector)
                open_list.append(child2)

        print("ECBS: Failed to find conflict-free paths.")
        return None
