import math

class PathNode:
    def __init__(self, state, parent=None, g=0.0, h=0.0, conflicts=0):
        self.state = state  # (x, y, z, vx, vy, vz, step)
        self.parent = parent
        self.g = g
        self.h = h
        self.f = g + h
        self.conflicts = conflicts

    def __lt__(self, other):
        if self.conflicts != other.conflicts:
            return self.conflicts < other.conflicts
        return self.h < other.h

class KinodynamicAStar:
    def __init__(self, dt=0.5, v_max=2.0, a_max=1.0, w=1.4):
        self.dt = dt
        self.v_max = v_max
        self.a_max = a_max
        self.w = w

    def calculate_heuristic(self, state, goal):
        dx = state[0] - goal[0]
        dy = state[1] - goal[1]
        dz = state[2] - goal[2]
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        return dist / self.v_max

    def count_conflicts(self, state, other_paths, detector):
        conflicts = 0
        step = state[6]
        for path in other_paths:
            if step < len(path):
                other_pt = path[step]
                if detector.check_agent_collision(state[0], state[1], state[2], other_pt[0], other_pt[1], other_pt[2]):
                    conflicts += 1
            elif len(path) > 0:
                other_pt = path[-1]
                if detector.check_agent_collision(state[0], state[1], state[2], other_pt[0], other_pt[1], other_pt[2]):
                    conflicts += 1
        return conflicts

    def violates_constraints(self, state, constraints, detector):
        step = state[6]
        for c in constraints:
            if c['step'] == step:
                if detector.check_agent_collision(state[0], state[1], state[2], c['x'], c['y'], c['z']):
                    return True
        return False

    def check_transition(self, start, end, voxel_map, detector):
        num_steps = 5
        for i in range(1, num_steps + 1):
            t = i / num_steps
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            z = start[2] + t * (end[2] - start[2])
            if detector.check_obstacle_collision(voxel_map, x, y, z):
                return False
        return True

    def state_to_key(self, state, use_time=True):
        res = 0.4
        ix = int(round(state[0] / res))
        iy = int(round(state[1] / res))
        iz = int(round(state[2] / res))
        ivx = int(round(state[3] / res))
        ivy = int(round(state[4] / res))
        ivz = int(round(state[5] / res))
        if use_time:
            return (ix, iy, iz, ivx, ivy, ivz, state[6])
        return (ix, iy, iz, ivx, ivy, ivz)

    def search(self, start, goal, voxel_map, detector, constraints, other_paths):
        start_state = (start[0], start[1], start[2], 0.0, 0.0, 0.0, 0)
        
        if detector.check_obstacle_collision(voxel_map, start[0], start[1], start[2]):
            print("Error: Start position in collision.")
            return None

        start_h = self.calculate_heuristic(start_state, goal)
        start_conf = self.count_conflicts(start_state, other_paths, detector)
        start_node = PathNode(start_state, None, 0.0, start_h, start_conf)

        dist_threshold = 1.0
        vel_threshold = 1.0
        max_iterations = 20000
        iterations = 0

        # Unified heap-based search for speed and conflict prioritization
        import heapq
        node_counter = 0
        
        f_weighted = start_node.g + self.w * start_node.h
        open_heap = [(start_node.conflicts, f_weighted, start_node.h, node_counter, start_node)]
        closed_set = set()

        pruned_speed = 0
        pruned_collision = 0
        pruned_closed = 0
        pruned_transition = 0
        added = 0
        max_y_reached = start[1]

        use_time = True if (constraints or other_paths) else False

        while open_heap and iterations < max_iterations:
            iterations += 1
            _, _, _, _, best_node = heapq.heappop(open_heap)

            key = self.state_to_key(best_node.state, use_time=use_time)
            if key in closed_set:
                continue
            closed_set.add(key)

            curr_state = best_node.state
            if curr_state[1] > max_y_reached:
                max_y_reached = curr_state[1]

            dist_to_goal = math.sqrt((curr_state[0] - goal[0])**2 + (curr_state[1] - goal[1])**2 + (curr_state[2] - goal[2])**2)
            vel_mag = math.sqrt(curr_state[3]**2 + curr_state[4]**2 + curr_state[5]**2)

            if dist_to_goal < dist_threshold and vel_mag < vel_threshold:
                path = []
                temp = best_node
                while temp is not None:
                    path.append([temp.state[0], temp.state[1], temp.state[2]])
                    temp = temp.parent
                path.reverse()
                return path

            acc_options = [-self.a_max, 0.0, self.a_max]
            for ax in acc_options:
                for ay in acc_options:
                    for az in acc_options:
                        next_vx = curr_state[3] + ax * self.dt
                        next_vy = curr_state[4] + ay * self.dt
                        next_vz = curr_state[5] + az * self.dt
                        
                        next_x = curr_state[0] + curr_state[3] * self.dt + 0.5 * ax * self.dt**2
                        next_y = curr_state[1] + curr_state[4] * self.dt + 0.5 * ay * self.dt**2
                        next_z = curr_state[2] + curr_state[5] * self.dt + 0.5 * az * self.dt**2
                        next_step = curr_state[6] + 1
                        
                        speed = math.sqrt(next_vx**2 + next_vy**2 + next_vz**2)
                        if speed > self.v_max:
                            pruned_speed += 1
                            continue

                        if detector.check_obstacle_collision(voxel_map, next_x, next_y, next_z):
                            pruned_collision += 1
                            continue

                        next_state = (next_x, next_y, next_z, next_vx, next_vy, next_vz, next_step)
                        
                        if self.violates_constraints(next_state, constraints, detector):
                            continue

                        if self.state_to_key(next_state, use_time=use_time) in closed_set:
                            pruned_closed += 1
                            continue

                        if not self.check_transition(curr_state, next_state, voxel_map, detector):
                            pruned_transition += 1
                            continue

                        u_mag_sq = ax**2 + ay**2 + az**2
                        cos_theta = 1.0
                        v_mag_curr = math.sqrt(curr_state[3]**2 + curr_state[4]**2 + curr_state[5]**2)
                        if v_mag_curr > 0.001 and speed > 0.001:
                            cos_theta = (curr_state[3]*next_vx + curr_state[4]*next_vy + curr_state[5]*next_vz) / (v_mag_curr * speed)
                        dir_change = 1.0 - cos_theta
                        
                        step_cost = (0.1 * u_mag_sq + 1.0 * dir_change + 1.0) * self.dt
                        child_g = best_node.g + step_cost
                        child_h = self.calculate_heuristic(next_state, goal)
                        child_conf = best_node.conflicts + self.count_conflicts(next_state, other_paths, detector)
                        
                        node_counter += 1
                        added += 1
                        child_node = PathNode(next_state, best_node, child_g, child_h, child_conf)
                        
                        child_f_weighted = child_g + self.w * child_h
                        heapq.heappush(open_heap, (child_node.conflicts, child_f_weighted, child_node.h, node_counter, child_node))
                        
        print(f"  [A* Search] Failed to find path. Iterations: {iterations}, open_heap: {len(open_heap)}, closed_set: {len(closed_set)}, max_y: {max_y_reached:.3f}")
        print(f"    Pruned by: Speed: {pruned_speed}, Collision: {pruned_collision}, Closed: {pruned_closed}, Transition: {pruned_transition}, Added: {added}")
        return None
