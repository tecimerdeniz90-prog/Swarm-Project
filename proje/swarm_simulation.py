import numpy as np
from ecbs import ECBS

class SwarmSimulation:
    def __init__(self, agents, voxel_map, detector, communication_range=5.0):
        self.agents = agents
        self.voxel_map = voxel_map
        self.detector = detector
        self.communication_range = communication_range
        self.history = []  # Her adımda tüm İHA'ların konumları
        self.communication_links = []  # Her adımda aktif haberleşme bağları
        self.leader_history = []  # Her adımda aktif liderlerin listesi
        self.max_steps = 100

    def run_simulation(self):
        """Tüm P2P sürü simülasyonunu adım adım çalıştırır."""
        print("====== Dinamik Liderlikli P2P Sürü İHA Simülasyonu Başlatılıyor ======")
        
        # 1. Başlangıç rotalarını diğer araçları hesaba katmadan planla
        for agent in self.agents:
            success = agent.plan_initial_path(self.voxel_map, self.detector)
            if not success:
                print(f"UAV {agent.agent_id} başlangıç rotasını çizemedi. Simülasyon iptal edildi.")
                return False

        # 2. Adım adım zamanı ilerlet
        step = 0
        all_reached = False
        
        while step < self.max_steps and not all_reached:
            print(f"\n--- Simülasyon Adımı {step} ---")
            
            # Adım hesaplaması (P2P haberleşme, lider seçimi ve çakışma çözümü)
            active_links, active_leaders = self.simulation_step(step)
            
            # Durumu görselleştirme için kaydet
            step_positions = [agent.get_position_at_step(step).tolist() for agent in self.agents]
            self.history.append(step_positions)
            self.communication_links.append(active_links)
            self.leader_history.append(active_leaders)
            
            # İHA konumlarını terminale yazdır
            for agent in self.agents:
                pos = agent.get_position_at_step(step)
                print(f"UAV {agent.agent_id}: Konum={pos.round(2)} | Hedef={agent.goal.round(2)}")
            
            # Tüm İHA'ların hedefe ulaşıp ulaşmadığını kontrol et
            all_reached = True
            for agent in self.agents:
                pos = agent.get_position_at_step(step)
                dist = np.linalg.norm(pos - agent.goal)
                if dist > 1.0:  # Hedef toleransı
                    all_reached = False
                    break
            
            step += 1
            
        if all_reached:
            print(f"\nBaşarılı: Tüm İHA'lar {step} adımda hedeflerine ulaştı!")
            return True
        else:
            print(f"\nUyarı: Simülasyon maksimum adıma ulaştığı için sonlandırıldı ({self.max_steps}).")
            return False

    def simulation_step(self, current_step):
        """Tek bir simülasyon zaman adımını yürütür."""
        num_agents = len(self.agents)
        active_links = []
        active_leaders = []
        
        # 1. Rolleri varsayılan olarak PEER (Standart Üye) yap
        for agent in self.agents:
            agent.role = "PEER"
            agent.cluster_id = None
        
        # 2. P2P Haberleşme: Menzil içindeki komşuları tespit et
        neighbors = {i: [] for i in range(num_agents)}
        for i in range(num_agents):
            pos_i = self.agents[i].get_position_at_step(current_step)
            for j in range(i + 1, num_agents):
                pos_j = self.agents[j].get_position_at_step(current_step)
                dist = np.linalg.norm(pos_i - pos_j)
                if dist <= self.communication_range:
                    neighbors[i].append(j)
                    neighbors[j].append(i)
                    active_links.append((i, j))

        # 3. Çakışma Tespiti: Menzil içindeki komşularla gelecek rotaların çakışıp çakışmadığını kontrol et
        conflicts = []
        for i in range(num_agents):
            path_i = self.agents[i].planned_path
            for j in neighbors[i]:
                if j > i:  # Çift kontrolü engelle
                    path_j = self.agents[j].planned_path
                    max_len = max(len(path_i), len(path_j))
                    for t in range(current_step, max_len):
                        pt_i = path_i[t] if t < len(path_i) else path_i[-1]
                        pt_j = path_j[t] if t < len(path_j) else path_j[-1]
                        if self.detector.check_agent_collision(pt_i[0], pt_i[1], pt_i[2], pt_j[0], pt_j[1], pt_j[2]):
                            conflicts.append((i, j))
                            break

        # 4. Çakışmaları kümelere grupla (Connected Components)
        clusters = []
        visited = set()
        
        def find_connected_component(node, cluster):
            visited.add(node)
            cluster.append(node)
            for neighbor in neighbors[node]:
                if (node, neighbor) in conflicts or (neighbor, node) in conflicts:
                    if neighbor not in visited:
                        find_connected_component(neighbor, cluster)
        
        for i in range(num_agents):
            if i not in visited:
                has_conflict = any(i in pair for pair in conflicts)
                if has_conflict:
                    cluster = []
                    find_connected_component(i, cluster)
                    if len(cluster) > 1:
                        clusters.append(cluster)
        
        # 5. Her Küme İçinde Yerel Koordinasyon (Dinamik Liderlik)
        for cluster_idx, cluster in enumerate(clusters):
            # Lider Seçimi: En düşük ID'li İHA lider seçilir
            leader_id = min(cluster)
            active_leaders.append(leader_id)
            
            cluster_agents = [self.agents[idx] for idx in cluster]
            leader_agent = self.agents[leader_id]
            leader_agent.role = "LEADER"
            
            for agent in cluster_agents:
                agent.cluster_id = cluster_idx
                
            print(f"Küme {cluster_idx} Oluştu: {cluster} | Seçilen Lider: UAV {leader_id}")
            
            # Lider İHA, küme için yerel ECBS çalıştırır
            local_ecbs = ECBS(w_high=1.5, w_low=1.4)
            
            starts_sub = [agent.get_position_at_step(current_step).tolist() for agent in cluster_agents]
            goals_sub = [agent.goal.tolist() for agent in cluster_agents]
            
            print(f"  Lider UAV {leader_id} küme için yerel ECBS planlaması başlatıyor...")
            solutions = local_ecbs.plan(starts_sub, goals_sub, self.voxel_map, self.detector)
            
            if solutions is not None:
                print(f"  Yerel koordinasyon başarılı. Küme {cluster} rotaları güncellendi.")
                for idx, agent in enumerate(cluster_agents):
                    agent.planned_path = agent.planned_path[:current_step] + solutions[idx]
            else:
                print(f"  Uyarı: Yerel ECBS başarısız oldu. İHA'lar bağımsız kaçınma deneyecek.")
                for agent in cluster_agents:
                    other_paths = [self.agents[idx].planned_path for idx in cluster if idx != agent.agent_id]
                    agent.replan_locally(current_step, self.voxel_map, self.detector, other_paths)

        return active_links, active_leaders