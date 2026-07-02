import json
from voxel_map import VoxelMap
from collision_detector import CollisionDetector
from uav_agent import UAVAgent
from swarm_simulation import SwarmSimulation
from visualize_swarm import visualize_swarm

def main():
    print("====== P2P Swarm İHA Yol Planlayıcı ======")
    
    # 1. Haritayı Yükle
    map_path = "Map/10X30m.pcd"
    voxel_map = VoxelMap(resolution=0.2)
    if not voxel_map.load_map(map_path):
        print("Harita yüklenemedi.")
        return

    # 2. Çarpışma Dedektörünü Kur (Elipsoid Downwash Modeli)
    detector = CollisionDetector(rx=0.15, ry=0.15, rz=0.5)

    # 3. 60 İHA için Otomatik Engelsiz Nokta Üretici (Karşı Karşıya Uçuş Konfigürasyonu)
    import numpy as np
    
    def generate_swarm_grid(voxel_map, detector, num_agents=60):
        starts = []
        goals = []
        half = num_agents // 2
        
        # Lansman/kurtarma açık alanlarındaki (foyer) grid yerleşim ayarları
        xs = np.linspace(-9.0, 0.0, 7)
        ys_bottom = [-13.0, -12.0, -11.0]
        ys_top = [11.0, 12.0, 13.0]
        zs = [1.5, 2.5, 3.5, 4.5]
        
        bottom_pts = []
        top_pts = []
        
        # Alt açık alandaki adayları üret
        for y in ys_bottom:
            for x in xs:
                for z in zs:
                    if not detector.check_obstacle_collision(voxel_map, float(x), float(y), float(z)):
                        # Çok yakın kopyaları engellemek için mesafe kontrolü yap
                        if not any(np.linalg.norm(np.array([x, y, z]) - np.array(p)) < 0.6 for p in bottom_pts):
                            bottom_pts.append([float(x), float(y), float(z)])
                        
        # Üst açık alandaki adayları üret
        for y in ys_top:
            for x in xs:
                for z in zs:
                    if not detector.check_obstacle_collision(voxel_map, float(x), float(y), float(z)):
                        # Çok yakın kopyaları engellemek için mesafe kontrolü yap
                        if not any(np.linalg.norm(np.array([x, y, z]) - np.array(p)) < 0.6 for p in top_pts):
                            top_pts.append([float(x), float(y), float(z)])
                            
        # Eğer grid yeterli nokta üretmezse araya titreşimli yedek (fallback) arama algoritmasını sok
        if len(bottom_pts) < half or len(top_pts) < half:
            for _ in range(500):
                if len(bottom_pts) >= half and len(top_pts) >= half:
                    break
                
                x = np.random.uniform(-9.0, 0.0)
                z = np.random.uniform(1.2, 5.0)
                
                # Alt açık alan için yedek arama
                if len(bottom_pts) < half:
                    y = np.random.uniform(-13.5, -10.5)
                    if not detector.check_obstacle_collision(voxel_map, x, y, z):
                        if not any(np.linalg.norm(np.array([x, y, z]) - np.array(p)) < 0.6 for p in bottom_pts):
                            bottom_pts.append([float(x), float(y), float(z)])
                            
                # Üst açık alan için yedek arama
                if len(top_pts) < half:
                    y = np.random.uniform(10.5, 13.5)
                    if not detector.check_obstacle_collision(voxel_map, x, y, z):
                        if not any(np.linalg.norm(np.array([x, y, z]) - np.array(p)) < 0.6 for p in top_pts):
                            top_pts.append([float(x), float(y), float(z)])
                        
        # Gerekli İHA sayısı kadarını kırp (Tam 30'ar adet olacak şekilde)
        bottom_pts = bottom_pts[:half]
        top_pts = top_pts[:half]
        
        # Grup 1: Aşağıdan Yukarıya (30 İHA)
        for i in range(half):
            starts.append(bottom_pts[i])
            goals.append(top_pts[i])
            
        # Grup 2: Yukarıdan Aşağıya (30 İHA)
        for i in range(half):
            starts.append(top_pts[i])
            goals.append(bottom_pts[i])
            
        return starts, goals

    starts, goals = generate_swarm_grid(voxel_map, detector, num_agents=60)
    print(f"60 İHA için engelsiz {len(starts)} adet başlangıç ve hedef noktası üretildi.")

    # 4. İHA Ajanlarını Tanımla (Haberleşme yarıçapı = 5.0 metre)
    agents = []
    for i in range(len(starts)):
        agent = UAVAgent(agent_id=i, start=starts[i], goal=goals[i], communication_range=5.0)
        agents.append(agent)

    # 5. Sürü Simülasyonunu Başlat
    sim = SwarmSimulation(agents, voxel_map, detector, communication_range=5.0)
    success = sim.run_simulation()

    if not success:
        if len(sim.history) == 0:
            print("Sürü simülasyonu başlatılırken hata oluştu. Görselleştirme iptal ediliyor.")
            return
        print("Sürü simülasyonu uyarılarla tamamlandı. Veriler kaydediliyor ve GIF oluşturuluyor...")

    # 6. Sürü Rota Verilerini paths_swarm.json Olarak Kaydet
    swarm_paths = [agent.planned_path for agent in agents]
    output_data = {
        "map_file": map_path,
        "starts": starts,
        "goals": goals,
        "paths": swarm_paths,
        "history": sim.history,
        "communication_links": sim.communication_links,
        "leader_history": sim.leader_history
    }
    with open("paths_swarm.json", "w") as f:
        json.dump(output_data, f, indent=2)
    print("Sürü rotaları 'paths_swarm.json' dosyasına kaydedildi.")

    # 7. Çıktıyı simulation_swarm.gif Olarak Görselleştir
    visualize_swarm(map_path, starts, goals, sim.history, sim.communication_links, sim.leader_history, "simulation_swarm.gif")

if __name__ == "__main__":
    main()