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

    # 3. Başlangıç ve Hedef Noktaları (Karşılaştırma için main.py ile aynı)
    starts = [
        [-6.5, -10.0, 1.5],
        [-3.8, -10.0, 2.4],
        [-1.6, -10.0, 2.2]
    ]

    goals = [
        [-9.0, 10.0, 2.0],
        [-6.4, 10.0, 2.6],
        [-7.0, 10.0, 2.0]
    ]

    # Başlangıç ve hedeflerin engelsiz olduğunu doğrula
    for i in range(len(starts)):
        s_coll = detector.check_obstacle_collision(voxel_map, starts[i][0], starts[i][1], starts[i][2])
        g_coll = detector.check_obstacle_collision(voxel_map, goals[i][0], goals[i][1], goals[i][2])
        print(f"UAV {i}: Başlangıç {starts[i]} Engel Çakışması={s_coll} | Hedef {goals[i]} Engel Çakışması={g_coll}")
        if s_coll or g_coll:
            print("Başlangıç veya hedef koordinatı engelle çakışıyor. İşlem iptal edildi.")
            return

    # 4. İHA Ajanlarını Tanımla (Haberleşme yarıçapı = 5.0 metre)
    agents = []
    for i in range(len(starts)):
        agent = UAVAgent(agent_id=i, start=starts[i], goal=goals[i], communication_range=5.0)
        agents.append(agent)

    # 5. Sürü Simülasyonunu Başlat
    sim = SwarmSimulation(agents, voxel_map, detector, communication_range=5.0)
    success = sim.run_simulation()

    if not success:
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