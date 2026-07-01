import struct
import numpy as np
import matplotlib
matplotlib.use('Agg') # Ekrana pencere açılmasını engeller (Cloud Shell uyumlu)
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

def visualize_swarm(map_path, starts, goals, history, links_history, leader_history, save_path="simulation_swarm.gif"):
    print("Harita noktaları çizim için yükleniyor...")
    points = []
    try:
        with open(map_path, 'rb') as f:
            while True:
                line = f.readline()
                if not line:
                    break
                if b"DATA binary" in line:
                    break
            data = f.read()
            num_points = len(data) // 20
            for i in range(num_points):
                offset = i * 20
                idx, x, y, z = struct.unpack_from('ffff', data, offset)
                points.append([x, y, z])
        points = np.array(points)
    except Exception as e:
        print(f"Harita yüklenirken hata oluştu: {e}")
        points = np.array([[-10, -10, 0], [10, 10, 5]])

    points_ds = points[::300] if len(points) > 0 else np.array([])
    
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Dağıtık Çoklu İHA Sürü Yol Planlama\n(P2P İletişim ve Dinamik Liderlik)", fontsize=14, fontweight='bold', pad=20)
    ax.set_xlabel("X (m)", fontsize=11)
    ax.set_ylabel("Y (m)", fontsize=11)
    ax.set_zlabel("Z (m)", fontsize=11)

    if len(points_ds) > 0:
        ax.scatter(points_ds[:, 0], points_ds[:, 1], points_ds[:, 2], 
                   c='gray', s=1.5, alpha=0.15, label='Engeller (Nokta Bulutu)')

    colors = ['cyan', 'magenta', 'lime', 'orange', 'blue', 'purple']
    num_agents = len(starts)
    
    for i in range(num_agents):
        ax.scatter(starts[i][0], starts[i][1], starts[i][2], c='green', marker='o', s=80, edgecolors='black', zorder=5)
        ax.text(starts[i][0], starts[i][1], starts[i][2] + 0.3, f"Start {i}", color='green', weight='bold', fontsize=9)
        ax.scatter(goals[i][0], goals[i][1], goals[i][2], c='red', marker='*', s=120, edgecolors='black', zorder=5)
        ax.text(goals[i][0], goals[i][1], goals[i][2] + 0.3, f"Goal {i}", color='red', weight='bold', fontsize=9)

    ax.set_xlim(min(points_ds[:, 0]) if len(points_ds) > 0 else -10, max(points_ds[:, 0]) if len(points_ds) > 0 else 10)
    ax.set_ylim(min(points_ds[:, 1]) if len(points_ds) > 0 else -15, max(points_ds[:, 1]) if len(points_ds) > 0 else 15)
    ax.set_zlim(0, 6)

    ax.legend(loc='upper right')

    max_len = len(history)
    
    drone_dots = []
    for i in range(num_agents):
        c = colors[i % len(colors)]
        dot, = ax.plot([], [], [], c=c, marker='o', markersize=7, markeredgecolor='black', zorder=10)
        drone_dots.append(dot)
        
    max_links = 50
    link_lines = []
    for _ in range(max_links):
        line, = ax.plot([], [], [], c='yellow', linestyle='--', linewidth=1.2, alpha=0.5)
        link_lines.append(line)

    def update(frame):
        positions = history[frame]
        links = links_history[frame]
        leaders = leader_history[frame]
        
        for i in range(num_agents):
            pos = positions[i]
            drone_dots[i].set_data([pos[0]], [pos[1]])
            drone_dots[i].set_3d_properties([pos[2]])
            
            # Liderleri büyük kırmızı altıgen (h), üyeleri normal daire (o) olarak çiz
            if i in leaders:
                drone_dots[i].set_color('red')
                drone_dots[i].set_markersize(11)
                drone_dots[i].set_marker('h')
            else:
                drone_dots[i].set_color(colors[i % len(colors)])
                drone_dots[i].set_markersize(7)
                drone_dots[i].set_marker('o')

        num_active_links = len(links)
        for idx in range(max_links):
            if idx < num_active_links:
                uav_a, uav_b = links[idx]
                pos_a = positions[uav_a]
                pos_b = positions[uav_b]
                
                # Lider bağı ise daha kalın turuncu, standart bağ ise ince sarı çiz
                if uav_a in leaders or uav_b in leaders:
                    link_lines[idx].set_color('orange')
                    link_lines[idx].set_linewidth(1.8)
                    link_lines[idx].set_alpha(0.8)
                else:
                    link_lines[idx].set_color('gold')
                    link_lines[idx].set_linewidth(1.0)
                    link_lines[idx].set_alpha(0.4)
                    
                link_lines[idx].set_data([pos_a[0], pos_b[0]], [pos_a[1], pos_b[1]])
                link_lines[idx].set_3d_properties([pos_a[2], pos_b[2]])
            else:
                link_lines[idx].set_data([], [])
                link_lines[idx].set_3d_properties([])

        return drone_dots + link_lines

    print(f"Sürü animasyonu oluşturuluyor ve {save_path} dosyasına kaydediliyor...")
    ani = FuncAnimation(fig, update, frames=max_len, interval=250, blit=False)
    ani.save(save_path, writer='pillow', fps=4)
    print(f"Simülasyon animasyonu başarıyla kaydedildi: '{save_path}'!")
    plt.close(fig)