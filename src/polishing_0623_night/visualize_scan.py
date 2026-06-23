import os
import numpy as np
import matplotlib.pyplot as plt

def load_ply_points(ply_path):
    points = []
    header_done = False
    with open(ply_path, "r") as f:
        for line in f:
            line = line.strip()
            if not header_done:
                if line == "end_header":
                    header_done = True
                continue
            parts = line.split()
            if len(parts) >= 3:
                try:
                    points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                except ValueError:
                    continue
    return np.asarray(points, dtype=np.float32)

def main():
    ply_path = "/home/rokey/rokey_Corp3/src/polishing_0623/scan_result/points/real_camera_surface_points.ply"
    points = load_ply_points(ply_path)

    if len(points) > 10000:
        points = points[::len(points)//10000]

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    sc = ax.scatter(x, y, z, c=z, cmap='viridis', s=5)
    plt.colorbar(sc, label="Z Height (m)")

    ax.set_title("Scanned 3D Point Cloud (Cube)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z Height (m)")
    
    try:
        ax.set_box_aspect((np.ptp(x), np.ptp(y), np.ptp(z)))
    except:
        pass

    plt.tight_layout()
    save_path = "/tmp/scan_3d_plot.png"
    plt.savefig(save_path, dpi=200)
    print(f"Saved to {save_path}")

if __name__ == "__main__":
    main()
