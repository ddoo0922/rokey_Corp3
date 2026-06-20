import os
import numpy as np
import matplotlib.pyplot as plt


BASE_DIR = "/home/rokey/scan_project"
DEPTH_DIR = os.path.join(BASE_DIR, "output", "depth")
MESH_DIR = os.path.join(BASE_DIR, "output", "mesh")
IMAGE_DIR = os.path.join(BASE_DIR, "output", "images")

os.makedirs(IMAGE_DIR, exist_ok=True)


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
                    x, y, z = map(float, parts[:3])
                    points.append([x, y, z])
                except ValueError:
                    continue

    return np.asarray(points, dtype=np.float32)


def save_curved_height_map_image():
    height_map_path = os.path.join(DEPTH_DIR, "curved_top_height_map.npy")
    height_map = np.load(height_map_path)

    plt.figure(figsize=(7, 5))
    plt.imshow(height_map, origin="lower")
    plt.colorbar(label="Height (m)")
    plt.title("Curved Top Surface Height Map")
    plt.xlabel("X index")
    plt.ylabel("Y index")
    plt.tight_layout()

    save_path = os.path.join(IMAGE_DIR, "curved_top_height_map.png")
    plt.savefig(save_path, dpi=200)
    plt.close()

    print(f"[OK] saved: {save_path}")


def save_curved_3d_surface_image():
    ply_path = os.path.join(MESH_DIR, "curved_top_surface_points.ply")
    points = load_ply_points(ply_path)

    if points.shape[0] == 0:
        raise RuntimeError("PLY point data is empty")

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")

    ax.scatter(x, y, z, s=2)

    ax.set_title("Curved Top Surface 3D Shape")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z Height (m)")

    plt.tight_layout()

    save_path = os.path.join(IMAGE_DIR, "curved_top_surface_3d_shape.png")
    plt.savefig(save_path, dpi=200)
    plt.close()

    print(f"[OK] saved: {save_path}")


def main():
    save_curved_height_map_image()
    save_curved_3d_surface_image()


if __name__ == "__main__":
    main()
