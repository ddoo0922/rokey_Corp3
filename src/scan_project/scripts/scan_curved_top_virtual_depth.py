from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
import json
import numpy as np
import omni
from pxr import Usd, UsdGeom

from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid


BASE_DIR = "/home/rokey/cobot4_ws/rokey_Corp3/src/scan_project"
OUTPUT_DIR = os.path.join(BASE_DIR, "output")

DEPTH_DIR = os.path.join(OUTPUT_DIR, "depth")
POSE_DIR = os.path.join(OUTPUT_DIR, "pose")
MESH_DIR = os.path.join(OUTPUT_DIR, "mesh")

os.makedirs(DEPTH_DIR, exist_ok=True)
os.makedirs(POSE_DIR, exist_ok=True)
os.makedirs(MESH_DIR, exist_ok=True)


OBJECT_PATH = "/World/ScanObject"


def save_json(path, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def save_pose_json(path, position, orientation_wxyz):
    data = {
        "position_m": position.tolist(),
        "orientation_wxyz": orientation_wxyz.tolist(),
    }
    save_json(path, data)


def get_bbox_info(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)

    if not prim.IsValid():
        raise RuntimeError(f"Invalid prim path: {prim_path}")

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        includedPurposes=[UsdGeom.Tokens.default_]
    )

    bbox = bbox_cache.ComputeWorldBound(prim)
    box = bbox.ComputeAlignedBox()

    min_pt = box.GetMin()
    max_pt = box.GetMax()

    min_arr = np.array([float(min_pt[0]), float(min_pt[1]), float(min_pt[2])])
    max_arr = np.array([float(max_pt[0]), float(max_pt[1]), float(max_pt[2])])
    center = (min_arr + max_arr) / 2.0
    size = max_arr - min_arr

    return {
        "prim_path": prim_path,
        "bbox_min_m": min_arr.tolist(),
        "bbox_max_m": max_arr.tolist(),
        "bbox_center_m": center.tolist(),
        "bbox_size_m": size.tolist(),
    }, min_arr, max_arr


def create_curved_top_surface(min_pt, max_pt, resolution=80):
    """
    굴곡 있는 윗면 형상 생성.
    현재는 테스트용 가상 표면이다.

    z = 기본 높이 + 곡면 굴곡 + 작은 물결
    """

    x_min, y_min, z_min = min_pt
    x_max, y_max, z_max = max_pt

    xs = np.linspace(x_min, x_max, resolution)
    ys = np.linspace(y_min, y_max, resolution)

    x_center = (x_min + x_max) / 2.0
    y_center = (y_min + y_max) / 2.0

    x_range = max(x_max - x_min, 1e-6)
    y_range = max(y_max - y_min, 1e-6)

    points = []

    for x in xs:
        for y in ys:
            nx = (x - x_center) / x_range
            ny = (y - y_center) / y_range

            # 굴곡이 확실히 보이도록 매직마우스 형태 곡면의 높이를 키움 (최대 +20cm)
            # nx, ny는 -1 ~ 1 범위입니다.
            dome = 0.20 * (1.0 - nx**2) * (1.0 - ny**2)
            z = z_max + dome

            points.append([x, y, z])

    return np.asarray(points, dtype=np.float32), xs, ys


def save_surface_points_ply(path, points):
    with open(path, "w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {points.shape[0]}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")

        for p in points:
            f.write(f"{p[0]} {p[1]} {p[2]}\n")


def save_top_surface_mesh_ply(path, points, resolution):
    vertices = points.tolist()
    faces = []

    for ix in range(resolution - 1):
        for iy in range(resolution - 1):
            a = ix * resolution + iy
            b = (ix + 1) * resolution + iy
            c = (ix + 1) * resolution + (iy + 1)
            d = ix * resolution + (iy + 1)

            faces.append([a, b, c])
            faces.append([a, c, d])

    with open(path, "w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(vertices)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write(f"element face {len(faces)}\n")
        f.write("property list uchar int vertex_indices\n")
        f.write("end_header\n")

        for v in vertices:
            f.write(f"{v[0]} {v[1]} {v[2]}\n")

        for face in faces:
            f.write(f"3 {face[0]} {face[1]} {face[2]}\n")


def analyze_top_surface(points):
    min_pt = np.min(points, axis=0)
    max_pt = np.max(points, axis=0)
    center = np.mean(points, axis=0)

    z_values = points[:, 2]

    return {
        "description": "Curved upper surface geometry information for sanding and polishing path planning",
        "num_surface_points": int(points.shape[0]),
        "surface_center_m": center.tolist(),
        "surface_min_m": min_pt.tolist(),
        "surface_max_m": max_pt.tolist(),
        "surface_size_m": (max_pt - min_pt).tolist(),
        "height_min_m": float(np.min(z_values)),
        "height_max_m": float(np.max(z_values)),
        "height_mean_m": float(np.mean(z_values)),
        "height_range_m": float(np.max(z_values) - np.min(z_values)),
        "height_range_cm": float((np.max(z_values) - np.min(z_values)) * 100.0),
    }


def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    scan_obj = world.scene.add(
        FixedCuboid(
            prim_path=OBJECT_PATH,
            name="scan_object",
            position=np.array([0.0, 0.0, 0.025]),
            size=1.0,
            scale=np.array([1.0, 1.0, 0.05]),
            color=np.array([0.8, 0.2, 0.2]),
        )
    )

    world.reset()

    for _ in range(60):
        simulation_app.update()

    # 실제 pose 저장
    pos, quat = scan_obj.get_world_pose()
    pose_path = os.path.join(POSE_DIR, "object_pose.json")
    save_pose_json(pose_path, pos, quat)

    # bbox 정보 저장
    bbox_info, min_pt, max_pt = get_bbox_info(OBJECT_PATH)
    bbox_path = os.path.join(MESH_DIR, "object_mesh_info.json")
    save_json(bbox_path, bbox_info)

    # 굴곡 있는 윗면 생성
    resolution = 80
    surface_points, xs, ys = create_curved_top_surface(
        min_pt=min_pt,
        max_pt=max_pt,
        resolution=resolution
    )

    # height map 저장
    height_map = surface_points[:, 2].reshape(resolution, resolution)
    height_map_path = os.path.join(DEPTH_DIR, "curved_top_height_map.npy")
    np.save(height_map_path, height_map)

    # 윗면 점 데이터 저장
    surface_points_path = os.path.join(MESH_DIR, "curved_top_surface_points.ply")
    save_surface_points_ply(surface_points_path, surface_points)

    # 윗면 mesh 저장
    surface_mesh_path = os.path.join(MESH_DIR, "curved_top_surface_mesh.ply")
    save_top_surface_mesh_ply(surface_mesh_path, surface_points, resolution)

    # 윗면 형상 정보 저장
    surface_info = analyze_top_surface(surface_points)
    surface_info_path = os.path.join(MESH_DIR, "curved_top_surface_info.json")
    save_json(surface_info_path, surface_info)

    print("[OK] Curved Top Surface Scan completed")
    print(f"[OK] saved object pose:          {pose_path}")
    print(f"[OK] saved object bbox info:     {bbox_path}")
    print(f"[OK] saved curved height map:    {height_map_path}")
    print(f"[OK] saved curved surface pts:   {surface_points_path}")
    print(f"[OK] saved curved surface mesh:  {surface_mesh_path}")
    print(f"[OK] saved curved surface info:  {surface_info_path}")


if __name__ == "__main__":
    try:
        main()
        print("\n[INFO] Scan finished and auto-exiting.")
    finally:
        simulation_app.close()
