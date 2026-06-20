from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
import json
import numpy as np
import omni
import omni.replicator.core as rep

from pxr import Usd, UsdGeom, Vt, Gf
from isaacsim.core.api import World

BASE_DIR = "/home/rokey/cobot4_ws/rokey_Corp3/src/scan_project"
OUTPUT_DIR = os.path.join(BASE_DIR, "output")

DEPTH_DIR = os.path.join(OUTPUT_DIR, "depth")
MESH_DIR = os.path.join(OUTPUT_DIR, "mesh")

os.makedirs(DEPTH_DIR, exist_ok=True)
os.makedirs(MESH_DIR, exist_ok=True)

CAMERA_POSITION = np.array([0.0, 0.0, 1.60], dtype=np.float32)
CAMERA_FOV_DEG = 60.0
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
RESOLUTION = 80 # Mesh generation resolution

def save_json(path, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

def create_magic_mouse_points(resolution):
    x_min, x_max = -0.5, 0.5
    y_min, y_max = -0.5, 0.5
    z_max = 0.0

    xs = np.linspace(x_min, x_max, resolution)
    ys = np.linspace(y_min, y_max, resolution)

    x_range = x_max - x_min
    y_range = y_max - y_min
    
    points = []
    for x in xs:
        for y in ys:
            nx = x / (x_range / 2.0)
            ny = y / (y_range / 2.0)
            dome = 0.20 * (1.0 - nx**2) * (1.0 - ny**2)
            z = z_max + dome
            points.append([x, y, z])
    
    faces = []
    for ix in range(resolution - 1):
        for iy in range(resolution - 1):
            a = ix * resolution + iy
            b = (ix + 1) * resolution + iy
            c = (ix + 1) * resolution + (iy + 1)
            d = ix * resolution + (iy + 1)
            faces.append([a, b, c])
            faces.append([a, c, d])
            
    return np.array(points, dtype=np.float32), faces

def create_usd_mesh(stage, prim_path, points, faces):
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    
    # Python float로 변환하여 C++ 바인딩 충돌 방지
    pts_list = [Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in points]
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(pts_list))
    
    face_vertex_counts = [int(len(f)) for f in faces]
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(face_vertex_counts))
    
    face_vertex_indices = [int(idx) for f in faces for idx in f]
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(face_vertex_indices))
    
    min_pt = np.min(points, axis=0)
    max_pt = np.max(points, axis=0)
    extents = [Gf.Vec3f(float(min_pt[0]), float(min_pt[1]), float(min_pt[2])),
               Gf.Vec3f(float(max_pt[0]), float(max_pt[1]), float(max_pt[2]))]
    mesh.GetExtentAttr().Set(Vt.Vec3fArray(extents))
    
    mesh.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.8, 0.2, 0.2)]))
    return mesh

def depth_to_top_surface(depth: np.ndarray, camera_position: np.ndarray, fov_deg: float):
    h, w = depth.shape
    fov_rad = np.deg2rad(fov_deg)
    # Replicator uses focal length based on horizontal FOV usually
    fx = (w / 2.0) / np.tan(fov_rad / 2.0)
    fy = fx
    cx = w / 2.0
    cy = h / 2.0
    cam_x, cam_y, cam_z = camera_position
    
    points = []
    step = 2
    for v in range(0, h, step):
        for u in range(0, w, step):
            d = float(depth[v, u])
            if not np.isfinite(d) or d <= 0.0 or d > 5.0:
                continue
            
            # Replicator camera looks down -Z. X is right, Y is up (in local camera frame).
            # Wait, if we use look_at=(0,0,0), camera might have a different orientation.
            # Assuming camera is at (0,0,1.6) looking straight down at (0,0,0).
            # Then local Z points UP (towards +Z world) and local -Z looks down.
            # Local X points to +X world. Local Y points to +Y world.
            x_cam = (u - cx) * d / fx
            y_cam = (v - cy) * d / fy
            
            # Since camera looks at -Z
            x_world = cam_x + x_cam
            y_world = cam_y - y_cam
            z_world = cam_z - d
            
            points.append([x_world, y_world, z_world])
            
    return np.asarray(points, dtype=np.float32)

def save_surface_points_ply(path: str, points: np.ndarray):
    with open(path, "w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {points.shape[0]}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for p in points:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

def analyze_top_surface(points: np.ndarray):
    min_pt = np.min(points, axis=0)
    max_pt = np.max(points, axis=0)
    center = np.mean(points, axis=0)
    z_values = points[:, 2]
    return {
        "description": "Top depth camera based real upper surface geometry",
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
    print("[DEBUG] Initializing World")
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    print("[DEBUG] Generating Magic Mouse Points")
    # 물리적인 Magic Mouse Mesh 생성
    points, faces = create_magic_mouse_points(RESOLUTION)
    
    print("[DEBUG] Getting USD Stage")
    stage = omni.usd.get_context().get_stage()
    print("[DEBUG] Stage:", stage)
    
    print("[DEBUG] Creating USD Mesh")
    create_usd_mesh(stage, "/World/MagicMouse", points, faces)

    print("[DEBUG] Resetting World")
    world.reset()

    print("[DEBUG] Creating Camera")
    # 상단에서 내려다보는 실제 Depth Camera 생성
    camera = rep.create.camera(
        position=tuple(CAMERA_POSITION.tolist()),
        look_at=(0.0, 0.0, 0.0)
    )

    render_product = rep.create.render_product(
        camera,
        resolution=(IMAGE_WIDTH, IMAGE_HEIGHT)
    )

    depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
    depth_annot.attach([render_product])

    print("[DEBUG] Stepping Simulation")
    # 시뮬레이션 안정화 및 렌더링
    for i in range(60):
        simulation_app.update()
        rep.orchestrator.step()

    print("[DEBUG] Getting Depth Data")
    # 실제 카메라에서 Depth 데이터 가져오기
    depth = depth_annot.get_data()
    if depth is None:
        raise RuntimeError("Depth 데이터를 획득하지 못했습니다.")
    
    depth = np.asarray(depth, dtype=np.float32)
    print(f"[DEBUG] Depth Map Shape: {depth.shape}")
    
    # 3D Point Cloud로 변환 (Reprojection)
    surface_points = depth_to_top_surface(depth, CAMERA_POSITION, CAMERA_FOV_DEG)
    
    # 바닥면(Z < 0.01) 필터링
    surface_points = surface_points[surface_points[:, 2] > 0.01]

    # 저장
    surface_points_path = os.path.join(MESH_DIR, "real_camera_surface_points.ply")
    save_surface_points_ply(surface_points_path, surface_points)

    surface_info = analyze_top_surface(surface_points)
    surface_info_path = os.path.join(MESH_DIR, "real_camera_surface_info.json")
    save_json(surface_info_path, surface_info)

    print("[OK] Real Depth Camera Scan Completed!")
    print(f"[OK] Saved points: {surface_points_path}")
    print(f"[OK] Saved info:   {surface_info_path}")

if __name__ == "__main__":
    try:
        main()
        print("\n[INFO] Auto-exiting.")
    finally:
        simulation_app.close()
