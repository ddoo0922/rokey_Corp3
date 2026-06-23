from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
import json
import numpy as np
import omni
import omni.replicator.core as rep

from pxr import Usd, UsdGeom, Vt, Gf
from isaacsim.core.api import World
from omni.isaac.core.utils.prims import create_prim

import argparse

# 경로 설정
BASE_DIR = "/home/rokey/rokey_Corp3/src/polishing_0623"

parser = argparse.ArgumentParser(description="Scan a USD object")
parser.add_argument("--obj_name", type=str, default="car", help="Name of the object to scan (e.g. car, cube)")
args, unknown = parser.parse_known_args()

USD_PATH = os.path.join(BASE_DIR, "scan_obj", f"{args.obj_name}.usd")
OUTPUT_DIR = os.path.join(BASE_DIR, "scan_result", args.obj_name)

DEPTH_DIR = os.path.join(OUTPUT_DIR, "depth")
POINTS_DIR = os.path.join(OUTPUT_DIR, "points")

os.makedirs(DEPTH_DIR, exist_ok=True)
os.makedirs(POINTS_DIR, exist_ok=True)

CAMERA_POSITION = np.array([0.0, -0.75, 2.50], dtype=np.float32)
CAMERA_FOV_DEG = 60.0
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

def save_json(path, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

def depth_to_top_surface(depth: np.ndarray, camera_position: np.ndarray, fx: float, fy: float, cx: float, cy: float):
    h, w = depth.shape
    cam_x, cam_y, cam_z = camera_position
    
    points = []
    step = 2
    for v in range(0, h, step):
        for u in range(0, w, step):
            d = float(depth[v, u])
            if not np.isfinite(d) or d <= 0.0 or d > 5.0:
                continue
            
            ray_x = (u - cx) / fx
            ray_y = (v - cy) / fy
            ray_length = np.sqrt(ray_x**2 + ray_y**2 + 1.0)
            
            z_cam = d / ray_length
            x_cam = ray_x * z_cam
            y_cam = ray_y * z_cam
            
            x_world = cam_x + y_cam
            y_world = cam_y + x_cam
            z_world = cam_z - z_cam
            
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

def save_mesh_ply(path: str, points: np.ndarray, faces: list):
    with open(path, "w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write(f"element face {len(faces)}\n")
        f.write("property list uchar int vertex_indices\n")
        f.write("end_header\n")
        for p in points:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
        for face in faces:
            f.write(f"{len(face)} " + " ".join(str(int(idx)) for idx in face) + "\n")



def analyze_top_surface(points: np.ndarray):
    if len(points) == 0:
        return {
            "description": "Empty surface points",
            "num_surface_points": 0
        }
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
    # 바닥면 삭제 (충돌 방지 및 원본 상태 유지)
    
    print("[DEBUG] Loading USD Object:", USD_PATH)
    # USD 객체를 씬에 배치
    create_prim(
        prim_path=f"/World/{args.obj_name.capitalize()}",
        prim_type="Xform",
        usd_path=USD_PATH
    )
    
    print("[DEBUG] Getting USD Stage")
    stage = omni.usd.get_context().get_stage()
    
    print("[DEBUG] Resetting World")
    world.reset()

    print("[DEBUG] Creating Camera")
    # 상단에서 내려다보는 실제 Depth Camera 생성
    camera = rep.create.camera(
        position=tuple(CAMERA_POSITION.tolist()),
        look_at=(0.0, -0.75, 0.0),
        focal_length=24.0,
        horizontal_aperture=20.955
    )

    render_product = rep.create.render_product(
        camera,
        resolution=(IMAGE_WIDTH, IMAGE_HEIGHT)
    )

    depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
    depth_annot.attach([render_product])
    
    # RGB 이미지도 같이 추출
    rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annot.attach([render_product])
    
    # 실제 카메라의 정확한 렌즈 파라미터 추출
    camera_params_annot = rep.AnnotatorRegistry.get_annotator("camera_params")
    camera_params_annot.attach([render_product])

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
    
    # 카메라 내장 파라미터(Intrinsic Matrix) 획득
    cam_params = camera_params_annot.get_data()
    if cam_params and "cameraMatrix" in cam_params:
        K = cam_params["cameraMatrix"]
        fx = float(K[0][0])
        fy = float(K[1][1])
        cx = float(K[0][2])
        cy = float(K[1][2])
        print(f"[DEBUG] Exact Camera Intrinsics: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")
    else:
        print("[DEBUG] Failed to get cameraMatrix, using exact calculation fallback")
        fx = (IMAGE_WIDTH * 24.0) / 20.955
        fy = fx
        cx = IMAGE_WIDTH / 2.0
        cy = IMAGE_HEIGHT / 2.0

    # 3D Point Cloud로 변환 (Reprojection)
    surface_points = depth_to_top_surface(depth, CAMERA_POSITION, fx, fy, cx, cy)
    
    # RGB 데이터 가져와서 이미지로 저장
    rgb_data = rgb_annot.get_data()
    if rgb_data is not None:
        from PIL import Image
        img = Image.fromarray(rgb_data, "RGBA")
        rgb_path = os.path.join(OUTPUT_DIR, "real_camera_rgb.png")
        img.convert("RGB").save(rgb_path)
        print(f"[OK] Saved RGB image: {rgb_path}")

    # 필터링: 바닥(Z <= 0.01) 및 유효 범위를 벗어나는 데이터 필터링
    mask = (
        (surface_points[:, 0] >= -0.8) & (surface_points[:, 0] <= 0.8) &
        (surface_points[:, 1] >= -0.8) & (surface_points[:, 1] <= 0.8) &
        (surface_points[:, 2] > 0.01)
    )
    surface_points = surface_points[mask]

    # 포인트 클라우드 저장
    surface_points_path = os.path.join(POINTS_DIR, "real_camera_surface_points.ply")
    save_surface_points_ply(surface_points_path, surface_points)

    # 분석 데이터 저장
    surface_info = analyze_top_surface(surface_points)
    surface_info_path = os.path.join(POINTS_DIR, "real_camera_surface_info.json")
    save_json(surface_info_path, surface_info)

    print("[OK] Real Depth Camera Scan Completed!")
    print(f"[OK] Saved points: {surface_points_path}")
    print(f"[OK] Saved info:   {surface_info_path}")

if __name__ == "__main__":
    try:
        main()
        print("\n[INFO] Auto-exiting.")
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"[ERROR] {e}")
    finally:
        simulation_app.close()
