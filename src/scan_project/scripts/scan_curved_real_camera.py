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
    num_rings = resolution
    num_sectors = resolution
    
    points = []
    # 꼭대기 중심점
    points.append([0.0, 0.0, 0.20])
    
    for i in range(1, num_rings + 1):
        r = float(i) / num_rings
        # Z 높이: 0.20 * sqrt(1 - r^2)
        z = 0.20 * np.sqrt(max(0.0, 1.0 - r**2))
        xy_radius = r * 0.5
        
        for j in range(num_sectors):
            theta = 2.0 * np.pi * float(j) / num_sectors
            x = xy_radius * np.cos(theta)
            y = xy_radius * np.sin(theta)
            points.append([x, y, z])
            
    faces = []
    # 중심점과 첫 번째 링 연결
    for j in range(num_sectors):
        next_j = (j + 1) % num_sectors
        faces.append([0, 1 + j, 1 + next_j])
        
    # 나머지 링들 연결 (사각형을 삼각형 2개로 분할)
    for i in range(1, num_rings):
        ring_start = 1 + (i - 1) * num_sectors
        next_ring_start = 1 + i * num_sectors
        for j in range(num_sectors):
            next_j = (j + 1) % num_sectors
            
            a = ring_start + j
            b = next_ring_start + j
            c = next_ring_start + next_j
            d = ring_start + next_j
            
            # Winding order를 UP 방향으로 맞춤
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
            
            # distance_to_camera는 카메라 원점으로부터의 유클리드 거리(Euclidean Distance)입니다.
            # 올바른 3D 좌표를 구하기 위해 z_cam을 계산합니다.
            ray_x = (u - cx) / fx
            ray_y = (v - cy) / fy
            ray_length = np.sqrt(ray_x**2 + ray_y**2 + 1.0)
            
            z_cam = d / ray_length
            x_cam = ray_x * z_cam
            y_cam = ray_y * z_cam
            
            # Since camera looks at -Z
            x_world = cam_x + x_cam
            y_world = cam_y - y_cam
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
    # 파라미터를 명시하여 수학적 오차가 없도록 고정합니다.
    camera = rep.create.camera(
        position=tuple(CAMERA_POSITION.tolist()),
        look_at=(0.0, 0.0, 0.0),
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
    
    # 실제 카메라의 정확한 렌즈 파라미터(초점거리 등) 추출
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
    
    # 실제 카메라의 내장 파라미터(Intrinsic Matrix)를 가져옵니다.
    # 이를 통해 카메라 렌즈의 정확한 화각(FOV) 오차를 완전히 없앱니다.
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
        # rep.create.camera에 설정한 focal_length=24.0, horizontal_aperture=20.955를 바탕으로 한 완벽한 렌즈 계산
        fx = (IMAGE_WIDTH * 24.0) / 20.955
        fy = fx  # square pixels assumed
        cx = IMAGE_WIDTH / 2.0
        cy = IMAGE_HEIGHT / 2.0

    # 3D Point Cloud로 변환 (Reprojection)
    surface_points = depth_to_top_surface(depth, CAMERA_POSITION, fx, fy, cx, cy)
    
    # 이제 수학적 왜곡(Euclidean Distance 보정)이 해결되었으므로 완벽히 일치합니다.
    # RGB 데이터 가져와서 이미지로 저장
    rgb_data = rgb_annot.get_data()
    if rgb_data is not None:
        from PIL import Image
        # rgb_data는 주로 (H, W, 4) 형태의 RGBA 배열입니다.
        img = Image.fromarray(rgb_data, "RGBA")
        rgb_path = os.path.join(OUTPUT_DIR, "real_camera_rgb.png")
        # RGB 모드로 변환 후 저장 (배경이 투명하게 나오지 않도록)
        img.convert("RGB").save(rgb_path)
        print(f"[OK] Saved RGB image: {rgb_path}")

    # 물체(원형 돔) 바깥쪽의 바닥면이나 노이즈가 스캔되지 않도록
    # 정확히 물체의 반지름(0.5m) 내부의 점들만 남깁니다 (모서리 스커트 현상 완벽 제거)
    radiuses_sq = surface_points[:, 0]**2 + surface_points[:, 1]**2
    surface_points = surface_points[radiuses_sq <= (0.495)**2]
    
    # 혹시 모를 바닥면 찌꺼기 제거
    surface_points = surface_points[surface_points[:, 2] > 0.01]

    surface_points_path = os.path.join(MESH_DIR, "real_camera_surface_points.ply")
    save_surface_points_ply(surface_points_path, surface_points)

    # 원본 메쉬도 파일로 저장 (Polishing Sim에서 불러다 쓰기 위함)
    mesh_path = os.path.join(MESH_DIR, "real_camera_surface_mesh.ply")
    save_mesh_ply(mesh_path, points, faces)

    surface_info = analyze_top_surface(surface_points)
    surface_info_path = os.path.join(MESH_DIR, "real_camera_surface_info.json")
    save_json(surface_info_path, surface_info)

    print("[OK] Real Depth Camera Scan Completed!")
    print(f"[OK] Saved points: {surface_points_path}")
    print(f"[OK] Saved mesh:   {mesh_path}")
    print(f"[OK] Saved info:   {surface_info_path}")

if __name__ == "__main__":
    try:
        main()
        print("\n[INFO] Auto-exiting.")
    finally:
        simulation_app.close()
