from isaacsim import SimulationApp

# 처음 확인할 때는 False, 나중에 자동 실행은 True
simulation_app = SimulationApp({"headless": False})

import os
import json
import numpy as np
import omni
import omni.replicator.core as rep

from pxr import Usd, UsdGeom
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid


BASE_DIR = "/home/rokey/scan_project"

OUTPUT_DIR = os.path.join(BASE_DIR, "output")
DEPTH_DIR = os.path.join(OUTPUT_DIR, "depth")
POSE_DIR = os.path.join(OUTPUT_DIR, "pose")
MESH_DIR = os.path.join(OUTPUT_DIR, "mesh")

os.makedirs(DEPTH_DIR, exist_ok=True)
os.makedirs(POSE_DIR, exist_ok=True)
os.makedirs(MESH_DIR, exist_ok=True)


CAMERA_POSITION = np.array([0.65, 0.0, 1.60], dtype=np.float32)
CAMERA_FOV_DEG = 60.0
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240


def save_json(path, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def save_pose_json(path: str, position: np.ndarray, orientation_wxyz: np.ndarray):
    data = {
        "position_m": position.tolist(),
        "orientation_wxyz": orientation_wxyz.tolist(),
    }
    save_json(path, data)


def save_mesh_info_json(prim_path: str, output_path: str):
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

    size = [
        float(max_pt[0] - min_pt[0]),
        float(max_pt[1] - min_pt[1]),
        float(max_pt[2] - min_pt[2]),
    ]

    center = [
        float((min_pt[0] + max_pt[0]) / 2.0),
        float((min_pt[1] + max_pt[1]) / 2.0),
        float((min_pt[2] + max_pt[2]) / 2.0),
    ]

    data = {
        "prim_path": prim_path,
        "bbox_min_m": [float(min_pt[0]), float(min_pt[1]), float(min_pt[2])],
        "bbox_max_m": [float(max_pt[0]), float(max_pt[1]), float(max_pt[2])],
        "bbox_size_m": size,
        "bbox_center_m": center,
    }

    save_json(output_path, data)


def depth_to_top_surface(depth: np.ndarray, camera_position: np.ndarray, fov_deg: float):
    """
    상부 Depth Camera 기준으로 윗면 표면 좌표를 계산한다.
    카메라는 위에서 아래 방향으로 본다고 가정한다.

    반환:
    - points: [x, y, z] 표면 좌표 배열
    - height_map: z 높이맵
    """

    h, w = depth.shape

    fov_rad = np.deg2rad(fov_deg)
    fx = (w / 2.0) / np.tan(fov_rad / 2.0)
    fy = fx

    cx = w / 2.0
    cy = h / 2.0

    cam_x, cam_y, cam_z = camera_position

    points = []
    height_map = np.full((h, w), np.nan, dtype=np.float32)

    # 너무 많은 점을 저장하지 않도록 2픽셀 간격으로 샘플링
    step = 2

    for v in range(0, h, step):
        for u in range(0, w, step):
            d = float(depth[v, u])

            if not np.isfinite(d):
                continue

            if d <= 0.0 or d > 5.0:
                continue

            # 카메라 좌표계에서의 x, y
            x_cam = (u - cx) * d / fx
            y_cam = (v - cy) * d / fy

            # 상부 카메라 기준 world 변환
            x_world = cam_x + x_cam
            y_world = cam_y - y_cam
            z_world = cam_z - d

            # 바닥 제거: 물체 윗면만 남기기 위한 간단한 필터
            # 지금 큐브가 z=0.2 근처이므로 5cm 이하는 바닥으로 보고 제거
            #if z_world < -0.10:
                #continue

            points.append([x_world, y_world, z_world])
            height_map[v, u] = z_world

    if len(points) == 0:
        return np.zeros((0, 3), dtype=np.float32), height_map

    return np.asarray(points, dtype=np.float32), height_map


def save_surface_points_ply(path: str, points: np.ndarray):
    """
    표면 좌표를 PLY로 저장한다.
    Mesh가 아니라 표면 점 데이터 저장용이다.
    """

    points = points[np.isfinite(points).all(axis=1)]

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


def save_top_surface_mesh_ply(path: str, points: np.ndarray):
    """
    표면 점들을 기반으로 간단한 윗면 mesh를 생성해서 PLY로 저장한다.
    정확한 산업용 mesh reconstruction이 아니라,
    윗면 형상 확인용 간단 mesh다.
    """

    if points.shape[0] < 4:
        raise RuntimeError("mesh를 만들기 위한 표면 점이 부족합니다.")

    # x, y 기준 정렬
    points = points[np.argsort(points[:, 1])]
    points = points[np.argsort(points[:, 0], kind="stable")]

    # 너무 복잡하지 않게 가까운 격자 형태로 재구성
    xs = np.unique(np.round(points[:, 0], 4))
    ys = np.unique(np.round(points[:, 1], 4))

    if len(xs) < 2 or len(ys) < 2:
        raise RuntimeError("mesh grid를 만들기 위한 x/y 데이터가 부족합니다.")

    # 각 x,y에 가장 가까운 z 저장
    z_map = {}
    for p in points:
        key = (round(float(p[0]), 4), round(float(p[1]), 4))
        z_map[key] = float(p[2])

    vertices = []
    vertex_index = {}

    for ix, x in enumerate(xs):
        for iy, y in enumerate(ys):
            key = (round(float(x), 4), round(float(y), 4))
            if key in z_map:
                vertex_index[(ix, iy)] = len(vertices)
                vertices.append([float(x), float(y), z_map[key]])

    faces = []

    for ix in range(len(xs) - 1):
        for iy in range(len(ys) - 1):
            a = vertex_index.get((ix, iy))
            b = vertex_index.get((ix + 1, iy))
            c = vertex_index.get((ix + 1, iy + 1))
            d = vertex_index.get((ix, iy + 1))

            if a is not None and b is not None and c is not None:
                faces.append([a, b, c])

            if a is not None and c is not None and d is not None:
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


def analyze_top_surface(points: np.ndarray):
    """
    윗면 형상 정보 계산.
    """

    if points.shape[0] == 0:
        raise RuntimeError("표면 점 데이터가 비어 있습니다.")

    min_pt = np.min(points, axis=0)
    max_pt = np.max(points, axis=0)
    center = np.mean(points, axis=0)

    z_values = points[:, 2]

    data = {
        "description": "Top depth camera based upper surface geometry information",
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

    return data


def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # 1) 스캔 대상 오브젝트 생성
    scan_obj = world.scene.add(
        FixedCuboid(
            prim_path="/World/ScanObject",
            name="scan_object",
            position=np.array([0.65, 0.0, 0.20]),
            scale=np.array([0.12, 0.18, 0.08]),
            color=np.array([0.8, 0.2, 0.2]),
        )
    )

    world.reset()

    # 2) 상부 Depth Camera 생성
    # USD 카메라는 기본적으로 -Z 방향을 바라보므로,
    # z=1.60 위치에 두면 아래쪽 물체를 본다.
    camera = rep.create.camera(
        position=tuple(CAMERA_POSITION.tolist()),
        rotation=(0.65, 0.0, 0.20)
    )

    render_product = rep.create.render_product(
        camera,
        resolution=(IMAGE_WIDTH, IMAGE_HEIGHT)
    )

    depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
    depth_annot.attach([render_product])

    # 3) 시뮬레이션 업데이트
    for _ in range(60):
        simulation_app.update()
        rep.orchestrator.step()

    # 4) Depth 데이터 획득
    depth = depth_annot.get_data()

    if depth is None:
        raise RuntimeError("Depth 데이터를 획득하지 못했습니다.")

    depth = np.asarray(depth, dtype=np.float32)
    
    print("[DEBUG] depth shape:", depth.shape)
    print("[DEBUG] depth min:", np.nanmin(depth))
    print("[DEBUG] depth max:", np.nanmax(depth))
    print("[DEBUG] depth mean:", np.nanmean(depth))
    print("[DEBUG] finite count:", np.isfinite(depth).sum())

    depth_path = os.path.join(DEPTH_DIR, "top_depth.npy")
    np.save(depth_path, depth)

    # 5) 윗면 표면 정보 계산
    surface_points, height_map = depth_to_top_surface(
        depth=depth,
        camera_position=CAMERA_POSITION,
        fov_deg=CAMERA_FOV_DEG
    )

    if surface_points.shape[0] == 0:
        raise RuntimeError("상부 Depth 기반 표면 데이터가 비어 있습니다.")

    height_map_path = os.path.join(DEPTH_DIR, "top_height_map.npy")
    np.save(height_map_path, height_map)

    # 6) 표면 점 데이터 저장
    surface_points_path = os.path.join(MESH_DIR, "top_surface_points.ply")
    save_surface_points_ply(surface_points_path, surface_points)

    # 7) 윗면 mesh 저장
    surface_mesh_path = os.path.join(MESH_DIR, "top_surface_mesh.ply")
    save_top_surface_mesh_ply(surface_mesh_path, surface_points)

    # 8) 윗면 형상 정보 JSON 저장
    top_surface_info = analyze_top_surface(surface_points)
    top_surface_info_path = os.path.join(MESH_DIR, "top_surface_info.json")
    save_json(top_surface_info_path, top_surface_info)

    # 9) 실제 오브젝트 위치 저장
    pos, quat = scan_obj.get_world_pose()

    pose_path = os.path.join(POSE_DIR, "object_pose.json")
    save_pose_json(pose_path, pos, quat)

    # 10) 오브젝트 bbox 정보 저장
    mesh_info_path = os.path.join(MESH_DIR, "object_mesh_info.json")
    save_mesh_info_json("/World/ScanObject", mesh_info_path)

    print("[OK] Top Depth Camera scan completed")
    print(f"[OK] saved depth:             {depth_path}")
    print(f"[OK] saved height map:        {height_map_path}")
    print(f"[OK] saved surface points:    {surface_points_path}")
    print(f"[OK] saved top surface mesh:  {surface_mesh_path}")
    print(f"[OK] saved top surface info:  {top_surface_info_path}")
    print(f"[OK] saved object pose:       {pose_path}")
    print(f"[OK] saved object bbox info:  {mesh_info_path}")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
