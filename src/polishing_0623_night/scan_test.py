import os
import argparse
import numpy as np
import omni
from isaacsim import SimulationApp

# 화면을 보면서 확인해야 하므로 headless=False
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.objects import VisualSphere
from omni.isaac.core.materials import OmniGlass
import omni.replicator.core as rep
from pxr import UsdGeom, Vt

def load_ply_points(ply_path):
    points = []
    header_done = False
    if not os.path.exists(ply_path):
        return np.array([])
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--obj_name", type=str, default="car")
    args, unknown = parser.parse_known_args()

    BASE_DIR = "/home/rokey/rokey_Corp3/src/polishing_0623"
    
    # 1. 월드 초기화
    world = World(stage_units_in_meters=1.0)
    
    # 2. 스캔 대상 객체 로드
    USD_PATH = os.path.join(BASE_DIR, "scan_obj", f"{args.obj_name}.usd")
    print(f"[DEBUG] Loading USD Object: {USD_PATH}")
    if os.path.exists(USD_PATH):
        create_prim(
            prim_path=f"/World/{args.obj_name.capitalize()}",
            prim_type="Xform",
            usd_path=USD_PATH
        )
    else:
        print(f"[ERROR] Cannot find {USD_PATH}")

    # 2-1. 로봇 로드
    ROBOT_USD_PATH = os.path.join(BASE_DIR, "m0609_polishing.usd")
    robot_base_pos = np.array([-0.8, -0.7, 0.2])
    if os.path.exists(ROBOT_USD_PATH):
        create_prim(
            prim_path="/World/M0609",
            prim_type="Xform",
            position=robot_base_pos,
            usd_path=ROBOT_USD_PATH
        )
        
    # 2-2. 0.9m 가동 반경(Reach) 반투명 구 생성
    try:
        glass_material = OmniGlass(
            prim_path="/World/Looks/Glass",
            color=np.array([0.5, 0.5, 0.5]),
            ior=1.0,
            depth=0.001
        )
        VisualSphere(
            prim_path="/World/ReachSphere",
            name="reach_sphere",
            position=robot_base_pos,
            radius=0.9,
            visual_material=glass_material
        )
    except Exception as e:
        print(f"[Warning] Failed to apply glass material: {e}")
        VisualSphere(
            prim_path="/World/ReachSphere",
            name="reach_sphere",
            position=robot_base_pos,
            radius=0.9,
            color=np.array([0.5, 0.5, 0.5])
        )
        
    # 3. scan.py 와 동일한 카메라 생성
    CAMERA_POSITION = (0.0, -0.75, 2.50)
    
    camera = rep.create.camera(
        position=CAMERA_POSITION,
        look_at=(0.0, -0.75, 0.0),
        focal_length=24.0,
        horizontal_aperture=20.955
    )
    
    # 카메라 위치를 눈으로 쉽게 확인할 수 있도록 노란색 구 생성
    VisualSphere(
        prim_path="/World/CameraMarker",
        name="camera_marker",
        position=np.array(CAMERA_POSITION),
        radius=0.05,
        color=np.array([1.0, 1.0, 0.0]) # Yellow
    )

    # 4. 스캔된 포인트 클라우드 로드 및 시각화
    PLY_PATH = os.path.join(BASE_DIR, "scan_result", args.obj_name, "points", "real_camera_surface_points.ply")
    points = load_ply_points(PLY_PATH)
    if len(points) > 0:
        print(f"[DEBUG] Loaded {len(points)} points from {PLY_PATH}")
        # 시각화 최적화를 위해 1/30로 다운샘플링
        sampled_points = points[::30]
        for i, p in enumerate(sampled_points):
            VisualSphere(
                prim_path=f"/World/Points/Point_{i}",
                name=f"point_{i}",
                position=p,
                radius=0.005, # 0.5cm
                color=np.array([1.0, 0.0, 0.0]) # Red
            )
    else:
        print(f"[WARN] Cannot find or empty PLY: {PLY_PATH}")


    world.reset()
    
    print("==================================================")
    print("스캔 환경 확인용 화면이 띄워졌습니다!")
    print("노란색 공 위치가 실제 카메라 렌즈의 위치입니다.")
    print("마우스 우클릭을 누른 채로 W,A,S,D를 눌러서 카메라가 자동차의 어느 부위를 찍고 있는지 확인해 보세요.")
    print("==================================================")
    
    # 사용자가 창을 닫을 때까지 유지
    while simulation_app.is_running():
        world.step(render=True)
        
    simulation_app.close()

if __name__ == "__main__":
    main()
