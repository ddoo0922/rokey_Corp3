import os
import argparse
import numpy as np
from isaacsim import SimulationApp

# GUI 화면을 띄우기 위해 headless=False 설정
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.objects import VisualSphere
from omni.isaac.core.materials import OmniGlass

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
    
    # 2. 원본 자동차 모델 로드
    USD_PATH = os.path.join(BASE_DIR, "scan_obj", f"{args.obj_name}.usd")
    print(f"[DEBUG] Loading USD Object: {USD_PATH}")
    if os.path.exists(USD_PATH):
        create_prim(
            prim_path=f"/World/{args.obj_name.capitalize()}",
            prim_type="Xform",
            usd_path=USD_PATH
        )
        
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
        # 실패 시 그냥 회색 구 생성 (투명도 없음)
        VisualSphere(
            prim_path="/World/ReachSphere",
            name="reach_sphere",
            position=robot_base_pos,
            radius=0.9,
            color=np.array([0.5, 0.5, 0.5])
        )
    
    # 3. 스캔된 포인트 클라우드 데이터 로드 (빨간색)
    PLY_PATH = os.path.join(BASE_DIR, "scan_result", args.obj_name, "points", "real_camera_surface_points.ply")
    print(f"[DEBUG] Loading points from {PLY_PATH}")
    points = load_ply_points(PLY_PATH)
    
    if len(points) > 0:
        print(f"Total points: {len(points)}")
        sampled_points = points[::30]
        print(f"Sampled points for visualization: {len(sampled_points)}")
        
        for i, p in enumerate(sampled_points):
            VisualSphere(
                prim_path=f"/World/Points/Point_{i}",
                name=f"point_{i}",
                position=p,
                radius=0.005,
                color=np.array([1.0, 0.0, 0.0]) # Red
            )
            
    # 4. 3D 웨이포인트 경로 로드
    PATH_NPY = os.path.join(BASE_DIR, "scan_result", args.obj_name, "path.npy")
    path_points = []
    if os.path.exists(PATH_NPY):
        path_points = np.load(PATH_NPY)
        print(f"[DEBUG] Loaded {len(path_points)} waypoints from {PATH_NPY}")
    else:
        print(f"[ERROR] Cannot find {PATH_NPY}")
        
    world.reset()
    
    # 5. 경로를 따라 움직일 초록색 구 생성
    green_sphere = None
    if len(path_points) > 0:
        green_sphere = VisualSphere(
            prim_path="/World/PathPointer",
            name="path_pointer",
            position=path_points[0],
            radius=0.015, # 빨간 점들보다 약간 크게
            color=np.array([0.0, 1.0, 0.0]) # Green
        )
    
    print("==================================================")
    print("Isaac Sim 화면이 띄워졌습니다!")
    print("마우스 우클릭을 누른 채로 W,A,S,D를 눌러 카메라를 움직여서 확인해 보세요.")
    print("창을 닫거나 터미널에서 Ctrl+C를 누르면 종료됩니다.")
    print("==================================================")
    
    # 시뮬레이터가 강제 종료될 때까지 화면 유지하며 초록색 공 애니메이션
    idx = 0
    while simulation_app.is_running():
        world.step(render=True)
        if green_sphere and len(path_points) > 0:
            # 매 프레임마다 초록색 공의 위치를 다음 웨이포인트로 이동
            green_sphere.set_world_pose(position=path_points[idx])
            idx += 1
            if idx >= len(path_points):
                idx = 0 # 끝까지 가면 다시 처음부터
                
    simulation_app.close()

if __name__ == "__main__":
    main()
