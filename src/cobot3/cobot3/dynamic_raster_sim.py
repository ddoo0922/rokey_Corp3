import json
import os
import numpy as np
from omni.isaac.kit import SimulationApp

# Isaac Sim 앱 시작
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid, VisualCylinder

# 1. 실제 Depth Camera 스캔 데이터(JSON) 로드
scan_json_path = "/home/rokey/cobot4_ws/rokey_Corp3/src/scan_project/output/mesh/real_camera_surface_info.json"
if not os.path.exists(scan_json_path):
    # 만약 curved_top_surface_info.json이 없다면 top_surface_info.json 시도
    scan_json_path = "/home/rokey/cobot4_ws/rokey_Corp3/src/scan_project/output/mesh/top_surface_info.json"

with open(scan_json_path, 'r') as f:
    scan_data = json.load(f)

print(f"Loaded Scan Data from: {scan_json_path}")

# 바운딩 박스 및 높이 정보 추출
x_min, y_min, z_min = scan_data["surface_min_m"]
x_max, y_max, z_max = scan_data["surface_max_m"]
max_height = scan_data["height_max_m"]
center_m = scan_data["surface_center_m"]

print(f"Scan Bounding Box: X({x_min:.3f} ~ {x_max:.3f}), Y({y_min:.3f} ~ {y_max:.3f})")
print(f"Max Height: {max_height:.3f}m")

# 스캔을 위해 조금 더 여유있는 높이 설정 (가장 높은 곳에서 1cm 위)
z_scan = max_height + 0.01

# 월드 생성
world = World(stage_units_in_meters=1.0)

# 조명(DomeLight) 추가
from omni.isaac.core.utils.prims import create_prim
create_prim(
    prim_path="/World/DomeLight",
    prim_type="DomeLight",
    attributes={"inputs:intensity": 1000.0, "inputs:color": (1.0, 1.0, 1.0)}
)

# 시각적 참고를 위해 실제 스캔된 영역 크기만큼의 바닥/물체를 하나 생성
size_x = x_max - x_min
size_y = y_max - y_min
size_z = max_height

scanned_volume = world.scene.add(
    VisualCuboid(
        prim_path="/World/ScannedArea",
        name="scanned_area",
        position=np.array([center_m[0], center_m[1], max_height / 2.0]),
        scale=np.array([size_x if size_x > 0 else 0.1, size_y if size_y > 0 else 0.1, max_height]),
        color=np.array([0.5, 0.5, 0.5])
    )
)

# 150mm(0.15m) 직경 원형 툴 (로봇/센서 모사)
radius = 0.075
obj = world.scene.add(
    VisualCylinder(
        prim_path="/World/Sensor",
        name="sensor",
        position=np.array([x_min, y_min, z_scan]),
        radius=radius,
        height=0.02,
        color=np.array([1.0, 0.2, 0.2])
    )
)

# 2. 동적 Raster 경로 생성 로직
# 센서 중심이 물체의 끝단까지 가야 전체가 커버되므로, x_min에서 x_max까지 이동합니다.
waypoints = []
x_start = x_min
x_end = x_max
y_start = y_min
y_end = y_max

step = 0.12 # 120mm 간격
y = y_start
direction = 1

# y_max가 y_min과 같거나 작을 정도로 아주 좁은 영역일 경우 1회만 스캔
if y_max - y_min < 0.01:
    waypoints.append(np.array([x_start, y_start, z_scan]))
    waypoints.append(np.array([x_end, y_start, z_scan]))
else:
    while y <= y_end:
        if direction == 1:
            waypoints.append(np.array([x_start, y, z_scan]))
            waypoints.append(np.array([x_end, y, z_scan]))
        else:
            waypoints.append(np.array([x_end, y, z_scan]))
            waypoints.append(np.array([x_start, y, z_scan]))
        
        y += step
        direction *= -1

    # 마지막 줄 처리 (빈틈없이 덮도록)
    if y - step < y_end - 0.001: # 부동소수점 오차 방지
        y = y_end
        if direction == 1:
            waypoints.append(np.array([x_start, y, z_scan]))
            waypoints.append(np.array([x_end, y, z_scan]))
        else:
            waypoints.append(np.array([x_end, y, z_scan]))
            waypoints.append(np.array([x_start, y, z_scan]))

print(f"Generated {len(waypoints)} dynamic waypoints.")

world.reset()

current_wp_idx = 0
speed = 0.2 # 이동 속도 0.2 m/s

# 3. 시뮬레이션 루프
while simulation_app.is_running():
    world.step(render=True)
    
    if current_wp_idx < len(waypoints):
        target_pos = waypoints[current_wp_idx]
        current_pos, _ = obj.get_world_pose()
        
        diff = target_pos - current_pos
        dist = np.linalg.norm(diff)
        
        if dist < 0.005: # 도달 확인
            current_wp_idx += 1
            print(f"웨이포인트 {current_wp_idx}/{len(waypoints)} 도달")
        else:
            dt = world.get_physics_dt()
            move_step = (diff / dist) * speed * dt
            
            if np.linalg.norm(move_step) > dist:
                new_pos = target_pos
            else:
                new_pos = current_pos + move_step
                
            obj.set_world_pose(position=new_pos)
            
simulation_app.close()
