import os
import numpy as np
from omni.isaac.kit import SimulationApp

# Isaac Sim 앱 시작
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid, VisualCylinder, VisualSphere

# 1. 실제 Depth Camera로 스캔한 데이터(PLY) 로드
ply_path = "/home/rokey/cobot4_ws/rokey_Corp3/src/scan_project/output/mesh/real_camera_surface_points.ply"

points = []
with open(ply_path, 'r') as f:
    lines = f.readlines()
    header_ended = False
    for line in lines:
        if header_ended:
            parts = line.strip().split()
            if len(parts) >= 3:
                points.append([float(parts[0]), float(parts[1]), float(parts[2])])
        if line.strip() == "end_header":
            header_ended = True

points = np.array(points)
print(f"Loaded {len(points)} points from {ply_path}")

# 바운딩 박스 추출
x_min, y_min, z_min = np.min(points, axis=0)
x_max, y_max, z_max = np.max(points, axis=0)
center_m = np.mean(points, axis=0)

print(f"Scan Bounding Box: X({x_min:.3f} ~ {x_max:.3f}), Y({y_min:.3f} ~ {y_max:.3f}), Z({z_min:.3f} ~ {z_max:.3f})")

# 월드 생성
world = World(stage_units_in_meters=1.0)

# 조명(DomeLight) 추가
from omni.isaac.core.utils.prims import create_prim
create_prim(
    prim_path="/World/DomeLight",
    prim_type="DomeLight",
    attributes={"inputs:intensity": 1000.0, "inputs:color": (1.0, 1.0, 1.0)}
)

# 상공에 스캔용 Depth Camera 모형 생성 (시각적 확인용)
world.scene.add(
    VisualCuboid(
        prim_path="/World/CameraBody",
        name="camera_body",
        position=np.array([0.0, 0.0, 1.60]),
        scale=np.array([0.1, 0.1, 0.1]),
        color=np.array([0.2, 0.2, 0.2])
    )
)
world.scene.add(
    VisualCylinder(
        prim_path="/World/CameraLens",
        name="camera_lens",
        position=np.array([0.0, 0.0, 1.54]),
        radius=0.04,
        height=0.04,
        color=np.array([0.1, 0.8, 1.0]) # 파란색 렌즈
    )
)

# 시각적 참고를 위해 실제 스캔된 영역 크기만큼의 바닥을 하나 생성
size_x = x_max - x_min
size_y = y_max - y_min
size_z = z_max - z_min

# 바닥 판 (Z=0.0 기준)
world.scene.add(
    VisualCuboid(
        prim_path="/World/BasePlate",
        name="base_plate",
        position=np.array([center_m[0], center_m[1], -0.01]),
        scale=np.array([size_x if size_x > 0 else 1.0, size_y if size_y > 0 else 1.0, 0.02]),
        color=np.array([0.3, 0.3, 0.3])
    )
)

# 포인트 클라우드를 샘플링하여 시뮬레이션 환경에 렌더링 (매직마우스 형태 3D 시각화)
# 너무 많으면 버벅이므로 약 500개 내외로 샘플링
sample_step = max(1, len(points) // 500)
for i, pt in enumerate(points[::sample_step]):
    world.scene.add(
        VisualSphere(
            prim_path=f"/World/PointCloud/pt_{i}",
            name=f"pt_{i}",
            position=pt,
            radius=0.008,
            color=np.array([0.9, 0.9, 0.9])
        )
    )

# 150mm(0.15m) 직경 원형 툴
radius = 0.075
obj = world.scene.add(
    VisualCylinder(
        prim_path="/World/Sensor",
        name="sensor",
        position=np.array([x_min, y_min, z_max + 0.05]),
        radius=radius,
        height=0.01,
        color=np.array([0.2, 0.2, 1.0]) # 3D 추종은 파란색
    )
)

# 2. 3D Contour Following Raster 경로 생성
waypoints = []
x_start = x_min
x_end = x_max
y_start = y_min
y_end = y_max

step = 0.05 # Y축 간격 (촘촘한 3D 추종을 위해 50mm 간격)
sample_dist = 0.01 # X축을 이동할 때 10mm(0.01m) 단위로 샘플링하여 굴곡을 추종

y = y_start
direction = 1

def get_z_for_xy(tx, ty):
    # 가장 가까운 포인트 검색 (간단한 Nearest Neighbor)
    dists = (points[:, 0] - tx)**2 + (points[:, 1] - ty)**2
    nearest_idx = np.argmin(dists)
    return points[nearest_idx, 2]

while y <= y_end:
    x_vals = np.arange(x_start, x_end, sample_dist)
    if x_vals[-1] != x_end:
        x_vals = np.append(x_vals, x_end)
        
    if direction == -1:
        x_vals = x_vals[::-1]
        
    for x in x_vals:
        z = get_z_for_xy(x, y)
        waypoints.append(np.array([x, y, z + 0.01])) # 표면보다 1cm 위로 부상하여 이동
    
    y += step
    direction *= -1

print(f"Generated {len(waypoints)} 3D contour waypoints.")

# 시각적으로 웨이포인트(경로)를 표시 (원한다면 주석 해제)
# for i, wp in enumerate(waypoints[::10]): # 너무 많으니 10개당 1개만
#     world.scene.add(
#         VisualSphere(
#             prim_path=f"/World/WP_{i}",
#             name=f"wp_{i}",
#             position=wp,
#             radius=0.002,
#             color=np.array([0.0, 1.0, 0.0])
#         )
#     )

world.reset()

current_wp_idx = 0
speed = 0.1 # 이동 속도 0.1 m/s

# 3. 시뮬레이션 루프
while simulation_app.is_running():
    world.step(render=True)
    
    if current_wp_idx < len(waypoints):
        target_pos = waypoints[current_wp_idx]
        current_pos, _ = obj.get_world_pose()
        
        diff = target_pos - current_pos
        dist = np.linalg.norm(diff)
        
        if dist < 0.002: # 도달 허용 오차 (2mm)
            current_wp_idx += 1
            if current_wp_idx % 10 == 0:
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
