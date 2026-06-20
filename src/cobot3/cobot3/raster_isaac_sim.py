from omni.isaac.kit import SimulationApp

# Isaac Sim 앱 시작
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid, VisualCylinder
import numpy as np

# 월드 생성 (단위: 미터)
world = World(stage_units_in_meters=1.0)

# 1. 1m x 1m 판 (바닥) 생성
plane = world.scene.add(
    VisualCuboid(
        prim_path="/World/Plane",
        name="plane",
        position=np.array([0, 0, -0.01]),
        scale=np.array([1.0, 1.0, 0.02]),
        color=np.array([0.7, 0.7, 0.7])
    )
)

# 2. 150mm(0.15m) 직경의 원형 물체 (로봇의 엔드이펙터/검사 센서 모사)
radius = 0.075 # 반지름 75mm
obj = world.scene.add(
    VisualCylinder(
        prim_path="/World/Object",
        name="object",
        position=np.array([-0.5 + radius, -0.5 + radius, 0.01]),
        radius=radius,
        height=0.02,
        color=np.array([1.0, 0.2, 0.2])
    )
)

# 3. RASTER 경로 생성 (Zig-Zag)
waypoints = []
# 1m x 1m 범위를 물체의 크기를 고려하여 끝까지 닿도록 시작/끝 좌표 설정
x_start, x_end = -0.5 + radius, 0.5 - radius
y_start, y_end = -0.5 + radius, 0.5 - radius

# 직경이 150mm이므로 간격(Step)을 120mm (0.12m) 정도로 하여 빈틈없이 스캔되도록 설정
step = 0.12 
y = y_start
direction = 1

while y <= y_end:
    if direction == 1:
        waypoints.append(np.array([x_start, y, 0.01]))
        waypoints.append(np.array([x_end, y, 0.01]))
    else:
        waypoints.append(np.array([x_end, y, 0.01]))
        waypoints.append(np.array([x_start, y, 0.01]))
    
    y += step
    direction *= -1 # 좌우 방향 반전

# 맨 위쪽에 다다르지 못했다면 마지막 경로 추가
if y - step < y_end:
    y = y_end
    if direction == 1:
        waypoints.append(np.array([x_start, y, 0.01]))
        waypoints.append(np.array([x_end, y, 0.01]))
    else:
        waypoints.append(np.array([x_end, y, 0.01]))
        waypoints.append(np.array([x_start, y, 0.01]))

# 시뮬레이션 초기화
world.reset()

current_wp_idx = 0
speed = 0.2 # 이동 속도 (0.2 m/s)

print(f"총 {len(waypoints)}개의 웨이포인트가 생성되었습니다. 시뮬레이션을 시작합니다.")

# 4. 시뮬레이션 루프 (경로 따라가기)
while simulation_app.is_running():
    world.step(render=True)
    
    if current_wp_idx < len(waypoints):
        target_pos = waypoints[current_wp_idx]
        current_pos, _ = obj.get_world_pose()
        
        # 방향 벡터 및 거리 계산
        diff = target_pos - current_pos
        dist = np.linalg.norm(diff)
        
        if dist < 0.005: # 목표 위치 도달 허용 오차 (5mm)
            current_wp_idx += 1
            print(f"웨이포인트 {current_wp_idx}/{len(waypoints)} 도달")
        else:
            # 지정된 속도로 프레임(dt) 당 이동 계산
            dt = world.get_physics_dt()
            move_step = (diff / dist) * speed * dt
            
            if np.linalg.norm(move_step) > dist:
                new_pos = target_pos
            else:
                new_pos = current_pos + move_step
                
            obj.set_world_pose(position=new_pos)
    else:
        # 경로 순회 완료
        pass
        
simulation_app.close()
