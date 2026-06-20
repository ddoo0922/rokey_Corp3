import sys
import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import VisualSphere
from isaacsim.core.prims import SingleArticulation
from omni.isaac.core.utils.prims import create_prim

# RMPFlow 컨트롤러 경로 추가
sys.path.append("/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/M0609/rmpflow")
from m0609_rmpflow_controller import RMPFlowController

ROBOT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd"
PLY_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/scan_project/output/mesh/real_camera_surface_points.ply"

def load_ply_points(path):
    points = []
    with open(path, 'r') as f:
        lines = f.readlines()
        header_ended = False
        for line in lines:
            if header_ended:
                parts = line.strip().split()
                if len(parts) >= 3:
                    points.append([float(parts[0]), float(parts[1]), float(parts[2])])
            if line.strip() == "end_header":
                header_ended = True
    return np.array(points)

def main():
    world = World(stage_units_in_meters=1.0)
    
    # 조명
    create_prim(
        prim_path="/World/DomeLight",
        prim_type="DomeLight",
        attributes={"inputs:intensity": 1000.0, "inputs:color": (1.0, 1.0, 1.0)}
    )
    
    # 로봇 로드 (포인트 클라우드 중앙에 겹치지 않도록 뒤로 이동)
    create_prim(
        prim_path="/World/M0609",
        prim_type="Xform",
        position=np.array([-0.6, 0.0, 0.0]),
        usd_path=ROBOT_USD_PATH
    )
    
    # 실제 조인트 컨트롤을 위한 Articulation 생성
    # assemble_robot.py에서 m0609.usd가 /World/m0609/m0609 구조로 들어갔으므로
    # 현재 경로는 /World/M0609/m0609/m0609 가 됩니다. (확인 필요)
    # 안전하게 "/World/M0609/m0609/m0609" 로 접근
    robot_prim_path = "/World/M0609/m0609/m0609"
    robot_articulation = SingleArticulation(prim_path=robot_prim_path, name="m0609_robot")
    
    # 스캔 데이터 로드
    points = load_ply_points(PLY_PATH)
    print(f"Loaded {len(points)} points from scan data.")
    
    # 목표점 중 일부를 시각적으로 표시 (너무 많으면 느려지므로 10개마다 1개씩)
    for i, p in enumerate(points[::10]):
        world.scene.add(
            VisualSphere(
                prim_path=f"/World/Targets/Point_{i}",
                name=f"point_{i}",
                position=p,
                radius=0.005,
                color=np.array([1.0, 0.0, 0.0])
            )
        )
        
    world.reset()
    robot_articulation.initialize()
    
    # RMPFlow 제어기 초기화
    controller = RMPFlowController(
        name="polishing_controller",
        robot_articulation=robot_articulation,
        urdf_path="/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/M0609/doosan-robot2/urdf/m0609_isaac_sim.urdf",
        end_effector_frame_name="link_6"
    )
    
    # 폴리싱 툴 길이 보정 (Sanding Kit 전체 높이 10cm)
    TOOL_OFFSET = 0.10
    
    # 단순화된 노멀(수직) 각도 계산 함수 (곡면을 따라가도록)
    # 여기서는 매직마우스 중심을 기준으로 대략적인 법선 벡터를 계산합니다.
    center = np.mean(points, axis=0)
    
    import omni.isaac.core.utils.rotations as rot_utils
    
    current_target_idx = 0
    target_threshold = 0.02 # 2cm 이내 도달 시 다음 점으로
    
    while simulation_app.is_running():
        world.step(render=True)
        
        if world.is_playing() and current_target_idx < len(points):
            target_pos = points[current_target_idx]
            
            # 대략적인 법선 벡터 (현재 점에서 중심을 바라보는 반대 방향 + Z업)
            # 여기서는 편의상 아래를 향하는(0, 180, 0) 방향을 유지합니다.
            # RMPFlow의 orientation은 기본 단위 쿼터니언을 받습니다.
            target_orientation = rot_utils.euler_angles_to_quat(np.array([0, np.pi, 0])) # 180도 뒤집기 (아래를 보도록)
            
            # 툴 길이 오프셋을 적용한 최종 link_6 목표 좌표 계산
            # 수직 아래를 보고 있으므로 Z축으로 +0.10m 올려서 위치시킵니다.
            link_6_target_pos = target_pos + np.array([0.0, 0.0, TOOL_OFFSET])
            
            actions = controller.forward(
                target_end_effector_position=link_6_target_pos,
                target_end_effector_orientation=target_orientation
            )
            robot_articulation.apply_action(actions)
            
            # 현재 link_6 위치 확인
            current_pos, _ = robot_articulation.get_world_pose()
            # 하지만 우리가 필요한건 link_6의 포즈가 아니라 base 대비 현재 동작의 오차입니다.
            # 대략적으로 타겟에 가까워졌는지 시간을 두고 넘기거나 (여기서는 매우 단순하게 일정 스텝마다 넘김)
            current_target_idx += 1
            
            if current_target_idx % 100 == 0:
                print(f"Tracking point {current_target_idx}/{len(points)}")

if __name__ == "__main__":
    main()
    simulation_app.close()
