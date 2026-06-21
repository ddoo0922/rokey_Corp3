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

def generate_raster_path(points, line_spacing=0.05):
    # Y축 기준으로 일정 간격(strip)으로 나눈 뒤, 각 strip 내에서 X축 기준으로 정렬하여 지그재그(Raster) 경로 생성
    min_y = np.min(points[:, 1])
    max_y = np.max(points[:, 1])
    num_lines = int(np.ceil((max_y - min_y) / line_spacing))
    
    raster_path = []
    left_to_right = True
    
    for i in range(num_lines):
        y_start = min_y + i * line_spacing
        y_end = y_start + line_spacing
        strip_mask = (points[:, 1] >= y_start) & (points[:, 1] < y_end)
        strip_points = points[strip_mask]
        
        if len(strip_points) == 0:
            continue
            
        sorted_indices = np.argsort(strip_points[:, 0])
        if not left_to_right:
            sorted_indices = sorted_indices[::-1] # 반대 방향으로 정렬
            
        strip_points_sorted = strip_points[sorted_indices]
        
        # 로봇이 따라가기 쉽도록 2cm 간격으로 다운샘플링
        downsampled_strip = [strip_points_sorted[0]]
        for p in strip_points_sorted[1:]:
            if np.linalg.norm(p - downsampled_strip[-1]) >= 0.02:
                downsampled_strip.append(p)
                
        raster_path.extend(downsampled_strip)
        left_to_right = not left_to_right
        
    return np.array(raster_path)

def main():
    world = World(stage_units_in_meters=1.0)
    
    # 조명
    create_prim(
        prim_path="/World/DomeLight",
        prim_type="DomeLight",
        attributes={"inputs:intensity": 1000.0, "inputs:color": (1.0, 1.0, 1.0)}
    )
    
    # 로봇 로드 (포인트 클라우드 중앙에 겹치지 않도록 뒤로 충분히 이동: -0.8m)
    create_prim(
        prim_path="/World/M0609",
        prim_type="Xform",
        position=np.array([-0.8, 0.0, 0.0]),
        usd_path=ROBOT_USD_PATH
    )
    
    # 실제 조인트 컨트롤을 위한 Articulation 생성
    # assemble_robot.py에서 m0609.usd가 /World/m0609/m0609 구조로 들어갔으므로
    # 현재 경로는 /World/M0609/m0609/m0609 가 됩니다. (확인 필요)
    # 안전하게 "/World/M0609/m0609/m0609" 로 접근
    robot_prim_path = "/World/M0609/m0609/m0609"
    robot_articulation = SingleArticulation(prim_path=robot_prim_path, name="m0609_robot")
    
    # 스캔 데이터 로드 (원본 포인트 클라우드)
    raw_points = load_ply_points(PLY_PATH)
    print(f"Loaded {len(raw_points)} points from scan data.")
    
    # 빨간 점들은 원본 포인트 클라우드 그대로 렌더링하여 형태 보존
    for i, p in enumerate(raw_points[::10]):
        world.scene.add(
            VisualSphere(
                prim_path=f"/World/Targets/Point_{i}",
                name=f"point_{i}",
                position=p,
                radius=0.005,
                color=np.array([1.0, 0.0, 0.0])
            )
        )
        
    # 로봇이 실제로 따라갈 정렬된 지그재그(Raster) 경로 생성
    points = generate_raster_path(raw_points, line_spacing=0.05)
    print(f"Generated raster path with {len(points)} points for robot tracking.")
    
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
    
    # RViz 스타일의 '앞으로 갈 경로(Future Path)' 선분(BasisCurves) 생성
    import omni.usd
    from pxr import UsdGeom, Vt, Gf
    stage = omni.usd.get_context().get_stage()
    
    future_path_prim = UsdGeom.BasisCurves.Define(stage, "/World/FuturePath")
    future_path_prim.CreateTypeAttr().Set(UsdGeom.Tokens.linear)
    future_path_prim.CreateWidthsAttr().Set([0.005]) # 선 두께
    future_path_prim.CreateDisplayColorAttr().Set([(0.0, 1.0, 0.0)]) # 연두색 선
    
    from omni.isaac.core.utils.xforms import get_world_pose
    from omni.isaac.core.utils.prims import get_prim_at_path
    
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
            
            # 현재 로봇 끝단(link_6)의 실제 위치 가져오기
            link_6_path = "/World/M0609/m0609/m0609/link_6"
            if get_prim_at_path(link_6_path):
                tcp_pos, _ = get_world_pose(link_6_path)
                
                # 앞으로 훑고 지나갈 미래의 경로(최대 500개 점)를 선으로 연결하여 시각화 (RViz Local Path 스타일)
                lookahead_pts = points[current_target_idx : current_target_idx + 500]
                if len(lookahead_pts) > 1:
                    # 점들을 Gf.Vec3f 형태로 변환하여 선분 데이터 업데이트
                    vec3f_pts = [Gf.Vec3f(p[0], p[1], p[2]) for p in lookahead_pts]
                    future_path_prim.GetPointsAttr().Set(Vt.Vec3fArray(vec3f_pts))
                    future_path_prim.GetCurveVertexCountsAttr().Set([len(lookahead_pts)])
                
                # 로봇 제어 속도를 기존처럼 빠르게 복구 (매 스텝마다 목표점 갱신)
                current_target_idx += 1
                
                if current_target_idx % 100 == 0:
                    print(f"Tracking point {current_target_idx}/{len(points)}")

if __name__ == "__main__":
    main()
    simulation_app.close()