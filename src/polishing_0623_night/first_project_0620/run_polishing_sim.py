import sys
import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import VisualSphere
from isaacsim.core.prims import SingleArticulation
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.sensor import ContactSensor

# RMPFlow 컨트롤러 경로 추가
sys.path.append("/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/M0609/rmpflow")
from m0609_rmpflow_controller import RMPFlowController

ROBOT_USD_PATH = "/home/rokey/rokey_Corp3/src/robotarm_polishing/first_project_0620/revolution.usd"
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

def z_align_quat(z_vec):
    """Z축을 원하는 방향(z_vec)으로 정렬시키는 쿼터니언(w, x, y, z)을 계산합니다."""
    z_vec = z_vec / np.linalg.norm(z_vec)
    up = np.array([0.0, 0.0, 1.0])
    axis = np.cross(up, z_vec)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-6:
        if z_vec[2] > 0:
            return np.array([1.0, 0.0, 0.0, 0.0])
        else:
            return np.array([0.0, 1.0, 0.0, 0.0])
    axis = axis / axis_norm
    angle = np.arccos(np.clip(np.dot(up, z_vec), -1.0, 1.0))
    w = np.cos(angle / 2)
    s = np.sin(angle / 2)
    return np.array([w, axis[0]*s, axis[1]*s, axis[2]*s])
def load_ply_mesh(path):
    points = []
    faces = []
    num_vertices = 0
    num_faces = 0
    header_ended = False
    in_vertices = False
    in_faces = False
    vertex_count = 0
    face_count = 0
    
    with open(path, 'r') as f:
        lines = f.readlines()
        
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if not header_ended:
            if line.startswith("element vertex"):
                num_vertices = int(line.split()[2])
            elif line.startswith("element face"):
                num_faces = int(line.split()[2])
            elif line == "end_header":
                header_ended = True
                if num_vertices > 0:
                    in_vertices = True
            continue
            
        if in_vertices:
            parts = line.split()
            points.append([float(parts[0]), float(parts[1]), float(parts[2])])
            vertex_count += 1
            if vertex_count == num_vertices:
                in_vertices = False
                if num_faces > 0:
                    in_faces = True
            continue
            
        if in_faces:
            parts = line.split()
            count = int(parts[0])
            face = [int(x) for x in parts[1:count+1]]
            if len(face) == 3: # 삼각형만 지원
                faces.append(face)
            elif len(face) == 4: # 사각형인 경우 2개의 삼각형으로 분할
                faces.append([face[0], face[1], face[2]])
                faces.append([face[0], face[2], face[3]])
            face_count += 1
            if face_count == num_faces:
                in_faces = False
            continue
            
    return np.array(points, dtype=np.float32), faces

def create_usd_mesh(stage, prim_path, points, faces):
    from pxr import UsdGeom, Vt, Gf
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    
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
    
    # 포인트 클라우드(빨강)와 대비되도록 원본 형체는 파란색 계열로 렌더링
    mesh.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.2, 0.5, 0.8)]))
    
    # 물리적 충돌 속성(Collision API) 추가 (진짜 껍데기 생성)
    from pxr import UsdPhysics
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
    # 굴곡진 표면을 정확히 반영하기 위해 충돌 근사를 'none'(Triangle Mesh)으로 설정
    mesh_collision.CreateApproximationAttr().Set("none")
    
    return mesh

def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    
    # 로봇이 흰색이라 잘 보이도록 어두운 톤의 바닥 시각화용 박스 추가
    from omni.isaac.core.objects import VisualCuboid
    world.scene.add(
        VisualCuboid(
            prim_path="/World/DarkFloor",
            name="dark_floor",
            position=np.array([0.0, 0.0, -0.01]),
            scale=np.array([10.0, 10.0, 0.02]),
            color=np.array([0.15, 0.15, 0.15])
        )
    )
    
    # 조명
    create_prim(
        prim_path="/World/DomeLight",
        prim_type="DomeLight",
        attributes={"inputs:intensity": 1000.0, "inputs:color": (1.0, 1.0, 1.0)}
    )
    
    # 로봇 로드 (요청에 따라 로봇을 지상에서 20cm 위로 살짝 올림)
    create_prim(
        prim_path="/World/M0609",
        prim_type="Xform",
        position=np.array([-0.45, 0.0, 0.2]),
        usd_path=ROBOT_USD_PATH
    )
    
    # 실제 조인트 컨트롤을 위한 Articulation 생성
    # revolution.usd 파일은 기존 m0609_polishing.usd를 /World/m0609_polishing 경로에 포함하고 있습니다.
    # 따라서 create_prim을 통해 /World/M0609 에 불러오면 한 뎁스 더 깊어집니다.
    robot_prim_path = "/World/M0609/m0609_polishing/m0609/m0609"
    robot_articulation = SingleArticulation(prim_path=robot_prim_path, name="m0609_robot")
    
    # 스캔 데이터 로드 (원본 포인트 클라우드)
    raw_points = load_ply_points(PLY_PATH)
    
    # [중요] 로봇 팔(M0609)의 최대 작업 반경(0.9m)에 비해 매직마우스 스캔 데이터(가로세로 약 1m)가 너무 큽니다.
    # 요청에 따라 물체의 크기를 30%(0.3)로 대폭 줄였습니다.
    raw_points = raw_points * 0.3
    
    # 지상 배치 요청: 이전에 지하로 내렸던 코드를 삭제하여 물체를 바닥에 정상 배치합니다.
    print(f"Loaded and scaled {len(raw_points)} points from scan data.")
    
    # [추가] 외부 파일에서 원본 메쉬(형체)를 직접 불러와 시각화 (포인트 클라우드와 동일한 0.3 스케일 적용)
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    
    ORIGINAL_MESH_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/scan_project/output/mesh/real_camera_surface_mesh.ply"
    mesh_points, mesh_faces = load_ply_mesh(ORIGINAL_MESH_PATH)
    mesh_points = mesh_points * 0.3
    create_usd_mesh(stage, "/World/MagicMouseOriginal", mesh_points, mesh_faces)
    
    # 빨간 점들은 원본 포인트 클라우드 그대로 렌더링하여 형태 보존
    # 너무 많은 구를 생성하면 로딩 시간이 길어지므로 100개 중 1개만 시각화합니다.
    for i, p in enumerate(raw_points[::100]):
        world.scene.add(
            VisualSphere(
                prim_path=f"/World/Targets/Point_{i}",
                name=f"point_{i}",
                position=p,
                radius=0.005, # 개수가 줄어든 대신 약간 더 크게 표시
                color=np.array([1.0, 0.0, 0.0])
            )
        )
        
    # 로봇이 실제로 따라갈 정렬된 지그재그(Raster) 경로 생성 (검토를 위해 라인 간격을 넓혀 점 개수를 줄임)
    points = generate_raster_path(raw_points, line_spacing=0.1)
    print(f"Generated raster path with {len(points)} points for robot tracking.")
    
    # 접촉 센서 부착 (어드미턴스 제어용)
    # 패드가 독립된 강체가 되었으므로, 빨간색 패드 부품에 직접 센서를 달아야 합니다.
    pad_path = "/World/M0609/m0609_polishing/m0609/m0609/link_6/sanding_kit/OnRobot_Sander_v2/tn__114850_"
    try:
        from omni.isaac.core.utils.prims import get_prim_at_path
        if not get_prim_at_path(pad_path):
            pad_path = "/World/M0609/World/m0609/m0609/link_6/sanding_kit/OnRobot_Sander_v2/tn__114850_"
    except:
        pass
    # 1. ContactSensor를 생성하기 전에 먼저 ContactReportAPI를 적용합니다 (뒷북 방지)
    import omni.usd
    from pxr import PhysxSchema
    stage = omni.usd.get_context().get_stage()
    
    # --- 1번 개선책: 부드러운 물리 재질(Soft Material) 및 물리 씬 안정화 적용 ---
    from pxr import UsdShade, UsdPhysics
    
    # 1-1. 폭발적인 튕김 방지를 위해 Physics Scene의 최대 반발 속도 제한
    physics_scene_prim = stage.GetPrimAtPath("/physicsScene")
    if physics_scene_prim.IsValid():
        physx_scene = PhysxSchema.PhysxSceneAPI(physics_scene_prim)
        # 물체가 파고들었을 때 튕겨내는 최대 속도를 5cm/s로 제한 (기본값은 무제한이라 폭발함)
        from pxr import Sdf
        physx_scene.GetPrim().CreateAttribute("physxScene:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float).Set(0.05)
        
    # 1-2. 스펀지처럼 반발력이 없는 물리 재질 생성
    material_path = "/World/SoftMaterial"
    material_prim = stage.DefinePrim(material_path, "Material")
    physics_material = UsdPhysics.MaterialAPI.Apply(material_prim)
    physics_material.CreateDynamicFrictionAttr(0.5) # 물리 모터에 의한 실제 샌딩 마찰력 복구
    physics_material.CreateStaticFrictionAttr(0.5)
    physics_material.CreateRestitutionAttr(0.0) # 반발계수 0 (통통 튀는 현상 제거)
    
    # 1-3. 샌더 전체에 재질 바인딩 (빨간색 패드 포함)
    sander_path = "/World/M0609/m0609_polishing/m0609/m0609/link_6/sanding_kit/OnRobot_Sander_v2"
    sander_prim = stage.GetPrimAtPath(sander_path)
    if sander_prim.IsValid():
        UsdShade.MaterialBindingAPI.Apply(sander_prim).Bind(UsdShade.Material(material_prim), UsdShade.Tokens.weakerThanDescendants, "physics")
        
    # 1-4. 타겟 물체(마우스)에도 재질 바인딩
    target_mesh_prim = stage.GetPrimAtPath("/World/TargetObject")
    if target_mesh_prim.IsValid():
        UsdShade.MaterialBindingAPI.Apply(target_mesh_prim).Bind(UsdShade.Material(material_prim), UsdShade.Tokens.weakerThanDescendants, "physics")
    # -------------------------------------------------------------------------
    # (실제 물리 모터(Revolute Joint) 추가 코드는 로봇 Articulation 트리를 붕괴시켜 로봇이 주저앉는 원인이 되므로 삭제함)
    
    pad_prim = stage.GetPrimAtPath(pad_path)
    if pad_prim.IsValid():
        PhysxSchema.PhysxContactReportAPI.Apply(pad_prim)
        
    # 2. API가 적용된 프림에 센서를 생성합니다.
    contact_sensor = ContactSensor(
        prim_path=pad_path + "/contact_sensor",
        name="pad_contact_sensor",
        frequency=60,
        translation=np.array([0, 0, 0]),
    )
    
    world.reset()
    robot_articulation.initialize()
    contact_sensor.initialize()

    # 샌딩 패드가 로봇의 Articulation 트리에 병합되었으므로 추가적인 initialize()가 필요하지 않습니다.
    # 터미널에서 로봇 관절의 일원으로서 패드의 모터 속도를 추출할 것입니다.
    
    # --- 동적 TCP 변환 초기화 (하드코딩 제거) ---
    from omni.isaac.core.utils.xforms import get_world_pose
    from scipy.spatial.transform import Rotation as R

    link_6_path = "/World/M0609/m0609_polishing/m0609/m0609/link_6"
    p_link6_init, q_link6_init = get_world_pose(link_6_path)
    # (동적 측정 삭제: USD Xform 원점이 실제 메쉬 끝단이 아니라 0.8cm 부근에 찍혀 있어서 오차가 발생함)
    # ---------------------------------------------
    
    # RMPFlow 제어기 초기화
    controller = RMPFlowController(
        name="polishing_controller",
        robot_articulation=robot_articulation,
        urdf_path="/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/M0609/doosan-robot2/urdf/m0609_isaac_sim.urdf",
        end_effector_frame_name="link_6"
    )
    
    # 툴 길이는 사용자가 측정한 10cm가 절대적으로 맞습니다.
    TOOL_OFFSET = 0.10
    
    # 곡면(Magic Mouse)의 곡률 중심을 근사하여 표면 법선(Normal)을 구하기 위한 기준점
    min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
    min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])
    min_z = np.min(points[:, 2])
    # 곡률 반경을 대략 30cm로 가정
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    center_of_curvature = np.array([center_x, center_y, min_z - 0.3])
    
    from omni.isaac.core.objects import VisualCuboid, VisualCylinder
    
    # 1. 로봇 단상 시각화 (Z=0 ~ Z=0.2)
    world.scene.add(
        VisualCylinder(
            prim_path="/World/Pedestal",
            name="robot_pedestal",
            position=np.array([-0.45, 0.0, 0.1]), # 0.2 높이의 중간 지점
            radius=0.12,
            height=0.2,
            color=np.array([0.3, 0.3, 0.3]) # 짙은 회색 단상
        )
    )
    

    # pyrefly: ignore [missing-import]
    import omni.isaac.core.utils.rotations as rot_utils
    
    current_target_idx = 0
    
    # 어드미턴스 제어용 변수 (안정성 강화)
    z_offset = 0.02  # 처음에는 표면에서 2cm 떨어져서 시작 (오래 기다리지 않도록 시간 단축)
    z_vel = 0.0
    admittance_mass = 1.0
    admittance_damping = 100.0 # 진동 방지를 위해 댐핑 증가
    admittance_stiffness = 0.0
    target_normal_force = 5.0 # 목표 접촉력 5N
    
    dt = 1.0 / 60.0 # 시뮬레이션 물리 스텝 (60Hz)
    
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
            
            # [RMPflow 동기화] 베이스 좌표계 동기화 (로봇을 Z=0.2로 올림)
            controller._motion_policy.set_robot_base_pose(
                robot_position=np.array([-0.45, 0.0, 0.2]),
                robot_orientation=controller._default_orientation
            )
            
            # 표면 법선 벡터(Normal) 계산: 가상의 곡률 중심에서 타겟을 향하는 방향
            normal = target_pos - center_of_curvature
            normal = normal / np.linalg.norm(normal)
            
            # --- 동적 TCP 제어 (직선형 툴 맞춤) ---
            # 1. 목표 회전: 스크린샷 확인 결과 툴이 L자가 아니라 일자(Straight) 형태입니다!
            # 따라서 90도 꺾을 필요 없이, 로봇 끝단(link_6)의 Z축을 바로 표면 안쪽(-normal)으로 일치시킵니다.
            base_orientation = z_align_quat(-normal) # [w, x, y, z] 형태
            r_base = R.from_quat([base_orientation[1], base_orientation[2], base_orientation[3], base_orientation[0]])
            r_target_link6 = r_base # 90도 회전 오프셋 제거!
            
            quat_target = r_target_link6.as_quat() # [x,y,z,w]
            target_orientation = np.array([quat_target[3], quat_target[0], quat_target[1], quat_target[2]]) # [w,x,y,z]
            
            # 2. 어드미턴스 제어: 센서 힘 읽기
            contact_reading = contact_sensor.get_current_frame()
            current_force = contact_reading['force'] if contact_reading and 'force' in contact_reading else 0.0
            
            force_error = current_force - target_normal_force
            accel = (force_error - admittance_damping * z_vel - admittance_stiffness * z_offset) / admittance_mass
            z_vel += accel * dt
            
            # 속도 제한: 물체에 쾅 부딪히거나 튕겨나가지 않도록 최대 속도를 2cm/s로 제한
            z_vel = np.clip(z_vel, -0.02, 0.02) 
            
            z_offset += z_vel * dt
            # 원래 하한선이 -0.01이었기 때문에 로봇이 마우스에 닿기 전에 하강을 멈췄습니다. 
            # -0.10으로 여유를 주어 확실히 접촉할 때까지 내려가도록 합니다.
            z_offset = np.clip(z_offset, -0.10, 0.10)
            
            # 3. 목표 위치 계산
            # 패드가 가야 할 최종 월드 위치 (어드미턴스 오프셋 적용)
            target_pad_pos = target_pos + normal * z_offset
            
            # link_6의 목표 위치는 표면(target_pad_pos)에서 툴의 길이(10cm)만큼 수직으로 떨어져야 합니다.
            link_6_target_pos = target_pad_pos + normal * TOOL_OFFSET
            
            actions = controller.forward(
                target_end_effector_position=link_6_target_pos,
                target_end_effector_orientation=target_orientation
            )
            robot_articulation.apply_action(actions)
            
            # 새로 추가하신 RevoluteJoint가 기존 로봇(m0609)의 7번째 관절(DOF)로 병합되었으므로,
            # RMPFlow가 제어하는 6개 관절 외에 7번째 관절(인덱스 6)에 직접 각속도를 쏴주어야 모터가 돌아갑니다.
            if robot_articulation.num_dof > 6:
                # 상태값이 아닌 실제 모터의 목표 속도(PD 타겟)를 설정해야 합니다.
                robot_articulation.set_joint_velocity_targets(
                    velocities=np.array([20.0]), # 약 1145 deg/s
                    joint_indices=np.array([6])
                )
                # 혹시 모를 내부 PD 충돌을 방지하기 위해 힘(Torque)도 지속적으로 가해줍니다.
                robot_articulation.set_joint_efforts(
                    efforts=np.array([5.0]), # 5 Nm 토크
                    joint_indices=np.array([6])
                )
            
            # 현재 로봇 끝단(link_6)의 실제 위치 가져오기
            link_6_path = "/World/M0609/m0609_polishing/m0609/m0609/link_6"
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
                
                if current_target_idx % 50 == 0:
                    print(f"Tracking point {current_target_idx}/{len(points)}")
                    print(f"  Target Pos: {target_pos}")
                    print(f"  Normal: {normal}")
                    print(f"  Z Offset: {z_offset:.4f}")
                    print(f"  Target Pad Pos: {target_pad_pos}")
                    print(f"  Link_6 Target Pos: {link_6_target_pos}")
                    print(f"  Link_6 Actual Pos: {tcp_pos}")
                    print(f"  Distance (Target vs Actual): {np.linalg.norm(link_6_target_pos - tcp_pos):.4f}")
                    print("-" * 50)

if __name__ == "__main__":
    main()
    simulation_app.close()