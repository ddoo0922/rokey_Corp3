import sys
import os
import numpy as np
from isaacsim import SimulationApp

# 시스템 ROS2 경로 제거 (Isaac Sim 내부 rclpy 강제 사용)
import sys
import os
if "PYTHONPATH" in os.environ:
    os.environ["PYTHONPATH"] = ":".join([p for p in os.environ["PYTHONPATH"].split(":") if "/opt/ros" not in p])
sys.path = [p for p in sys.path if "/opt/ros" not in p]

simulation_app = SimulationApp({"headless": False})

# ROS2 브릿지 활성화 및 rclpy 임포트
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

try:
    import rclpy
    from std_msgs.msg import Float64
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("[WARNING] rclpy 모듈을 찾을 수 없습니다. ROS2 퍼블리시가 비활성화됩니다.")

from omni.isaac.core import World
from omni.isaac.core.objects import VisualSphere
from isaacsim.core.prims import SingleArticulation
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.sensor import ContactSensor

# 스크립트 위치 기준 경로 설정
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_POLISHING_DIR = os.path.dirname(_SCRIPT_DIR)  # src/robotarm_polishing
_SRC_DIR = os.path.dirname(_POLISHING_DIR)      # src

# RMPFlow 컨트롤러 경로 추가
sys.path.append(os.path.join(_POLISHING_DIR, "M0609", "rmpflow"))
from m0609_rmpflow_controller import RMPFlowController

ROBOT_USD_PATH = os.path.join(_SCRIPT_DIR, "m0609_polishing.usd")
PLY_PATH = os.path.join(_SRC_DIR, "scan_project", "output", "mesh", "real_camera_surface_points.ply")

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

def generate_raster_path(points, line_spacing=0.015):
    # 1. 탑 뷰(Top-down)에서 물체 영역 찾기
    min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
    min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])
    
    raster_path = []
    left_to_right = True
    
    y = min_y
    while y <= max_y:
        # 현재 Y 라인 부근의 포인트들 찾기
        strip_mask = (points[:, 1] >= y) & (points[:, 1] < y + line_spacing)
        strip_points = points[strip_mask]
        
        if len(strip_points) > 0:
            # X축 방향으로 1.5cm(0.015) 간격으로 촘촘하게 샘플링
            x_steps = np.arange(min_x, max_x, 0.005)
            if not left_to_right:
                x_steps = x_steps[::-1]
                
            for x in x_steps:
                # X 부근의 포인트들 찾기
                cell_mask = (strip_points[:, 0] >= x - 0.01) & (strip_points[:, 0] <= x + 0.01) 
                cell_points = strip_points[cell_mask]
                
                if len(cell_points) > 0:
                    # [핵심] 해당 X, Y 영역에서 가장 Z가 높은(Top Surface) 점만 추출!
                    highest_point = cell_points[np.argmax(cell_points[:, 2])]
                    raster_path.append(highest_point)
                    
        y += line_spacing
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
    # m0609_polishing.usd 파일 구조에 맞게 경로 지정
    robot_prim_path = "/World/M0609/m0609/m0609"
    robot_articulation = SingleArticulation(prim_path=robot_prim_path, name="m0609_robot")
    
    # 스캔 데이터 로드 (원본 포인트 클라우드)
    raw_points = load_ply_points(PLY_PATH)
    
    # [중요] 로봇 팔(M0609)의 최대 작업 반경(0.9m)에 비해 매직마우스 스캔 데이터(가로세로 약 1m)가 너무 큽니다.
    # 요청에 따라 물체의 크기를 30%(0.3)로 대폭 줄였습니다.
    raw_points = raw_points * 0.3
    
    # 마우스 물체를 바닥에서 띄우기 위한 Z축 오프셋 (15cm)
    Z_OFFSET_MOUSE = 0.15
    raw_points[:, 2] += Z_OFFSET_MOUSE
    
    print(f"Loaded and scaled {len(raw_points)} points from scan data.")
    
    # [추가] 외부 파일에서 원본 메쉬(형체)를 직접 불러와 시각화 (포인트 클라우드와 동일한 0.3 스케일 적용)
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    
    ORIGINAL_MESH_PATH = os.path.join(_SRC_DIR, "scan_project", "output", "mesh", "real_camera_surface_mesh.ply")
    mesh_points, mesh_faces = load_ply_mesh(ORIGINAL_MESH_PATH)
    mesh_points = mesh_points * 0.3
    mesh_points[:, 2] += Z_OFFSET_MOUSE # 메쉬도 동일하게 띄움
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
        
    # 로봇이 실제로 따라갈 정렬된 지그재그(Raster) 촘촘한 경로 생성
    points = generate_raster_path(raw_points, line_spacing=0.015)
    print(f"Generated raster path with {len(points)} points for robot tracking.")
    
    # 접촉 센서 부착 (어드미턴스 제어용)
    # 패드가 독립된 강체가 되었으므로, 빨간색 패드 부품에 직접 센서를 달아야 합니다.
    pad_path = "/World/M0609/m0609/m0609/sander_pad/pad_visual/sander_ref/OnRobot_Sander_v2/tn__104327_"
    try:
        from omni.isaac.core.utils.prims import get_prim_at_path
        if not get_prim_at_path(pad_path):
            pad_path = "/World/M0609/m0609/m0609/link_6/quick_mount/sanding_kit/OnRobot_Sander_v2/tn__104327_"
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
        
    # 1-2. 스펀지처럼 반발력이 없는 물리 재질 생성 (PhysicsMaterial 사용)
    from omni.isaac.core.materials import PhysicsMaterial
    from omni.isaac.core.prims import GeometryPrim
    no_bounce_material = PhysicsMaterial(
        prim_path="/World/NoBounceMaterial",
        dynamic_friction=0.0,
        static_friction=0.0,
        restitution=0.0
    )
    
    # 1-3. 샌더 패드(충돌체)에 직접 재질 적용
    pad_geom = GeometryPrim(prim_path=pad_path)
    if pad_geom.is_valid():
        pad_geom.apply_physics_material(no_bounce_material)
        
    # 1-4. 타겟 물체(마우스)에도 직접 재질 적용
    target_geom = GeometryPrim(prim_path="/World/MagicMouseOriginal")
    if target_geom.is_valid():
        target_geom.apply_physics_material(no_bounce_material)
    # -------------------------------------------------------------------------
    # (실제 물리 모터(Revolute Joint) 추가 코드는 로봇 Articulation 트리를 붕괴시켜 로봇이 주저앉는 원인이 되므로 삭제함)
    
    pad_prim = stage.GetPrimAtPath(pad_path)
    if pad_prim.IsValid():
        # 인스턴스 프록시(Instance Proxy) 오류 방지: pad_path 상위의 모든 인스턴스 속성 해제
        parts = pad_path.split('/')
        for i in range(2, len(parts)+1):
            curr_path = '/'.join(parts[:i])
            p = stage.GetPrimAtPath(curr_path)
            if p.IsValid() and p.IsInstance():
                p.SetInstanceable(False)
                
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

    link_6_path = "/World/M0609/m0609/m0609/link_6"
    p_link6_init, q_link6_init = get_world_pose(link_6_path)
    # (동적 측정 삭제: USD Xform 원점이 실제 메쉬 끝단이 아니라 0.8cm 부근에 찍혀 있어서 오차가 발생함)
    # ---------------------------------------------
    
    # RMPFlow 제어기 초기화
    controller = RMPFlowController(
        name="polishing_controller",
        robot_articulation=robot_articulation,
        urdf_path=os.path.join(_POLISHING_DIR, "M0609", "doosan-robot2", "urdf", "m0609_isaac_sim.urdf"),
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
    current_path_idx_float = 0.0 # 부드러운 보간을 위한 실수형 인덱스
    
    # 어드미턴스 제어용 변수 (안정성 강화)
    z_offset = 0.02  # 처음에는 표면에서 2cm 떨어져서 시작 (오래 기다리지 않도록 시간 단축)
    z_vel = 0.0
    admittance_mass = 1.0
    admittance_damping = 50.0 # 진동 방지 및 부드러운 힘 조절을 위한 댐핑
    admittance_stiffness = 0.0
    target_normal_force = 5.0 # 목표 접촉력 5N
    
    dt = 1.0 / 60.0 # 시뮬레이션 물리 스텝 (60Hz)
    
    # RViz 스타일의 '앞으로 갈 경로(Future Path)' 선분(BasisCurves) 생성
    import omni.usd
    from pxr import UsdGeom, Vt, Gf
    stage = omni.usd.get_context().get_stage()
    
    future_path_prim = UsdGeom.BasisCurves.Define(stage, "/World/FuturePath")
    future_path_prim.CreateTypeAttr().Set(UsdGeom.Tokens.linear)
    future_path_prim.CreateWidthsAttr().Set([0.010]) # 선 두께
    future_path_prim.CreateDisplayColorAttr().Set([(0.0, 1.0, 0.0)]) # 연두색 선
    
    from omni.isaac.core.utils.xforms import get_world_pose
    from omni.isaac.core.utils.prims import get_prim_at_path
    
    if ROS2_AVAILABLE:
        rclpy.init(args=None)
        ros_node = rclpy.create_node('isaac_sim_polishing_node')
        force_pub = ros_node.create_publisher(Float64, '/contact_force/data', 10)
        print("[INFO] ROS2 /contact_force 토픽 퍼블리셔가 시작되었습니다.")
        
    log_file_path = os.path.join(_SCRIPT_DIR, "force_log.csv")
    with open(log_file_path, "w") as f:
        f.write("step,force\n")
        
    step_count = 0
    
    while simulation_app.is_running():
        world.step(render=True)
        
        if world.is_playing() and current_target_idx < len(points):
            # [핵심] 점과 점 사이를 부드럽게 이어주는 선형 보간(Linear Interpolation)
            if current_target_idx < len(points) - 1:
                progress = current_path_idx_float - current_target_idx
                p0 = points[current_target_idx]
                p1 = points[current_target_idx + 1]
                target_pos = p0 * (1.0 - progress) + p1 * progress
            else:
                target_pos = points[current_target_idx]
            
            # [RMPflow 동기화] 베이스 좌표계 동기화 (로봇을 Z=0.2로 올림)
            controller._motion_policy.set_robot_base_pose(
                robot_position=np.array([-0.45, 0.0, 0.2]),
                robot_orientation=controller._default_orientation
            )
            
            # 표면 법선 벡터(Normal) 계산: 가상의 곡률 중심에서 타겟을 향하는 방향
            normal = target_pos - center_of_curvature
            normal = normal / np.linalg.norm(normal)
            
            # --- 동적 TCP 제어 ---
            # 1. 목표 회전: 샌더가 일자(Straight) 형태이므로 로봇 끝단(link_6)의 Z축을 바로 표면 안쪽(-normal)으로 일치시킵니다.
            base_orientation = z_align_quat(-normal) # [w, x, y, z] 형태
            r_base = R.from_quat([base_orientation[1], base_orientation[2], base_orientation[3], base_orientation[0]])
            r_target_link6 = r_base # 90도 회전 오프셋 제거!
            
            quat_target = r_target_link6.as_quat() # [x,y,z,w]
            target_orientation = np.array([quat_target[3], quat_target[0], quat_target[1], quat_target[2]]) # [w,x,y,z]
            
            # 2. 어드미턴스 제어: 센서 힘 읽기
            contact_reading = contact_sensor.get_current_frame()
            current_force = np.linalg.norm(contact_reading['force']) if contact_reading and 'force' in contact_reading else 0.0
            
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
                from omni.isaac.core.utils.types import ArticulationAction
                # 상태값이 아닌 실제 모터의 목표 속도(PD 타겟)를 설정해야 합니다.
                pad_action = ArticulationAction(
                    joint_velocities=np.array([5.0]), # 속도 제어
                    joint_indices=np.array([6])
                )
                robot_articulation.apply_action(pad_action)
                
            step_count += 1
            if step_count % 30 == 0:
                print(f"[물리 접촉력] : {current_force:5.1f} N")
                with open(log_file_path, "a") as f:
                    f.write(f"{step_count},{current_force}\n")
                    
            if ROS2_AVAILABLE:
                msg = Float64()
                msg.data = float(current_force)
                force_pub.publish(msg)
                rclpy.spin_once(ros_node, timeout_sec=0.0)
            
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
                
                # 매 스텝(0.016초)마다 조금씩 전진 (15스텝에 1칸 이동)
                current_path_idx_float += (1.0 / 15.0)
                current_target_idx = int(current_path_idx_float)
                
                if current_target_idx > 0 and current_target_idx % 50 == 0 and step_count % 15 == 0:
                    print(f"Tracking point {current_target_idx}/{len(points)}")

if __name__ == "__main__":
    main()
    if 'ROS2_AVAILABLE' in globals() and ROS2_AVAILABLE:
        rclpy.shutdown()
    simulation_app.close()