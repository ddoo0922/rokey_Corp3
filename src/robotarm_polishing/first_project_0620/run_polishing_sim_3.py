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
from omni.isaac.core.objects import FixedCuboid, VisualCylinder
from omni.isaac.core.materials import PhysicsMaterial
from isaacsim.core.prims import SingleArticulation
from omni.isaac.core.prims import GeometryPrim
from omni.isaac.sensor import ContactSensor
from pxr import PhysxSchema, UsdPhysics
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.xforms import get_world_pose
from scipy.spatial.transform import Rotation as R

# 스크립트 위치 기준 경로 설정
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_POLISHING_DIR = os.path.dirname(_SCRIPT_DIR)  # src/robotarm_polishing
_SRC_DIR = os.path.dirname(_POLISHING_DIR)      # src


# RMPFlow 컨트롤러 경로 추가
sys.path.append(os.path.join(_POLISHING_DIR, "M0609", "rmpflow"))
from m0609_rmpflow_controller import RMPFlowController

ROBOT_USD_PATH = os.path.join(_SCRIPT_DIR, "m0609_polishing.usd")

def z_align_quat(target_z):
    target_z = target_z / np.linalg.norm(target_z)
    default_z = np.array([0, 0, 1])
    axis = np.cross(default_z, target_z)
    axis_norm = np.linalg.norm(axis)
    
    if axis_norm < 1e-6:
        if np.dot(default_z, target_z) > 0:
            return np.array([1.0, 0.0, 0.0, 0.0]) # [w, x, y, z]
        else:
            return np.array([0.0, 1.0, 0.0, 0.0]) # 180도 회전
    
    axis = axis / axis_norm
    angle = np.arccos(np.clip(np.dot(default_z, target_z), -1.0, 1.0))
    half_angle = angle / 2.0
    s = np.sin(half_angle)
    return np.array([np.cos(half_angle), axis[0]*s, axis[1]*s, axis[2]*s]) # [w, x, y, z]

def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # 조명 추가
    create_prim(
        prim_path="/World/Light",
        prim_type="DomeLight",
        attributes={"inputs:intensity": 1000.0}
    )

    # 로봇 로드
    create_prim(
        prim_path="/World/M0609",
        prim_type="Xform",
        position=np.array([-0.45, 0.0, 0.2]),
        usd_path=ROBOT_USD_PATH
    )

    # 로봇의 인스턴스 속성 해제 (수정 권한 확보)
    m0609_prim = world.scene.stage.GetPrimAtPath("/World/M0609")
    if m0609_prim.IsValid() and m0609_prim.IsInstance():
        m0609_prim.SetInstanceable(False)

    robot_prim_path = "/World/M0609/m0609/m0609"
    robot_articulation = SingleArticulation(prim_path=robot_prim_path, name="m0609_robot")
    
    # 인터랙티브 타겟 생성 (사용자가 마우스로 드래그할 마커이자 물리적 접촉 대상)
    target_path = "/World/InteractiveTarget"
    target = FixedCuboid(
        prim_path=target_path,
        name="interactive_target",
        position=np.array([0.1, 0.0, 0.3]),
        scale=np.array([0.2, 0.2, 0.02]), # 면적을 조금 넓히고 두께를 줌
        color=np.array([0.0, 1.0, 0.0])   # 초록색
    )

    # [중요] 샌더와 물체가 닿았을 때 튕겨나오는 현상(반발력)을 물리법칙상 제거합니다.
    # 마찰 계수도 기존의 1/10 수준(0.05)으로 줄여 모터가 멈추지 않게 합니다.
    no_bounce_material = PhysicsMaterial(
        prim_path="/World/NoBounceMaterial",
        dynamic_friction=0.00,
        static_friction=0.00,
        restitution=0.0
    )
    target.apply_physics_material(no_bounce_material)

    # 타겟뿐만 아니라 로봇팔의 "샌더 패드" 자체에도 동일한 무반발 매테리얼을 씌워줍니다!
    pad_phys_path = "/World/M0609/m0609/m0609/sander_pad/pad_visual/sander_ref/OnRobot_Sander_v2/tn__104327_"
    pad_geom = GeometryPrim(prim_path=pad_phys_path)
    if pad_geom.is_valid():
        # 복잡한 USD 토큰 에러를 피하기 위해 Isaac Sim 기본 API 사용
        pad_geom.apply_physics_material(no_bounce_material)
        print(f"[INFO] 샌더 패드({pad_phys_path})에 무반발 매테리얼 적용 완료!")

    # 타겟 표면의 법선 방향을 시각적으로 보여주기 위한 작은 기둥
    VisualCylinder(
        prim_path=f"{target_path}/NormalVisual",
        name="normal_visual",
        position=np.array([0.0, 0.0, 0.05]), # 로컬 Z축으로 약간 띄움
        scale=np.array([0.02, 0.02, 0.1]),   # 얇고 짧은 빨간색 선
        color=np.array([1.0, 0.0, 0.0])      # 빨간색
    )

    # 샌더 패드에 접촉 센서 부착 (물리적 힘 측정)
    pad_path = "/World/M0609/m0609/m0609/sander_pad/pad_visual/sander_ref/OnRobot_Sander_v2/tn__104327_"
    pad_prim = world.scene.stage.GetPrimAtPath(pad_path)
    if pad_prim.IsValid():
        parts = pad_path.split('/')
        for i in range(2, len(parts)+1):
            curr_path = '/'.join(parts[:i])
            p = world.scene.stage.GetPrimAtPath(curr_path)
            if p.IsValid() and p.IsInstance():
                p.SetInstanceable(False)
        PhysxSchema.PhysxContactReportAPI.Apply(pad_prim)
        
    contact_sensor = ContactSensor(
        prim_path=pad_path + "/contact_sensor",
        name="pad_contact_sensor",
        frequency=60,
        translation=np.array([0, 0, 0]),
    )

    # RMPFlow 컨트롤러 초기화
    controller = RMPFlowController(
        name="polishing_controller",
        robot_articulation=robot_articulation,
        urdf_path=os.path.join(_POLISHING_DIR, "M0609", "doosan-robot2", "urdf", "m0609_isaac_sim.urdf"),
        end_effector_frame_name="link_6" # 다시 link_6 기준으로 복구
    )

    world.reset()
    robot_articulation.initialize()
    
    # ROS2 노드 초기화
    if ROS2_AVAILABLE:
        rclpy.init()
        ros_node = rclpy.create_node('polishing_force_node')
        force_pub = ros_node.create_publisher(Float64, '/contact_force', 10)
        print("[INFO] ROS2 /contact_force 토픽 퍼블리셔가 시작되었습니다.")
    
    # RMPFlow도 reset 필요
    controller.reset()

    # 사용자가 지정한 '샌더의 끝자락(Mesh)' 프림 경로
    pad_tip_mesh_path = "/World/M0609/m0609/m0609/sander_pad/pad_visual/sander_ref/OnRobot_Sander_v2/tn__104327_/tn__BossExtrude4_VI/Mesh"
    
    # link_6와 샌더 끝자락의 초기 위치/회전을 읽어와 정확한 '3D 로컬 벡터'를 계산합니다.
    link6_pos, link6_rot = get_world_pose("/World/M0609/m0609/m0609/link_6")
    pad_tip_pos, _ = get_world_pose(pad_tip_mesh_path)
    
    # get_world_pose의 쿼터니언은 [w, x, y, z] 순서이므로 scipy [x, y, z, w]로 변환
    r_link6 = R.from_quat([link6_rot[1], link6_rot[2], link6_rot[3], link6_rot[0]])
    
    # link_6 기준의 샌더 끝자락의 로컬 3D 위치 (X, Y, Z 축으로 얼마나 떨어져 있는지)
    local_tool_offset = r_link6.inv().apply(pad_tip_pos - link6_pos)
    print(f"[INFO] 샌더 끝자락 로컬 3D 오프셋: {local_tool_offset} m")

    # --- 관측용 파라미터 ---
    dt = 1.0 / 60.0

    print("==================================================")
    print("인터랙티브 추적 시뮬레이션을 시작합니다.")
    print("Isaac Sim 우측 'Stage' 창에서 'InteractiveTarget'을 선택하고")
    print("W(이동) 또는 E(회전) 툴을 이용해 마우스로 드래그해 보세요!")
    print("==================================================")

    step_count = 0
    log_file_path = os.path.join(_SCRIPT_DIR, "force_log.csv")
    with open(log_file_path, "w") as f:
        f.write("step,force\n")

    while simulation_app.is_running():
        world.step(render=True)
        
        # 1. 사용자가 드래그하는 InteractiveTarget의 현재 포즈 읽기
        target_pos, target_rot = get_world_pose(target_path)
        
        # 타겟의 로컬 Z축이 곧 '표면의 법선(Normal)' 방향이 됩니다.
        r_target = R.from_quat([target_rot[1], target_rot[2], target_rot[3], target_rot[0]])
        normal = r_target.apply([0, 0, 1]) # 로컬 +Z 벡터를 월드 벡터로 변환
        normal = normal / np.linalg.norm(normal) # 정규화

        # 2. 물리적 접촉력 측정 (순수 관측용)
        contact_reading = contact_sensor.get_current_frame()
        current_force = 0.0
        if contact_reading and "force" in contact_reading:
            current_force = np.linalg.norm(contact_reading["force"])

        # 3. 로봇의 목표 포즈 계산
        base_orientation = z_align_quat(-normal)
        target_orientation = np.array(base_orientation) # [w, x, y, z]
        r_target_ori = R.from_quat([target_orientation[1], target_orientation[2], target_orientation[3], target_orientation[0]])

        # 타겟판(FixedCuboid)의 두께가 0.02(2cm)이므로, 중심에서 1cm(0.01) 위가 실제 표면입니다.
        target_surface_pos = target_pos + normal * 0.01
        
        # 목표 회전 상태에서의 샌더 끝자락 오프셋 벡터 계산
        rotated_tool_offset = r_target_ori.apply(local_tool_offset)
        
        # 샌더 끝자락이 정가운데(target_surface_pos)에 오도록 손목(link_6)의 위치를 3D로 완벽히 역산!
        link_6_target_pos = target_surface_pos - rotated_tool_offset

        # 4. RMPFlow 제어
        actions = controller.forward(
            target_end_effector_position=link_6_target_pos,
            target_end_effector_orientation=target_orientation
        )
        robot_articulation.apply_action(actions)

        # 5. 샌더 패드 회전 모터 가동 (7번째 조인트)
        if robot_articulation.num_dof > 6:
            from omni.isaac.core.utils.types import ArticulationAction
            current_actions = robot_articulation.get_applied_action()
            joint_velocities = current_actions.joint_velocities
            if joint_velocities is None:
                joint_velocities = np.zeros(robot_articulation.num_dof)
            joint_velocities[-1] = 10.0
            pad_action = ArticulationAction(
                joint_velocities=joint_velocities,
                joint_indices=np.arange(robot_articulation.num_dof)
            )
            robot_articulation.apply_action(pad_action)

        step_count += 1
        if step_count % 30 == 0:
            print(f"[물리 피드백] 현재 순수 접촉력: {current_force:5.1f} N")
            with open(log_file_path, "a") as f:
                f.write(f"{step_count},{current_force}\n")
                
        # 매 스텝마다 ROS2로 접촉력 퍼블리시 (rqt_plot 용)
        if ROS2_AVAILABLE:
            msg = Float64()
            msg.data = float(current_force)
            force_pub.publish(msg)
            rclpy.spin_once(ros_node, timeout_sec=0.0)

if __name__ == "__main__":
    main()
    if 'ROS2_AVAILABLE' in globals() and ROS2_AVAILABLE:
        rclpy.shutdown()
    simulation_app.close()
