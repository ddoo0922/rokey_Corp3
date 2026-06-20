# ── 1. SimulationApp (반드시 최상단) ──────────────────────────
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

# ── 2. 시스템 경로 설정 ──────────────────────────────────────
import sys, os

sys.path = [p for p in sys.path if "opt/ros" not in p and "local/lib/python3.10" not in p]

isaac_sim_path = os.environ.get("ISAAC_SIM_PATH", os.path.expanduser("~/dev_ws/isaac_sim/isaacsim"))
rclpy_path = os.path.join(isaac_sim_path, "_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/humble/rclpy")
if os.path.exists(rclpy_path):
    sys.path.append(rclpy_path)

rmpflow_dir = os.path.expanduser("~/cobot3_ws/isaacpjt/M0609/rmpflow")
if rmpflow_dir not in sys.path:
    sys.path.insert(0, rmpflow_dir)

# ── 3. Import ────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleGeometryPrim
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from pxr import Usd, UsdGeom, UsdPhysics, Gf
import omni.usd
import numpy as np

# ── 4. 상수 ──────────────────────────────────────────────────
USD_PATH       = "/home/rokey/cobot3_ws/isaacpjt/M0609/Collected_m0609_camera/m0609_camera.usd"
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME   = "link_6"
GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

GRIPPER_OPEN  = [0.0, 0.0]
GRIPPER_CLOSE = [0.5, 0.5]
GRIPPER_DELTA = [-0.5, -0.5]

FINGER_STATIC  = 1.8
FINGER_DYNAMIC = 1.4
CUBE_STATIC    = 1.2
CUBE_DYNAMIC   = 1.0

M0609_URDF_PATH          = "/home/rokey/cobot3_ws/isaacpjt/M0609/doosan-robot2/urdf/m0609_isaac_sim.urdf"
M0609_DESCRIPTION_PATH   = "/home/rokey/cobot3_ws/isaacpjt/M0609/rmpflow/m0609_description.yaml"
M0609_RMPFLOW_CONFIG_PATH = "/home/rokey/cobot3_ws/isaacpjt/M0609/rmpflow/m0609_rmpflow_common.yaml"

EVENTS_DT = [
    0.008,   # 0. 접근 이동
    0.005,   # 1. 하강
    1.0,     # 2. 그리퍼 닫기 대기
    0.1,     # 3. 그리퍼 닫힘 유지
    0.0025,  # 4. 들어올리기
    0.01,    # 5. Place 위치로 이동
    0.0025,  # 6. 하강
    1.0,     # 7. 그리퍼 열기 대기
    0.008,   # 8. 상승
    0.08,    # 9. 복귀
]

# Pick 영역 & Place 마커 좌표
PICK_ZONE    = np.array([0.5, 0.0, 0.025])
BLUE_PLACE   = np.array([0.4, 0.3, 0.025])
GREEN_PLACE  = np.array([0.4, -0.3, 0.025])
HIDDEN_POS   = np.array([0.0, 0.0, -5.0])   # 바닥 아래 숨김 위치


# ── 5. 유틸 함수 ─────────────────────────────────────────────
def find_prim_path_by_name(root_path, name):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def initialize_robot(robot, world):
    robot.initialize()
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    robot.set_joint_positions(np.zeros(robot.num_dof))


# ── 6. Task ──────────────────────────────────────────────────
class M0609PickPlaceColorTask(BaseTask):
    def __init__(self, name):
        super().__init__(name=name, offset=None)

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        stage = omni.usd.get_context().get_stage()

        # ── 1) USD 로드 ──
        print("\n" + "=" * 60)
        print("[1] USD 로드")
        print("=" * 60)
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()
        print(f"  [OK] {USD_PATH}")

        # USD 안의 불필요한 red_block 제거
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            if prim.GetName() == "red_block":
                stage.RemovePrim(prim.GetPath())
                print(f"  [삭제] red_block 제거: {prim.GetPath()}")
                break

        # ── 2) 링크 탐색 ──
        print("\n[2] 링크 탐색")
        ee_path = find_prim_path_by_name(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found")
        self._ee_path = ee_path
        print(f"  EE = {ee_path}")

        # ── 3) 물리 드라이브 설정 ──
        print("\n[3] 물리 드라이브 설정")
        cnt = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            for dt in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, dt)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                    cnt += 1
        print(f"  [OK] drive updated: {cnt}")

        # ── 4) 로봇 등록 ──
        print("\n[4] 로봇 등록")
        gripper = ParallelGripper(
            end_effector_prim_path=self._ee_path,
            joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=np.array(GRIPPER_OPEN),
            joint_closed_positions=np.array(GRIPPER_CLOSE),
            action_deltas=np.array(GRIPPER_DELTA),
        )
        scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM_PATH,
                name="m0609_robot",
                end_effector_prim_path=self._ee_path,
                gripper=gripper,
            )
        )
        print(f"  [OK] SingleManipulator 등록")

        # ── 5) 큐브 & 마커 생성 ──
        print("\n[5] 큐브 & Place 마커 생성")
        cube_mat = PhysicsMaterial(
            prim_path="/World/Physics_Materials/cube_material",
            static_friction=CUBE_STATIC,
            dynamic_friction=CUBE_DYNAMIC,
            restitution=0.0,
        )
        # 파란 큐브 (숨김)
        scene.add(DynamicCuboid(
            prim_path="/World/blue_cube", name="blue_cube",
            position=HIDDEN_POS, scale=np.array([0.05, 0.05, 0.05]),
            color=np.array([0.0, 0.0, 1.0]), mass=0.05,
            physics_material=cube_mat,
        ))
        # 초록 큐브 (숨김)
        scene.add(DynamicCuboid(
            prim_path="/World/green_cube", name="green_cube",
            position=HIDDEN_POS, scale=np.array([0.05, 0.05, 0.05]),
            color=np.array([0.0, 1.0, 0.0]), mass=0.05,
            physics_material=cube_mat,
        ))
        print("  [OK] blue/green 큐브 생성 (숨김)")

        # 파란 Place 마커 (납작한 판)
        scene.add(DynamicCuboid(
            prim_path="/World/blue_place", name="blue_place",
            position=np.array([BLUE_PLACE[0], BLUE_PLACE[1], 0.001]),
            scale=np.array([0.08, 0.08, 0.002]),
            color=np.array([0.0, 0.0, 1.0]), mass=100.0,
        ))
        # 초록 Place 마커
        scene.add(DynamicCuboid(
            prim_path="/World/green_place", name="green_place",
            position=np.array([GREEN_PLACE[0], GREEN_PLACE[1], 0.001]),
            scale=np.array([0.08, 0.08, 0.002]),
            color=np.array([0.0, 1.0, 0.0]), mass=100.0,
        ))
        print("  [OK] blue/green Place 마커 생성")

        # ── 6) 손가락 마찰 ──
        print("\n[6] 손가락 마찰 설정")
        finger_mat = PhysicsMaterial(
            prim_path="/World/Physics_Materials/finger_material",
            static_friction=FINGER_STATIC,
            dynamic_friction=FINGER_DYNAMIC,
            restitution=0.0,
        )
        for link_name in ["left_inner_finger", "right_inner_finger"]:
            lp = find_prim_path_by_name(ROBOT_PRIM_PATH, link_name)
            if lp:
                SingleGeometryPrim(prim_path=lp, name=f"{link_name}_geom").apply_physics_material(finger_mat)
                print(f"  [OK] friction: {lp}")

        print("\n  ===== 씬 구성 완료 =====\n")


# ── 7. ROS2 Node ─────────────────────────────────────────────
class ColorSubscriberNode(Node):
    def __init__(self):
        super().__init__('isaac_pick_place_color_node')
        self.color_id = None
        self.create_subscription(Int32, '/color_id', self._cb, 10)
        self.get_logger().info("토픽 대기 중... ros2 topic pub --once /color_id std_msgs/msg/Int32 \"{data: 1}\"")

    def _cb(self, msg):
        self.color_id = msg.data
        self.get_logger().info(f"수신: color_id={self.color_id} (1=파랑, 2=초록)")


# ── 8. RobotController ───────────────────────────────────────
class RobotController:
    def __init__(self, world):
        self.world = world
        self.robot      = world.scene.get_object("m0609_robot")
        self.blue_cube  = world.scene.get_object("blue_cube")
        self.green_cube = world.scene.get_object("green_cube")
        self.cube = None  # 현재 Pick 대상

        from m0609_pick_place_controller import PickPlaceController
        self.controller = PickPlaceController(
            name="pp_ctrl",
            gripper=self.robot.gripper,
            robot_articulation=self.robot,
            end_effector_initial_height=0.30,
            events_dt=EVENTS_DT,
            urdf_path=M0609_URDF_PATH,
            robot_description_path=M0609_DESCRIPTION_PATH,
            rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
            end_effector_frame_name=EE_LINK_NAME,
        )

    def initialize(self):
        initialize_robot(self.robot, self.world)
        self.controller.reset()

    def hide_cubes(self):
        """두 큐브 모두 바닥 아래로 숨김"""
        for c in [self.blue_cube, self.green_cube]:
            c.set_world_pose(position=HIDDEN_POS)
            c.set_linear_velocity(np.zeros(3))
            c.set_angular_velocity(np.zeros(3))

    def spawn_random_cube(self):
        """큐브 하나를 랜덤으로 골라 Pick 영역에 배치"""
        self.hide_cubes()
        choice = np.random.choice([1, 2])
        pos = PICK_ZONE + np.array([np.random.uniform(-0.02, 0.02),
                                     np.random.uniform(-0.02, 0.02), 0.0])
        if choice == 1:
            self.cube = self.blue_cube
            name = "파란"
        else:
            self.cube = self.green_cube
            name = "초록"

        self.cube.set_world_pose(position=pos)
        self.cube.set_linear_velocity(np.zeros(3))
        self.cube.set_angular_velocity(np.zeros(3))
        print(f"\n[Spawn] {name} 큐브 → Pick 영역 {pos}")


# ── 9. Main ──────────────────────────────────────────────────
def main():
    rclpy.init()

    # 월드 & 태스크
    world = World(stage_units_in_meters=1.0)
    world.add_task(M0609PickPlaceColorTask(name="pp_task"))
    world.reset()

    # 컨트롤러
    ctrl = RobotController(world)
    ctrl.initialize()

    # 시뮬레이션 시작
    world.play()
    world.step(render=True)

    # 안정화
    for _ in range(30):
        world.step(render=True)

    # ROS2 노드 & 첫 큐브
    node = ColorSubscriberNode()
    ctrl.spawn_random_cube()

    state = 1        # 1=대기, 2=동작중
    target = None
    print("\n▶ 준비 완료! 토픽을 쏴주세요:")
    print("  파랑: ros2 topic pub --once /color_id std_msgs/msg/Int32 \"{data: 1}\"")
    print("  초록: ros2 topic pub --once /color_id std_msgs/msg/Int32 \"{data: 2}\"")

    while simulation_app.is_running():
        world.step(render=True)
        rclpy.spin_once(node, timeout_sec=0.001)

        if not world.is_playing():
            continue

        # ── state 1: 토픽 대기 ──
        if state == 1 and node.color_id is not None:
            if node.color_id == 1:
                target = BLUE_PLACE
            elif node.color_id == 2:
                target = GREEN_PLACE
            else:
                node.color_id = None
                continue

            print(f"[Pick&Place] color_id={node.color_id} → Place 위치: {target}")
            ctrl.controller.reset()
            state = 2

        # ── state 2: Pick & Place 실행 ──
        elif state == 2 and ctrl.cube is not None:
            cube_pos, _ = ctrl.cube.get_world_pose()
            joints = ctrl.robot.get_joint_positions()

            actions = ctrl.controller.forward(
                picking_position=cube_pos,
                placing_position=target,
                current_joint_positions=joints,
                end_effector_offset=np.array([0.0, 0.0, 0.2]),
            )
            ctrl.robot.apply_action(actions)

            # 이벤트 진행 로그
            ev = ctrl.controller.get_current_event()
            if not hasattr(ctrl, "_ev") or ctrl._ev != ev:
                ctrl._ev = ev
                ee, _ = ctrl.robot.end_effector.get_world_pose()
                print(f"  Event {ev}  cube_z={cube_pos[2]:.3f}  ee_z={ee[2]:.3f}")

            if ctrl.controller.is_done():
                print("[완료] Pick & Place 성공!\n")
                node.color_id = None
                if hasattr(ctrl, "_ev"):
                    delattr(ctrl, "_ev")
                ctrl.spawn_random_cube()
                state = 1

    node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == '__main__':
    main()