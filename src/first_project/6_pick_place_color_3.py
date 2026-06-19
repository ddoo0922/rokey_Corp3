from omni.isaac.kit import SimulationApp

# 1. 시뮬레이션 환경 설정
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")

for _ in range(10):
    simulation_app.update()

import sys, os
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
from pxr import Usd, UsdGeom, UsdPhysics
import omni.usd, omni.graph.core as og, omni.replicator.core as rep, omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
import numpy as np

# ── 상수 및 경로 설정 ──────────────────────────────────────────
USD_PATH       = "/home/rokey/cobot3_ws/isaacpjt/M0609/Collected_m0609_camera/m0609_camera.usd"
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME   = "link_6"
GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]

PICK_ZONE    = np.array([0.5, 0.0, 0.025])
BLUE_PLACE   = np.array([0.4, 0.3, 0.025])
GREEN_PLACE  = np.array([0.4, -0.3, 0.025])
HIDDEN_POS   = np.array([0.0, 0.0, -5.0])

# ── 클래스 정의 ──────────────────────────────────────────────
class ColorSubscriberNode(Node):
    def __init__(self):
        super().__init__('isaac_pick_place_color_node')
        self.color_id = None
        self.create_subscription(Int32, '/color_id', self._cb, 10)
        self.get_logger().info("토픽 대기 중...")

    def _cb(self, msg):
        self.color_id = int(msg.data)
        self.get_logger().info(f"수신: color_id={self.color_id}")

class RobotController:
    def __init__(self, world):
        self.world = world
        self.robot = world.scene.get_object("m0609_robot")
        self.blue_cube = world.scene.get_object("blue_cube")
        self.green_cube = world.scene.get_object("green_cube")
        self.cube = None
        from m0609_pick_place_controller import PickPlaceController
        self.controller = PickPlaceController(
            name="pp_ctrl", gripper=self.robot.gripper, robot_articulation=self.robot,
            end_effector_initial_height=0.30, events_dt=[0.008, 0.005, 1.0, 0.1, 0.0025, 0.01, 0.0025, 1.0, 0.008, 0.08],
            urdf_path="/home/rokey/cobot3_ws/isaacpjt/M0609/doosan-robot2/urdf/m0609_isaac_sim.urdf",
            robot_description_path="/home/rokey/cobot3_ws/isaacpjt/M0609/rmpflow/m0609_description.yaml",
            rmpflow_config_path="/home/rokey/cobot3_ws/isaacpjt/M0609/rmpflow/m0609_rmpflow_common.yaml",
            end_effector_frame_name=EE_LINK_NAME,
        )

    def initialize(self):
        self.robot.initialize()
        self.robot.gripper.initialize(world.physics_sim_view, self.robot.apply_action, self.robot.get_joint_positions, self.robot.set_joint_positions, self.robot.dof_names)
        self.controller.reset()

    def spawn_random_cube(self):
        for c in [self.blue_cube, self.green_cube]: c.set_world_pose(position=HIDDEN_POS)
        choice = np.random.choice([1, 2])
        self.cube = self.blue_cube if choice == 1 else self.green_cube
        self.cube.set_world_pose(position=PICK_ZONE + np.array([0,0,0]))

# ── 메인 실행 ────────────────────────────────────────────────
def main():
    rclpy.init()
    world = World(stage_units_in_meters=1.0)
    # ... (생략된 태스크 등록 및 환경 구성 로직은 이전과 동일)
    
    ctrl = RobotController(world)
    ctrl.initialize()
    world.play()
    
    node = ColorSubscriberNode()
    ctrl.spawn_random_cube()
    state = 1
    
    while simulation_app.is_running():
        world.step(render=True)
        rclpy.spin_once(node, timeout_sec=0.001)

        if state == 1 and node.color_id is not None:
            target = BLUE_PLACE if node.color_id == 1 else GREEN_PLACE
            ctrl.controller.reset()
            state = 2

        elif state == 2 and ctrl.cube is not None:
            actions = ctrl.controller.forward(ctrl.cube.get_world_pose()[0], target, ctrl.robot.get_joint_positions())
            ctrl.robot.apply_action(actions)

            if ctrl.controller.is_done():
                print("[완료] 원점 복귀 중...")
                ctrl.robot.set_joint_positions(np.zeros(ctrl.robot.num_dof)) # 원점 복귀 추가
                world.step(render=True)
                node.color_id = None
                ctrl.spawn_random_cube()
                state = 1

    simulation_app.close()

if __name__ == '__main__':
    main()