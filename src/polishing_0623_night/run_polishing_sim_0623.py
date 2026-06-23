import sys
import os
import numpy as np
import math
from isaacsim import SimulationApp

# Headless 모드는 False로 하여 UI를 띄웁니다.
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import VisualSphere
from isaacsim.core.prims import SingleArticulation
from omni.isaac.core.utils.prims import create_prim

# RMPFlow 컨트롤러 경로 추가
sys.path.append("/home/rokey/rokey_Corp3/src/polishing_0623/rmpflow")
from m0609_rmpflow_controller import RMPFlowController

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--obj_name", type=str, default="car")
args, unknown = parser.parse_known_args()

BASE_DIR = "/home/rokey/rokey_Corp3/src/polishing_0623"
ROBOT_USD_PATH = os.path.join(BASE_DIR, "m0609_polishing.usd")
ROOM_USD_PATH = os.path.join(BASE_DIR, "room.usd")
TARGET_USD_PATH = os.path.join(BASE_DIR, "scan_obj", f"{args.obj_name}.usd")


def z_align_quat(z_vec):
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

def is_reachable(p, base_pos, max_radius=0.9):
    """로봇 베이스로부터의 거리가 max_radius 이내인지 확인하는 필터 함수"""
    dist = np.linalg.norm(np.array(p) - np.array(base_pos))
    return dist <= max_radius

def main():
    world = World(stage_units_in_meters=1.0)
    
    # 1. 룸 로드 (유저 커스텀 환경)
    create_prim(
        prim_path="/World/Room",
        prim_type="Xform",
        position=np.array([0.0, 0.0, 0.0]),
        usd_path=ROOM_USD_PATH
    )
    
    # 타겟 물체 로드
    create_prim(
        prim_path=f"/World/{args.obj_name.capitalize()}",
        prim_type="Xform",
        usd_path=TARGET_USD_PATH
    )
    
    # 2. 로봇 로드
    robot_base_pos = np.array([-0.8, -0.7, 0.2])
    create_prim(
        prim_path="/World/M0609",
        prim_type="Xform",
        position=robot_base_pos,
        usd_path=ROBOT_USD_PATH
    )
    
    robot_prim_path = "/World/M0609/m0609/m0609"
    robot_articulation = SingleArticulation(prim_path=robot_prim_path, name="m0609_robot")
    
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    
    # 3. 3D 경로 데이터 로드
    path_file = os.path.join(BASE_DIR, "scan_result", args.obj_name, "path.npy")
    if os.path.exists(path_file):
        points = np.load(path_file)
        print(f"[run_polishing] {len(points)} 개의 웨이포인트를 로드했습니다.")
    else:
        print(f"[run_polishing] 경로 파일을 찾을 수 없습니다: {path_file}")
        points = []

    world.reset()
    robot_articulation.initialize()
    
    controller = RMPFlowController(
        name="polishing_controller",
        robot_articulation=robot_articulation,
        urdf_path="/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/M0609/doosan-robot2/urdf/m0609_isaac_sim.urdf",
        end_effector_frame_name="link_6"
    )
    
    if len(points) > 0:
        min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
        min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])
        min_z = np.min(points[:, 2])
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        center_of_curvature = np.array([center_x, center_y, min_z - 0.3])
    else:
        print("입력된 경로가 없습니다. 대기 상태로 시뮬레이션을 유지합니다.")
        center_of_curvature = np.array([0, 0, 0])

    from pxr import UsdGeom, Vt, Gf
    future_path_prim = UsdGeom.BasisCurves.Define(stage, "/World/FuturePath")
    future_path_prim.CreateTypeAttr().Set(UsdGeom.Tokens.linear)
    future_path_prim.CreateWidthsAttr().Set([0.005])
    future_path_prim.CreateDisplayColorAttr().Set([(0.0, 1.0, 0.0)])
    
    from omni.isaac.core.utils.xforms import get_world_pose
    from omni.isaac.core.utils.prims import get_prim_at_path
    
    current_target_idx = 0
    
    while simulation_app.is_running():
        world.step(render=True)
        
        if world.is_playing() and current_target_idx < len(points):
            target_pos = points[current_target_idx]
            
            controller._motion_policy.set_robot_base_pose(
                robot_position=robot_base_pos,
                robot_orientation=controller._default_orientation
            )
            
            normal = target_pos - center_of_curvature
            normal = normal / np.linalg.norm(normal)
            
            base_orientation = z_align_quat(-normal)
            target_orientation = np.array(base_orientation)
            
            CONTACT_OFFSET = 0.005
            link_6_target_pos = target_pos + normal * (0.008 + CONTACT_OFFSET)
            
            actions = controller.forward(
                target_end_effector_position=link_6_target_pos,
                target_end_effector_orientation=target_orientation
            )
            robot_articulation.apply_action(actions)
            
            link_6_path = "/World/M0609/m0609/m0609/link_6"
            if get_prim_at_path(link_6_path):
                lookahead_pts = points[current_target_idx : current_target_idx + 500]
                if len(lookahead_pts) > 1:
                    vec3f_pts = [Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in lookahead_pts]
                    future_path_prim.GetPointsAttr().Set(Vt.Vec3fArray(vec3f_pts))
                    future_path_prim.GetCurveVertexCountsAttr().Set([len(lookahead_pts)])
                
                current_target_idx += 1
                
                try:
                    pad_prim = stage.GetPrimAtPath("/World/M0609/m0609/m0609/link_6/sanding_kit/OnRobot_Sander_v2/tn__104327_")
                    if not pad_prim.IsValid():
                        pad_prim = stage.GetPrimAtPath("/World/M0609/World/m0609/m0609/link_6/sanding_kit/OnRobot_Sander_v2/tn__104327_")
                    
                    if pad_prim.IsValid():
                        xform = UsdGeom.Xformable(pad_prim)
                        rot_op = None
                        for op in xform.GetOrderedXformOps():
                            if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                                rot_op = op
                                break
                        if not rot_op:
                            rot_op = xform.AddRotateXYZOp()
                        
                        current_rot = rot_op.Get()
                        if current_rot is None:
                            current_rot = Gf.Vec3d(0, 0, 0)
                        
                        if type(current_rot).__name__ == 'Vec3f':
                            rot_op.Set(current_rot + Gf.Vec3f(0.0, 20.0, 0.0))
                        else:
                            rot_op.Set(current_rot + Gf.Vec3d(0.0, 20.0, 0.0))
                except Exception as e:
                    pass

if __name__ == "__main__":
    main()
    simulation_app.close()
