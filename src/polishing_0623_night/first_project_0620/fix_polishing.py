import os
assemble_file = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/assemble_robot.py"
run_file = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/run_polishing_sim.py"

with open(assemble_file, "r") as f:
    code = f.read()

# Change pad_joint Body0 back to tool_path so it doesn't violently snap
code = code.replace(
    'pad_joint.CreateBody0Rel().SetTargets([link_6_prim.GetPath()])',
    'pad_joint.CreateBody0Rel().SetTargets([tool_path])'
)
with open(assemble_file, "w") as f:
    f.write(code)

with open(run_file, "r") as f:
    run_code = f.read()

# Fix target orientation (add 90 degree pitch/roll so pad faces the object)
import re

ori_code_old = """            # 툴이 일자형으로 똑바로 부착되어 있으므로, 90도 오프셋 없이 기본 자세를 유지합니다.
            base_orientation = z_align_quat(-normal) # [w, x, y, z] 형태
            
            target_orientation = base_orientation"""

ori_code_new = """            # 툴의 패드가 옆면(L자 형태)에 있으므로, Z축 대신 옆면이 바닥을 향하도록 90도 회전 오프셋을 줍니다.
            base_orientation = z_align_quat(-normal) # [w, x, y, z] 형태
            
            from scipy.spatial.transform import Rotation as R
            rot_offset = R.from_euler('x', 90, degrees=True).as_quat() # [x, y, z, w]
            # scipy는 [x,y,z,w] 이고 Isaac Sim은 [w,x,y,z] 이므로 변환
            offset_quat = np.array([rot_offset[3], rot_offset[0], rot_offset[1], rot_offset[2]])
            
            from omni.isaac.core.utils.rotations import quat_mul
            target_orientation = quat_mul(base_orientation, offset_quat)"""

run_code = run_code.replace(ori_code_old, ori_code_new)

# Fix tool offset
offset_old = """            CONTACT_OFFSET = 0.05
            link_6_target_pos = target_pos + normal * (TOOL_OFFSET + CONTACT_OFFSET)"""

offset_new = """            # 패드가 옆면에 있으므로 TOOL_OFFSET 방향을 보정해야 합니다.
            # 정상 방향 벡터(normal)를 따라 로봇 팔 끝단 위치를 계산합니다.
            CONTACT_OFFSET = 0.05
            link_6_target_pos = target_pos + normal * (0.15 + CONTACT_OFFSET)"""

run_code = run_code.replace(offset_old, offset_new)

# Fix pad speed error (use set_prim_attribute instead of Articulation)
speed_old = """                    try:
                        dof_index = robot_articulation.get_dof_index("pad_joint")
                        robot_articulation.set_joint_velocities([100.0], joint_indices=[dof_index])
                    except Exception as e:
                        print(f"Tracking point {current_target_idx}/{len(points)} | Pad Speed Error: {e}")"""

speed_new = """                    try:
                        from omni.isaac.core.utils.prims import set_prim_attribute
                        pad_joint_path = "/World/M0609/m0609/m0609/link_6/sanding_kit/pad_joint"
                        set_prim_attribute(pad_joint_path, "drive:angular:physics:targetVelocity", 1000.0)
                    except Exception as e:
                        print(f"Tracking point {current_target_idx}/{len(points)} | Pad Speed Error: {e}")"""

run_code = run_code.replace(speed_old, speed_new)

with open(run_file, "w") as f:
    f.write(run_code)
print("Files fixed.")
