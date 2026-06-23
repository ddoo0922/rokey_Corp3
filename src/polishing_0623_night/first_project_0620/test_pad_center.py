from scipy.spatial.transform import Rotation as R
import numpy as np

# Simulate z_align_quat
def z_align_quat(target_z):
    target_z = target_z / np.linalg.norm(target_z)
    up = np.array([0, 0, 1])
    axis = np.cross(up, target_z)
    axis = axis / np.linalg.norm(axis)
    angle = np.arccos(np.dot(up, target_z))
    return R.from_rotvec(axis * angle)

normal = np.array([-0.847, 0.0, 0.529]) # Example normal
target_pos = np.array([-0.45, 0.0, 0.05])

rot_link_6 = z_align_quat(-normal)

pad_center_local = np.array([-0.0025, 0.0, -0.0037])
pad_offset_world = rot_link_6.apply(pad_center_local)

link_6_target = target_pos - pad_offset_world

print(f"To make pad touch target_pos, link_6 must be at:")
print(f"{link_6_target}")

# How much is it "above" the target surface along normal?
# The distance from link_6 to target_pos along normal:
dist_along_normal = np.dot(link_6_target - target_pos, normal)
print(f"Link 6 distance from target along normal: {dist_along_normal}")
