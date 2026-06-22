import numpy as np
from scipy.spatial.transform import Rotation as R

def z_align_quat(target_z):
    # This is roughly what the sim does
    target_z = target_z / np.linalg.norm(target_z)
    up = np.array([0, 0, 1])
    if np.allclose(target_z, up):
        return [1, 0, 0, 0] # w, x, y, z
    if np.allclose(target_z, -up):
        return [0, 1, 0, 0]
    axis = np.cross(up, target_z)
    axis = axis / np.linalg.norm(axis)
    angle = np.arccos(np.dot(up, target_z))
    
    qx = axis[0] * np.sin(angle/2)
    qy = axis[1] * np.sin(angle/2)
    qz = axis[2] * np.sin(angle/2)
    qw = np.cos(angle/2)
    return [qw, qx, qy, qz]

target_pos = np.array([-0.45, 0.0, 0.05])
center_of_curvature = np.array([-0.05, 0.0, -0.2])
normal = target_pos - center_of_curvature
normal = normal / np.linalg.norm(normal)

print(f"Normal (points away from surface): {normal}")

base_orientation = z_align_quat(-normal)
base_scipy = [base_orientation[1], base_orientation[2], base_orientation[3], base_orientation[0]]
rot_base = R.from_quat(base_scipy)

rot_offset = R.from_euler('x', -90, degrees=True) * R.from_euler('z', 180, degrees=True)
rot_target = rot_base * rot_offset

print(f"Link 6 Z axis (Handle): {rot_target.apply([0, 0, 1])}")
print(f"Link 6 Y axis (Pad side): {rot_target.apply([0, 1, 0])}")
print(f"Link 6 -Y axis (Pad bottom): {rot_target.apply([0, -1, 0])}")

pad_local_pos = np.array([0.0, -0.05, 0.15])
world_pad_offset = rot_target.apply(pad_local_pos)

print(f"World Pad Offset (from Link 6): {world_pad_offset}")

CONTACT_OFFSET = 0.005
link_6_target_pos = target_pos - world_pad_offset + normal * CONTACT_OFFSET
print(f"Target Pos (Mouse): {target_pos}")
print(f"Link 6 Target Pos: {link_6_target_pos}")

# Compute pad's world pos if link 6 is at link_6_target_pos
computed_pad_pos = link_6_target_pos + world_pad_offset
print(f"Computed Pad Pos: {computed_pad_pos}")
print(f"Target Pos + normal*0.005: {target_pos + normal * 0.005}")

