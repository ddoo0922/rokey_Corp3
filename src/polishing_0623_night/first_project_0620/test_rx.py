from scipy.spatial.transform import Rotation as R
import numpy as np

# R_x(-90)
r = R.from_euler('x', -90, degrees=True)
print("R_x(-90) applied to +Y [0, 1, 0]:", r.apply([0, 1, 0]))
print("R_x(-90) applied to -Y [0, -1, 0]:", r.apply([0, -1, 0]))

# In Isaac Sim, xform.AddRotateXYZOp().Set(Gf.Vec3d(-90, 0, 0))
# Let's check how Isaac Sim computes rotation.
# It is an intrinsic rotation? No, just Euler angles XYZ.
