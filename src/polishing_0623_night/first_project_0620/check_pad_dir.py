from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import Usd, UsdGeom, Gf

stage = Usd.Stage.Open("/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd")

link_6 = stage.GetPrimAtPath("/World/m0609/m0609/link_6")
pad = stage.GetPrimAtPath("/World/m0609/m0609/link_6/sanding_kit/OnRobot_Sander_v2/tn__104327_")

link_6_xform = UsdGeom.Xformable(link_6).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
pad_xform = UsdGeom.Xformable(pad).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

# transform from pad to link_6
# We want to see the direction of the pad's Z axis (if the pad is a flat disc, Z is usually its normal)
# Let's see the world normal of the pad.
pad_z_world = pad_xform.TransformDir(Gf.Vec3d(0, 0, 1))
pad_y_world = pad_xform.TransformDir(Gf.Vec3d(0, 1, 0))
pad_x_world = pad_xform.TransformDir(Gf.Vec3d(1, 0, 0))

link_6_inv = link_6_xform.GetInverse()

pad_z_local = link_6_inv.TransformDir(pad_z_world)
pad_y_local = link_6_inv.TransformDir(pad_y_world)
pad_x_local = link_6_inv.TransformDir(pad_x_world)

print("Pad Z axis in link_6 frame:", pad_z_local)
print("Pad Y axis in link_6 frame:", pad_y_local)
print("Pad X axis in link_6 frame:", pad_x_local)

# Let's also check the position of the pad relative to link_6
pad_pos_world = pad_xform.ExtractTranslation()
pad_pos_local = link_6_inv.Transform(pad_pos_world)
print("Pad position in link_6 frame:", pad_pos_local)

simulation_app.close()
