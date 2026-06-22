from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import Usd, UsdGeom, Gf

stage = Usd.Stage.Open("/home/rokey/cobot4_ws/Downloads/OnRobot_Sander_v2.usd")

pad = stage.GetPrimAtPath("/OnRobot_Sander_v2/OnRobot_Sander_v2/tn__104327_")
pad_xform = UsdGeom.Xformable(pad).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

pad_pos = pad_xform.ExtractTranslation()
pad_z = pad_xform.TransformDir(Gf.Vec3d(0, 0, 1))
pad_y = pad_xform.TransformDir(Gf.Vec3d(0, 1, 0))
pad_x = pad_xform.TransformDir(Gf.Vec3d(1, 0, 0))

print("Raw Pad position:", pad_pos)
print("Raw Pad Z axis:", pad_z)
print("Raw Pad Y axis:", pad_y)
print("Raw Pad X axis:", pad_x)

simulation_app.close()
