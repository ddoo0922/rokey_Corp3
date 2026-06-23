from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdGeom, Usd, Gf
import numpy as np

ROBOT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/M0609/doosan-robot2/usd/m0609.usd"
stage = Usd.Stage.Open(ROBOT_USD_PATH)

link_6 = stage.GetPrimAtPath("/World/m0609/m0609/link_6")
if not link_6:
    # Try just /m0609/link_6
    link_6 = stage.GetPrimAtPath("/m0609/link_6")

if not link_6:
    print("link_6 not found anywhere!")
else:
    xform = UsdGeom.Xformable(link_6)
    time = Usd.TimeCode.Default()
    transform = xform.ComputeLocalToWorldTransform(time)
    
    origin = transform.Transform(Gf.Vec3d(0, 0, 0))
    x_axis = transform.Transform(Gf.Vec3d(1, 0, 0)) - origin
    y_axis = transform.Transform(Gf.Vec3d(0, 1, 0)) - origin
    z_axis = transform.Transform(Gf.Vec3d(0, 0, 1)) - origin
    
    print(f"Origin: {origin}")
    print(f"X axis: {x_axis}")
    print(f"Y axis: {y_axis}")
    print(f"Z axis: {z_axis}")

simulation_app.close()
