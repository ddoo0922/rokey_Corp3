from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdGeom, Usd, Gf
import numpy as np

OUTPUT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd"
stage = Usd.Stage.Open(OUTPUT_USD_PATH)

def find_prim_by_name(stage, prim_name):
    for prim in stage.TraverseAll():
        if prim.GetName() == prim_name:
            return prim
    return None

link_6 = find_prim_by_name(stage, "link_6")
if link_6:
    xform = UsdGeom.Xformable(link_6)
    time = Usd.TimeCode.Default()
    transform = xform.ComputeLocalToWorldTransform(time)
    
    origin = transform.Transform(Gf.Vec3d(0, 0, 0))
    x_axis = transform.Transform(Gf.Vec3d(1, 0, 0)) - origin
    y_axis = transform.Transform(Gf.Vec3d(0, 1, 0)) - origin
    z_axis = transform.Transform(Gf.Vec3d(0, 0, 1)) - origin
    
    print(f"LINK_6 Origin: {origin}")
    print(f"LINK_6 X axis: {x_axis}")
    print(f"LINK_6 Y axis: {y_axis}")
    print(f"LINK_6 Z axis: {z_axis}")
else:
    print("link_6 not found")

tool = find_prim_by_name(stage, "sanding_kit")
if tool:
    xform = UsdGeom.Xformable(tool)
    transform = xform.ComputeLocalToWorldTransform(time)
    origin = transform.Transform(Gf.Vec3d(0, 0, 0))
    x_axis = transform.Transform(Gf.Vec3d(1, 0, 0)) - origin
    y_axis = transform.Transform(Gf.Vec3d(0, 1, 0)) - origin
    z_axis = transform.Transform(Gf.Vec3d(0, 0, 1)) - origin
    print(f"TOOL Origin: {origin}")
    print(f"TOOL X axis: {x_axis}")
    print(f"TOOL Y axis: {y_axis}")
    print(f"TOOL Z axis: {z_axis}")

simulation_app.close()
