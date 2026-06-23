from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdGeom, Usd, Gf
import time
import math

OUTPUT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd"
stage = Usd.Stage.Open(OUTPUT_USD_PATH)

def find_prim_by_name(stage, prim_name):
    for prim in stage.TraverseAll():
        if prim.GetName() == prim_name:
            return prim
    return None

tool_prim = find_prim_by_name(stage, "sanding_kit")
xform = UsdGeom.Xformable(tool_prim)
xform.ClearXformOpOrder()
xform.SetResetXformStack(True)

# Try rotating X by 90, and translating along Z by 0.15
xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.15))
xform.AddRotateXYZOp().Set(Gf.Vec3d(90, 0, 0))
xform.AddScaleOp().Set(Gf.Vec3d(0.001, 0.001, 0.001))

stage.Save()
print("Saved with Z translation.")
simulation_app.close()
