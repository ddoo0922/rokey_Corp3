from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import os
from pxr import Usd, UsdGeom

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
stage = Usd.Stage.CreateNew(os.path.join(_SCRIPT_DIR, "debug.usd"))
UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

tool_prim = stage.DefinePrim("/World/sanding_kit")
tool_prim.GetReferences().AddReference(os.path.join(_SCRIPT_DIR, "OnRobot_Sander_v2.usd"))

print("Prims under sanding_kit:")
for prim in stage.TraverseAll():
    print(prim.GetPath())

simulation_app.close()
