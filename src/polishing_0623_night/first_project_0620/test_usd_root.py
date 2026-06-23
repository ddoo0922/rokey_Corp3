from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdGeom, Usd, Gf

TOOL_USD_PATH = "/home/rokey/cobot4_ws/Downloads/OnRobot_Sander_v2.usd"
stage = Usd.Stage.Open(TOOL_USD_PATH)
root = stage.GetDefaultPrim()
print(f"ROOT PRIM: {root.GetName()}")
xformable = UsdGeom.Xformable(root)
print(f"XformOpOrder: {xformable.GetXformOpOrderAttr().Get()}")
for op in xformable.GetOrderedXformOps():
    print(f"Op: {op.GetOpName()} = {op.Get()}")
simulation_app.close()
