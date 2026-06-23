from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdGeom, Usd, Gf

TOOL_USD_PATH = "/home/rokey/cobot4_ws/Downloads/OnRobot_Sander_v2.usd"
stage = Usd.Stage.Open(TOOL_USD_PATH)

bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

print("--- BOUNDING BOXES ---")
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Xformable):
        bbox = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
        print(f"Prim: {prim.GetName()}, Min: {bbox.GetMin()}, Max: {bbox.GetMax()}, Center: {bbox.GetMidpoint()}")

simulation_app.close()
