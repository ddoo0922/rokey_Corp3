from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdGeom, Usd, Gf
import os

ROBOT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/M0609/doosan-robot2/usd/m0609.usd"
TOOL_USD_PATH = "/home/rokey/cobot4_ws/Downloads/OnRobot_Sander_v2.usd"

stage = Usd.Stage.CreateNew("temp_test.usd")
tool_prim = stage.DefinePrim("/tool")
tool_prim.GetReferences().AddReference(TOOL_USD_PATH)

bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default'])
bbox = bbox_cache.ComputeLocalBound(tool_prim)
r = bbox.GetRange()
print(f"TOOL LOCAL BBOX: Min {r.GetMin()}, Max {r.GetMax()}")
print(f"TOOL CENTER: {r.GetMidpoint()}")

simulation_app.close()
