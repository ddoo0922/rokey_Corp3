from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, Gf

stage = Usd.Stage.Open("/home/rokey/cobot4_ws/Downloads/OnRobot_Sander_v2.usd")
pad_prim = stage.GetPrimAtPath("/OnRobot_Sander_v2/OnRobot_Sander_v2/tn__104327_")
bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default'])
bbox = bbox_cache.ComputeWorldBound(pad_prim).ComputeAlignedBox()

print(f"Min: {bbox.GetMin()}")
print(f"Max: {bbox.GetMax()}")
size = bbox.GetMax() - bbox.GetMin()
print(f"Size: {size}")

tool_prim = stage.GetPrimAtPath("/OnRobot_Sander_v2")
tool_bbox = bbox_cache.ComputeWorldBound(tool_prim).ComputeAlignedBox()
print(f"Tool Size: {tool_bbox.GetMax() - tool_bbox.GetMin()}")
print(f"Tool Min: {tool_bbox.GetMin()}")
print(f"Tool Max: {tool_bbox.GetMax()}")

simulation_app.close()
