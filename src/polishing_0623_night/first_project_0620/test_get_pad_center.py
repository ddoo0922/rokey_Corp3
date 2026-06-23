from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, Gf

stage = Usd.Stage.Open("/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd")

link_6 = stage.GetPrimAtPath("/World/m0609/link_6")
pad = stage.GetPrimAtPath("/World/m0609/link_6/sanding_kit/OnRobot_Sander_v2/tn__104327_")

bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default'])

link_6_bbox = bbox_cache.ComputeWorldBound(link_6).ComputeAlignedRange()
pad_bbox = bbox_cache.ComputeWorldBound(pad).ComputeAlignedRange()

print(f"Link 6 Center: {link_6_bbox.GetMidpoint()}")
print(f"Link 6 Min: {link_6_bbox.GetMin()}")
print(f"Link 6 Max: {link_6_bbox.GetMax()}")

print(f"Pad Center: {pad_bbox.GetMidpoint()}")
print(f"Pad Min: {pad_bbox.GetMin()}")
print(f"Pad Max: {pad_bbox.GetMax()}")

# Calculate offset
pad_center = pad_bbox.GetMidpoint()
link_6_center = link_6_bbox.GetMidpoint()
offset = pad_center - link_6_center
print(f"Offset (Pad - Link6): {offset}")

simulation_app.close()
