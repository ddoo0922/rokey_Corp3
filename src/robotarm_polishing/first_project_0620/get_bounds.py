from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, Gf
stage = Usd.Stage.Open("HEX_E_H_QC.usd")
bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default'])
bounds = bbox_cache.ComputeWorldBound(stage.GetPseudoRoot())
box = bounds.ComputeAlignedBox()
print(f"Mount Bounds: Min {box.GetMin()}, Max {box.GetMax()}")
size = box.GetMax() - box.GetMin()
print(f"Mount Size: {size}")
simulation_app.close()
