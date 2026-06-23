from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom
stage = Usd.Stage.Open("/home/rokey/cobot4_ws/Downloads/OnRobot_Sander_v2.usd")
for prim in stage.TraverseAll():
    if prim.IsInstance():
        prim.SetInstanceable(False)

stage.GetRootLayer().Save()
app.close()
