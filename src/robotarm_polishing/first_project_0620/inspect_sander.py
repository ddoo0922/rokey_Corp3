from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd, UsdPhysics
import os

stage = Usd.Stage.Open(os.path.abspath("OnRobot_Sander_v2.usd"))
print("Prims in OnRobot_Sander_v2.usd:")
for prim in stage.TraverseAll():
    print(prim.GetPath())
simulation_app.close()
