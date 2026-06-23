from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd
stage = Usd.Stage.Open("/home/rokey/cobot4_ws/Downloads/OnRobot_Sander_v2.usd")
for prim in stage.TraverseAll():
    print(prim.GetPath())
simulation_app.close()
