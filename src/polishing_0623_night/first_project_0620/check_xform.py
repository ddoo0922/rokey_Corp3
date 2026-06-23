from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom
stage = Usd.Stage.Open('m0609_polishing.usd')
pad = stage.GetPrimAtPath('/World/m0609/m0609/link_6/sanding_kit/OnRobot_Sander_v2/tn__104327_')
print("================================")
print("Is Valid:", pad.IsValid())
if pad.IsValid():
    print("XformOpOrder:", pad.GetAttribute('xformOpOrder').Get())
print("================================")
simulation_app.close()
