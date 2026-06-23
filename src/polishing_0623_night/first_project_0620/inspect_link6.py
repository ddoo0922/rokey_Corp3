from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdGeom, Usd, Gf

ROBOT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/M0609/doosan-robot2/usd/m0609.usd"
stage = Usd.Stage.Open(ROBOT_USD_PATH)

link_6 = stage.GetPrimAtPath("/m0609/link_6")
if not link_6:
    print("link_6 not found")
else:
    xform = UsdGeom.Xformable(link_6)
    print(f"link_6 ops: {xform.GetXformOpOrderAttr().Get()}")

simulation_app.close()
