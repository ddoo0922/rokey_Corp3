from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd, UsdPhysics

OUTPUT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd"
stage = Usd.Stage.Open(OUTPUT_USD_PATH)

for prim in stage.TraverseAll():
    if prim.GetName() == "pad_joint":
        print(f"Found pad_joint at: {prim.GetPath()}")
        joint = UsdPhysics.RevoluteJoint(prim)
        print(f"  Body0: {joint.GetBody0Rel().GetTargets()}")
        print(f"  Body1: {joint.GetBody1Rel().GetTargets()}")
        print(f"  Axis: {joint.GetAxisAttr().Get()}")
    elif prim.GetName() == "tn__104327_":
        print(f"Found pad at: {prim.GetPath()}")
        mass_api = UsdPhysics.MassAPI(prim)
        if mass_api:
            print(f"  Mass: {mass_api.GetMassAttr().Get()}")

simulation_app.close()
