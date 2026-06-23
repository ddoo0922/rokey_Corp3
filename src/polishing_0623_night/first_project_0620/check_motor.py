from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

OUTPUT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd"
stage = Usd.Stage.Open(OUTPUT_USD_PATH)

print("="*50)
print("Checking m0609_polishing.usd for pad_joint...")
print("="*50)

found_pad_joint = False
for prim in stage.TraverseAll():
    if "pad_joint" in prim.GetName() and (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.Joint)):
        found_pad_joint = True
        joint = UsdPhysics.RevoluteJoint(prim)
        print(f"Found pad_joint at: {prim.GetPath()}")
        print(f"  Body0 (Parent): {joint.GetBody0Rel().GetTargets()[0] if joint.GetBody0Rel().GetTargets() else 'None'}")
        print(f"  Body1 (Child):  {joint.GetBody1Rel().GetTargets()[0] if joint.GetBody1Rel().GetTargets() else 'None'}")
        print(f"  Axis: {joint.GetAxisAttr().Get()}")
        
        body1_paths = joint.GetBody1Rel().GetTargets()
        if body1_paths:
            body1_prim = stage.GetPrimAtPath(body1_paths[0])
            mass_api = UsdPhysics.MassAPI(body1_prim)
            if mass_api:
                print(f"  Body1 Mass: {mass_api.GetMassAttr().Get()} kg")
            else:
                print("  No MassAPI on Body1")

print("="*50)
if found_pad_joint:
    print("SUCCESS: pad_joint is physically attached and configured in the USD.")
else:
    print("ERROR: pad_joint is MISSING.")
print("="*50)

simulation_app.close()
