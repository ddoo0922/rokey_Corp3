from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, UsdPhysics, Gf, UsdShade
import os

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
stage = Usd.Stage.Open(os.path.join(_SCRIPT_DIR, "m0609_polishing.usd"))
pad_path = "/World/m0609/m0609/link_6/quick_mount/sanding_kit/OnRobot_Sander_v2/tn__104327_"
pad_prim = stage.GetPrimAtPath(pad_path)
print("Pad prim is valid:", pad_prim.IsValid())

if pad_prim.IsValid():
    try:
        UsdPhysics.RigidBodyAPI.Apply(pad_prim)
        mass_api = UsdPhysics.MassAPI.Apply(pad_prim)
        mass_api.CreateMassAttr(0.5)
        
        filtered_api = UsdPhysics.FilteredPairsAPI.Apply(pad_prim)
        filtered_api.GetFilteredPairsRel().AddTarget("/World/m0609/m0609/link_6")
        
        joint_path = "/World/m0609/m0609/link_6/quick_mount/sanding_kit/pad_joint"
        joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
        joint.CreateBody0Rel().SetTargets(["/World/m0609/m0609/link_6/quick_mount/sanding_kit"])
        joint.CreateBody1Rel().SetTargets([pad_path])
        joint.CreateAxisAttr("Y")
        print("Joint created:", joint.GetPrim().IsValid())
        stage.GetRootLayer().Save()
    except Exception as e:
        print("Error:", e)

simulation_app.close()
