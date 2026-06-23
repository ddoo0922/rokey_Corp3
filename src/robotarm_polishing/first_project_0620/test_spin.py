from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf
import os

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_USD_PATH = os.path.join(_SCRIPT_DIR, "m0609_polishing.usd")

stage = Usd.Stage.Open(OUTPUT_USD_PATH)
pad_path = "/World/m0609/m0609/link_6/quick_mount/sanding_kit/OnRobot_Sander_v2/tn__104327_"
pad_prim = stage.GetPrimAtPath(pad_path)

if pad_prim.IsValid():
    joint_path = "/World/m0609/m0609/link_6/quick_mount/sanding_kit/pad_joint"
    joint_prim = stage.GetPrimAtPath(joint_path)
    drive_api = UsdPhysics.DriveAPI(joint_prim, "angular")
    print(f"Drive Target Velocity: {drive_api.GetTargetVelocityAttr().Get()}")
    print(f"Drive Damping: {drive_api.GetDampingAttr().Get()}")
    print(f"Drive Stiffness: {drive_api.GetStiffnessAttr().Get()}")
else:
    print("Pad not found")

simulation_app.close()
