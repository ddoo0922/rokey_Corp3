from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdGeom, Usd, Gf, UsdPhysics, PhysxSchema

OUTPUT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd"
stage = Usd.Stage.Open(OUTPUT_USD_PATH)

tool_path = "/World/m0609/link_6/sanding_kit"
pad_path = f"{tool_path}/tn__104327_"
pad_joint_path = f"{tool_path}/pad_joint"

pad_joint = UsdPhysics.RevoluteJoint.Get(stage, pad_joint_path)
if pad_joint:
    print(f"Pad joint axis: {pad_joint.GetAxisAttr().Get()}")
    print(f"Body0: {pad_joint.GetBody0Rel().GetTargets()}")
    print(f"Body1: {pad_joint.GetBody1Rel().GetTargets()}")
else:
    print("No pad joint found!")

simulation_app.close()
