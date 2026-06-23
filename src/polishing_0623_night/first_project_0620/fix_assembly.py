from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdGeom, Usd, Gf
import os

ROBOT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/M0609/doosan-robot2/usd/m0609.usd"
TOOL_USD_PATH = "/home/rokey/cobot4_ws/Downloads/OnRobot_Sander_v2.usd"
OUTPUT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd"

stage = Usd.Stage.CreateNew("temp.usd")
root_path = "/World"
UsdGeom.Xform.Define(stage, root_path)
robot_path = f"{root_path}/m0609"
robot_prim = stage.DefinePrim(robot_path)
robot_prim.GetReferences().AddReference(ROBOT_USD_PATH)

def find_prim_by_name(stage, prim_name):
    for prim in stage.TraverseAll():
        if prim.GetName() == prim_name:
            return prim
    return None

link_6_prim = find_prim_by_name(stage, "link_6")
tool_path = f"{link_6_prim.GetPath()}/sanding_kit"
tool_prim = stage.DefinePrim(tool_path)
tool_prim.GetReferences().AddReference(TOOL_USD_PATH)

xform = UsdGeom.Xformable(tool_prim)
xform.ClearXformOpOrder()

# Let's try to match the user's image.
# The flange outward normal is +Z or -Z? Let's assume +Z.
# The tool's connector is at Y=0 (or Z=0?), and pad is at Y=something.
# I will use translation (0,0,0) and rotate X by 90.
xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
# Let's test a few rotations to see which one works without the joint breaking it
xform.AddRotateXYZOp().Set(Gf.Vec3d(90, 0, 0))
xform.AddScaleOp().Set(Gf.Vec3d(0.001, 0.001, 0.001))

if os.path.exists(OUTPUT_USD_PATH):
    os.remove(OUTPUT_USD_PATH)
stage.GetRootLayer().Export(OUTPUT_USD_PATH)
print("Saved fixed assembly.")
simulation_app.close()
