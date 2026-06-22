from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdGeom, Usd, Gf, UsdPhysics, PhysxSchema

OUTPUT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd"
stage = Usd.Stage.Open(OUTPUT_USD_PATH)

tool_path = "/World/m0609/link_6/sanding_kit"
pad_path = f"{tool_path}/tn__104327_"

# Check if pad_joint exists
pad_joint_path = f"{tool_path}/pad_joint"
if stage.GetPrimAtPath(pad_joint_path):
    print("pad_joint already exists!")
else:
    # Add RigidBodyAPI to pad
    pad_prim = stage.GetPrimAtPath(pad_path)
    if pad_prim:
        physics_api = UsdPhysics.RigidBodyAPI.Apply(pad_prim)
        PhysxSchema.PhysxRigidBodyAPI.Apply(pad_prim)
        
        # We don't apply RigidBodyAPI to tool_prim because it's a child of link_6 (which is in the articulation)
        
        pad_joint = UsdPhysics.RevoluteJoint.Define(stage, pad_joint_path)
        pad_joint.CreateBody0Rel().SetTargets([tool_path]) # parent is tool (kinematic)
        pad_joint.CreateBody1Rel().SetTargets([pad_path])  # child is pad (dynamic)
        pad_joint.CreateAxisAttr("Z")
        
        # Add drive
        drive = UsdPhysics.DriveAPI.Apply(pad_joint.GetPrim(), "angular")
        drive.CreateTypeAttr("velocity")
        drive.CreateDampingAttr(1e4)
        drive.CreateStiffnessAttr(0)
        
        print("Created pad_joint successfully")
        stage.GetRootLayer().Save()

simulation_app.close()
