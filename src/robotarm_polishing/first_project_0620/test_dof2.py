from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
import numpy as np
import os

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_USD_PATH = os.path.join(_SCRIPT_DIR, "m0609_polishing.usd")

stage = Usd.Stage.Open(OUTPUT_USD_PATH)
# Change joint Body0 to link_6
joint_path = "/World/m0609/m0609/link_6/quick_mount/sanding_kit/pad_joint"
joint = UsdPhysics.RevoluteJoint.Get(stage, joint_path)
joint.CreateBody0Rel().SetTargets(["/World/m0609/m0609/link_6"])

# We must update localPos0 because it's now relative to link_6, not sanding_kit!
cache = UsdGeom.XformCache()
pad_prim = stage.GetPrimAtPath("/World/m0609/m0609/link_6/quick_mount/sanding_kit/OnRobot_Sander_v2/tn__104327_")
link6_prim = stage.GetPrimAtPath("/World/m0609/m0609/link_6")

pad_world = cache.GetLocalToWorldTransform(pad_prim)
link6_world = cache.GetLocalToWorldTransform(link6_prim)
pad_local_to_link6 = pad_world * link6_world.GetInverse()

pos = pad_local_to_link6.ExtractTranslation()
rot = pad_local_to_link6.ExtractRotationQuat()
joint.CreateLocalPos0Attr().Set(Gf.Vec3f(pos[0], pos[1], pos[2]))
joint.CreateLocalRot0Attr().Set(Gf.Quatf(rot.GetReal(), rot.GetImaginary()[0], rot.GetImaginary()[1], rot.GetImaginary()[2]))

stage.GetRootLayer().Save()

world = World()
from omni.isaac.core.utils.prims import create_prim
create_prim(
    prim_path="/World/M0609",
    prim_type="Xform",
    usd_path=OUTPUT_USD_PATH
)
robot = Articulation("/World/M0609/m0609/m0609", name="robot")
world.scene.add(robot)
world.reset()

print("DOFs:", robot.num_dof)
print("DOF Names:", robot.dof_names)

simulation_app.close()
