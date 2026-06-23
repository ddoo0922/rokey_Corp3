from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdGeom, Usd, Gf, UsdPhysics, PhysxSchema
from isaacsim.core.api.world.world import World
from isaacsim.robot.manipulators.examples.universal_robots.ur10 import UR10
from isaacsim.core.api.articulations import Articulation
import numpy as np

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Add robot
ROBOT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd"
import omni.isaac.core.utils.prims as prim_utils
prim_utils.create_prim(
    prim_path="/World/M0609",
    prim_type="Xform",
    position=np.array([-0.45, 0.0, 0.2]),
    usd_path=ROBOT_USD_PATH
)

robot_articulation = Articulation(prim_path="/World/M0609/m0609/m0609", name="m0609_robot")
world.scene.add(robot_articulation)

world.reset()

print("DOF NAMES:", robot_articulation.dof_names)

simulation_app.close()
