from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
import numpy as np
import os

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_USD_PATH = os.path.join(_SCRIPT_DIR, "m0609_polishing.usd")

world = World()
stage = omni.usd.get_context().get_stage()

# Load the USD into the world
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
