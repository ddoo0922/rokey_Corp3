from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import create_prim

world = World(stage_units_in_meters=1.0)
create_prim(
    prim_path="/World/M0609",
    prim_type="Xform",
    usd_path="/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd"
)

robot = Articulation(prim_path="/World/M0609/m0609/m0609", name="robot")
world.scene.add(robot)

world.reset()

print("\n" + "="*50)
print(f"Number of DOFs: {robot.num_dof}")
print(f"Joint Names: {robot.dof_names}")

# Let's run a few steps and print joint velocities
for i in range(10):
    world.step()

vel = robot.get_joint_velocities()
print(f"Joint Velocities after 10 steps: {vel}")

# Try to apply an action to the 7th joint if it exists
if robot.num_dof > 6:
    print(f"Applying velocity to {robot.dof_names[6]}")
    from omni.isaac.core.utils.types import ArticulationAction
    action = ArticulationAction(joint_velocities=np.array([0,0,0,0,0,0,500.0]))
    robot.apply_action(action)
    for i in range(50):
        world.step()
    vel2 = robot.get_joint_velocities()
    print(f"Joint Velocities after action: {vel2}")

print("="*50 + "\n")

app.close()
