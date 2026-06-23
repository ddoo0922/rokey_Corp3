from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, UsdPhysics, Gf
import os

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_USD_PATH = os.path.join(_SCRIPT_DIR, "m0609_polishing.usd")

stage = Usd.Stage.Open(OUTPUT_USD_PATH)

# Let's see if we can move quick_mount to be a sibling of link_6
robot_path = "/World/m0609/m0609"
link_6_path = f"{robot_path}/link_6"

# Print the hierarchy to confirm
print("Children of m0609:")
for child in stage.GetPrimAtPath(robot_path).GetChildren():
    print(" -", child.GetName())

simulation_app.close()
