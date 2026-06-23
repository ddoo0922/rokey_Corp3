from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom
import sys

def check_usd():
    stage = Usd.Stage.Open('/home/rokey/cobot4_ws/Downloads/OnRobot_Sander_v2.usd')
    if not stage:
        print("Failed to open")
        sys.exit(1)
    
    def print_hierarchy(prim, level=0):
        if level > 3: return
        print("  " * level + prim.GetName())
        for child in prim.GetChildren():
            print_hierarchy(child, level + 1)
            
    print_hierarchy(stage.GetPseudoRoot())

if __name__ == "__main__":
    check_usd()
