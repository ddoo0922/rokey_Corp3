from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, UsdPhysics
import omni.physx.scripts.physicsUtils as physicsUtils
import inspect
print(inspect.signature(physicsUtils.add_joint))
simulation_app.close()
