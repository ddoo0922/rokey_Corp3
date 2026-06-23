from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdGeom, Usd, Gf
import omni.isaac.core.utils.render as render_utils
from omni.isaac.core.utils.viewports import set_camera_view
import time

TOOL_USD_PATH = "/home/rokey/cobot4_ws/Downloads/OnRobot_Sander_v2.usd"
omni.usd.get_context().open_stage(TOOL_USD_PATH)
stage = omni.usd.get_context().get_stage()

# Add a dome light for lighting
from pxr import UsdLux
sphereLight = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight"))
sphereLight.CreateIntensityAttr(1000)

# We need to wait a bit for rendering to initialize
simulation_app.update()
for _ in range(10):
    simulation_app.update()

# Set camera to look at the tool
set_camera_view(eye=[0.3, 0.3, 0.3], target=[0.0, 0.0, 0.0], camera_prim_path="/OmniverseKit_Persp")

for _ in range(10):
    simulation_app.update()

# Render to file
output_path = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/tool_render.png"
# Isaac Sim's render API usually relies on Repicatior or simple capture.
import omni.kit.capture.viewport
from omni.kit.viewport.utility import get_active_viewport

viewport_window = get_active_viewport()
viewport_api = viewport_window.viewport_api

def on_capture_completed(success, file_path):
    print(f"Capture completed: {success} to {file_path}")

options = {
    "file_path": output_path,
    "width": 800,
    "height": 600,
    "overwrite": True
}
omni.kit.capture.viewport.capture_viewport(viewport_api, on_capture_completed, options)

# Wait for capture to complete
for _ in range(20):
    simulation_app.update()
    time.sleep(0.1)

simulation_app.close()
