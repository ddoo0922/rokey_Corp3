from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni
import omni.replicator.core as rep
from pxr import UsdGeom
from isaacsim.core.api import World

world = World(stage_units_in_meters=1.0)
world.reset()

camera = rep.create.camera(
    position=(0.0, -0.75, 2.50),
    look_at=(0.0, -0.75, 0.0),
    focal_length=24.0,
    horizontal_aperture=20.955
)

# Step once to realize the replicator objects
for i in range(5):
    rep.orchestrator.step()
    app.update()

# Find the camera prim
stage = omni.usd.get_context().get_stage()
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Camera):
        xform = UsdGeom.Xformable(prim)
        transform = xform.ComputeLocalToWorldTransform(0)
        print(f"Camera Transform Matrix: {transform}")
        
        # Also let's print the actual rotation
        print(f"Position: {transform.ExtractTranslation()}")
        rot = transform.ExtractRotationMatrix()
        print(f"Rotation Matrix:\n{rot}")
        break

app.close()
