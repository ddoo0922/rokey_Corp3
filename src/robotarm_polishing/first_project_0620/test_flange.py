from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdGeom, Usd, Gf
import numpy as np

OUTPUT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd"
stage = Usd.Stage.Open(OUTPUT_USD_PATH)

def find_prim_by_name(stage, prim_name):
    for prim in stage.TraverseAll():
        if prim.GetName() == prim_name:
            return prim
    return None

link_6 = find_prim_by_name(stage, "link_6")
xform = UsdGeom.Xformable(link_6)
time = Usd.TimeCode.Default()
transform = xform.ComputeLocalToWorldTransform(time)
origin = transform.Transform(Gf.Vec3d(0, 0, 0))
x_axis = transform.Transform(Gf.Vec3d(1, 0, 0)) - origin
y_axis = transform.Transform(Gf.Vec3d(0, 1, 0)) - origin
z_axis = transform.Transform(Gf.Vec3d(0, 0, 1)) - origin

print(f"LINK_6 Origin: {origin}")
print(f"LINK_6 X axis: {x_axis}")
print(f"LINK_6 Y axis: {y_axis}")
print(f"LINK_6 Z axis: {z_axis}")

# Let's also check the world pose of link_6 mesh to see where the flange points!
# The M0609 flange usually points along +Z or +Y. Let's look at the vertices!
mesh = find_prim_by_name(stage, "link_6_mesh")
if not mesh:
    for prim in link_6.GetChildren():
        if prim.IsA(UsdGeom.Mesh):
            mesh = prim
            break

if mesh:
    print(f"Mesh found: {mesh.GetPath()}")
    # We can infer outward normal by looking at the bounding box of link_6 in local space!
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default'])
    bbox = bbox_cache.ComputeLocalBound(link_6)
    r = bbox.GetRange()
    print(f"Link 6 Local BBox: Min {r.GetMin()}, Max {r.GetMax()}")

simulation_app.close()
