from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom
import sys

usd_path = "/home/rokey/rokey_Corp3/src/polishing_0623/scan_obj/car.usd"
stage = Usd.Stage.Open(usd_path)
if not stage:
    print("Could not open USD")
    app.close()
    sys.exit(1)

for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Xformable):
        xform = UsdGeom.Xformable(prim)
        ops = xform.GetOrderedXformOps()
        if ops:
            print(f"Prim: {prim.GetPath()}")
            for op in ops:
                print(f"  Op: {op.GetOpName()}, Value: {op.Get()}")
app.close()
