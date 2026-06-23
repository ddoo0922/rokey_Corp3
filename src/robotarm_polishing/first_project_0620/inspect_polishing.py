from pxr import Usd
import os
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
stage = Usd.Stage.Open(os.path.join(_SCRIPT_DIR, "m0609_polishing.usd"))
for prim in stage.TraverseAll():
    if "pad_joint" in prim.GetName():
        print(prim.GetPath(), prim.GetTypeName())
