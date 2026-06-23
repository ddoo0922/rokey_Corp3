from pxr import Usd, UsdGeom
stage = Usd.Stage.Open("OnRobot_Sander_v2.usd")
pad = stage.GetPrimAtPath("/OnRobot_Sander_v2/OnRobot_Sander_v2/tn__104327_")
xform = UsdGeom.Xformable(pad)
print("Pad local transform ops:")
for op in xform.GetOrderedXformOps():
    print(op.GetOpName(), op.Get())
