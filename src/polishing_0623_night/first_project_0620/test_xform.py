from pxr import Usd, UsdGeom
stage = Usd.Stage.Open('/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd')
pad_prim = stage.GetPrimAtPath('/World/m0609/m0609/link_6/sanding_kit/OnRobot_Sander_v2/tn__104327_')
xformable = UsdGeom.Xformable(pad_prim)
print(f"XformOpOrder: {xformable.GetXformOpOrderAttr().Get()}")
print(f"ResetXformStack: {xformable.GetResetXformStack()}")
