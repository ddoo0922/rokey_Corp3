from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema
stage = Usd.Stage.Open("/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd")
pad = stage.GetPrimAtPath("/World/m0609/link_6/sanding_kit/tn__104327_")
mass_api = UsdPhysics.MassAPI(pad)
print("Mass:", mass_api.GetMassAttr().Get())
