from pxr import Usd, UsdGeom, Gf

stage = Usd.Stage.Open("/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd")

pad_path = "/World/m0609/link_6/sanding_kit/OnRobot_Sander_v2/tn__104327_"
pad_prim = stage.GetPrimAtPath(pad_path)

if pad_prim:
    print(f"Found pad prim: {pad_path}")
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default'])
    bbox = bbox_cache.ComputeWorldBound(pad_prim).ComputeAlignedRange()
    link6_bbox = bbox_cache.ComputeWorldBound(stage.GetPrimAtPath("/World/m0609/link_6")).ComputeAlignedRange()
    
    pad_center = bbox.GetMidpoint()
    link6_center = link6_bbox.GetMidpoint()
    
    print(f"Pad Center World: {pad_center}")
    print(f"Link 6 Center World: {link6_center}")
    
    # Let's get local transform of pad relative to sanding_kit
    sanding_kit = stage.GetPrimAtPath("/World/m0609/link_6/sanding_kit")
    kit_xform = UsdGeom.Xformable(sanding_kit).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    
    pad_xform = UsdGeom.Xformable(pad_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    
    pad_local = pad_xform * kit_xform.GetInverse()
    print("Pad transform relative to sanding_kit:")
    print(pad_local)
else:
    print("Pad prim not found!")
