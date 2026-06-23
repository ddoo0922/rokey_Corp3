import sys
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Gf

stage = Usd.Stage.Open("/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd")

pad_path = "/World/m0609/link_6/sanding_kit/OnRobot_Sander_v2/tn__104327_"
pad_prim = stage.GetPrimAtPath(pad_path)
link_6_prim = stage.GetPrimAtPath("/World/m0609/link_6")

if pad_prim and link_6_prim:
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default'])
    
    pad_bbox = bbox_cache.ComputeWorldBound(pad_prim).ComputeAlignedRange()
    link_6_bbox = bbox_cache.ComputeWorldBound(link_6_prim).ComputeAlignedRange()
    
    pad_center = pad_bbox.GetMidpoint()
    
    # We want the offset in link_6 local frame
    link_6_xform = UsdGeom.Xformable(link_6_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    
    # Transform pad center to link_6 local frame
    pad_center_local = link_6_xform.GetInverse().Transform(pad_center)
    
    print(f"PAD_CENTER_LOCAL_TO_LINK_6: {pad_center_local}")
    
    # Just to be sure, let's also find the pad's face (the bottom of the pad)
    # The pad's thin axis is Y in its local frame, but in world/link_6 frame we need to know where it is.
    # In link_6 frame, which axis points to the pad surface? 
    # Usually it's the bounding box min/max.
    pad_bbox_local = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default']).ComputeLocalBound(pad_prim).ComputeAlignedRange()
    print(f"PAD_BBOX_LOCAL: {pad_bbox_local}")
else:
    print("Could not find prims")

simulation_app.close()
