from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, Sdf

stage = Usd.Stage.Open("m0609_polishing.usd")
changed = False

def replace_refs(prim_spec):
    global changed
    ref_list = prim_spec.referenceList
    if ref_list:
        new_prepended = []
        for ref in ref_list.prependedItems:
            if 'm0609.usd' in ref.assetPath:
                new_ref = Sdf.Reference('../M0609/doosan-robot2/usd/m0609.usd', ref.primPath, ref.layerOffset, ref.customData)
                new_prepended.append(new_ref)
                changed = True
            elif 'OnRobot_Sander_v2.usd' in ref.assetPath:
                new_ref = Sdf.Reference('./OnRobot_Sander_v2.usd', ref.primPath, ref.layerOffset, ref.customData)
                new_prepended.append(new_ref)
                changed = True
            else:
                new_prepended.append(ref)
                
        if changed:
            ref_list.prependedItems = new_prepended
            
    for child in prim_spec.nameChildren:
        replace_refs(child)

layer = Sdf.Layer.FindOrOpen("m0609_polishing.usd")
replace_refs(layer.pseudoRoot)

if changed:
    layer.Save()
    print("USD file updated successfully.")
else:
    print("No references needed updating.")

simulation_app.close()
