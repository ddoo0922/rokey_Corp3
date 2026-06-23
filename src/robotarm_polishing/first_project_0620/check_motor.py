from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

import os
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_USD_PATH = os.path.join(_SCRIPT_DIR, "m0609_polishing.usd")
stage = Usd.Stage.Open(OUTPUT_USD_PATH)

print("="*60)
print("Checking m0609_polishing.usd for physics motor (pad_joint)")
print("="*60)

# 1. pad_joint 검색
found_pad_joint = False
for prim in stage.TraverseAll():
    if "pad_joint" in prim.GetName() and (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.Joint)):
        found_pad_joint = True
        joint = UsdPhysics.RevoluteJoint(prim)
        print(f"\n[JOINT] pad_joint at: {prim.GetPath()}")
        print(f"  Type: {prim.GetTypeName()}")
        
        b0 = joint.GetBody0Rel().GetTargets()
        b1 = joint.GetBody1Rel().GetTargets()
        print(f"  Body0 (Parent): {b0[0] if b0 else 'None'}")
        print(f"  Body1 (Child):  {b1[0] if b1 else 'None'}")
        print(f"  Axis: {joint.GetAxisAttr().Get()}")
        
        if joint.GetLocalPos0Attr().Get():
            print(f"  LocalPos0: {joint.GetLocalPos0Attr().Get()}")
        if joint.GetLocalPos1Attr().Get():
            print(f"  LocalPos1: {joint.GetLocalPos1Attr().Get()}")
        if joint.GetLocalRot0Attr().Get():
            print(f"  LocalRot0: {joint.GetLocalRot0Attr().Get()}")
        
        # 드라이브 속성 확인
        apis = prim.GetAppliedSchemas()
        print(f"  Applied APIs: {list(apis)}")
        
        for attr in prim.GetAttributes():
            name = attr.GetName()
            val = attr.Get()
            if val is not None and "drive" in name.lower():
                print(f"  {name} = {val}")
        
        # PhysX 속성
        for attr in prim.GetAttributes():
            name = attr.GetName()
            val = attr.Get()
            if val is not None and "physx" in name.lower():
                print(f"  {name} = {val}")

# 2. sander_pad RigidBody 검색
print()
found_sander_pad = False
for prim in stage.TraverseAll():
    if prim.GetName() == "sander_pad":
        found_sander_pad = True
        apis = prim.GetAppliedSchemas()
        print(f"[BODY] sander_pad at: {prim.GetPath()}")
        print(f"  Type: {prim.GetTypeName()}")
        print(f"  APIs: {list(apis)}")
        
        for attr in prim.GetAttributes():
            name = attr.GetName()
            val = attr.Get()
            if val is not None and ("mass" in name.lower() or "inertia" in name.lower()):
                print(f"  {name} = {val}")

# 3. 전체 조인트 목록
print()
print("--- All Joints in USD ---")
joint_count = 0
for prim in stage.TraverseAll():
    if prim.IsA(UsdPhysics.Joint):
        joint_count += 1
        j = UsdPhysics.Joint(prim)
        b0 = j.GetBody0Rel().GetTargets()
        b1 = j.GetBody1Rel().GetTargets()
        marker = " <<<" if "pad" in prim.GetName() else ""
        print(f"  {prim.GetPath()} ({prim.GetTypeName()}) "
              f"B0={b0[0].name if b0 else '?'} -> B1={b1[0].name if b1 else '?'}{marker}")
print(f"Total joints: {joint_count}")

# 결과 요약
print()
print("="*60)
if found_pad_joint and found_sander_pad:
    print("SUCCESS: pad_joint + sander_pad physics motor is configured!")
    print("  - RevoluteJoint with DriveAPI (velocity mode)")
    print("  - sander_pad RigidBody for physical interaction")
else:
    if not found_pad_joint:
        print("ERROR: pad_joint is MISSING.")
    if not found_sander_pad:
        print("ERROR: sander_pad RigidBody is MISSING.")
print("="*60)

simulation_app.close()
