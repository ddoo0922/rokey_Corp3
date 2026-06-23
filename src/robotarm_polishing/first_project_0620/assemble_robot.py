from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import os
from pxr import Usd, UsdGeom, Sdf, UsdPhysics, Gf, PhysxSchema, Vt

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROBOT_USD_PATH = os.path.join(os.path.dirname(_SCRIPT_DIR), "M0609", "doosan-robot2", "usd", "m0609.usd")
MOUNT_USD_PATH = os.path.join(_SCRIPT_DIR, "HEX_E_H_QC.usd")
TOOL_USD_PATH = os.path.join(_SCRIPT_DIR, "OnRobot_Sander_v2.usd")
OUTPUT_USD_PATH = os.path.join(_SCRIPT_DIR, "m0609_polishing.usd")

# tn__104327_ 패드 메쉬의 sander 모델 내 중심 위치 (mm 단위, BBox 분석 결과)
PAD_CENTER_MM = Gf.Vec3d(-2.5, -11.3, 0.0)
PAD_RADIUS_M = 0.0867   # 173.4mm / 2
PAD_THICKNESS_M = 0.023  # 23mm

def assemble():
    stage = Usd.Stage.CreateNew(OUTPUT_USD_PATH)
    root_path = "/World"
    UsdGeom.Xform.Define(stage, root_path)
    stage.SetDefaultPrim(stage.GetPrimAtPath(root_path))
    
    robot_prim = stage.DefinePrim(f"{root_path}/m0609")
    robot_prim.GetReferences().AddReference(ROBOT_USD_PATH)
    
    link_6_prim = None
    for prim in stage.TraverseAll():
        if prim.GetName() == "link_6":
            link_6_prim = prim; break
    if not link_6_prim: print("[ERROR] link_6 not found"); return
    
    L6 = str(link_6_prim.GetPath())
    M = f"{root_path}/m0609/m0609"
    
    # ================================================================
    # 1. 샌더 본체 → link_6 고정 (tn__104327_ 숨김)
    # ================================================================
    mount_path = f"{L6}/quick_mount"
    mp = stage.DefinePrim(mount_path)
    mp.GetReferences().AddReference(MOUNT_USD_PATH)
    mx = UsdGeom.Xformable(mp)
    mx.ClearXformOpOrder()
    mx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.015))
    mx.AddRotateXYZOp().Set(Gf.Vec3d(-90, 0, 0))
    mx.AddScaleOp().Set(Gf.Vec3d(0.001, 0.001, 0.001))
    
    kit_path = f"{mount_path}/sanding_kit"
    kp = stage.DefinePrim(kit_path)
    kp.GetReferences().AddReference(TOOL_USD_PATH)
    kx = UsdGeom.Xformable(kp)
    kx.ClearXformOpOrder()
    kx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -20))
    kx.AddScaleOp().Set(Gf.Vec3d(1, 1, 1))
    
    # 본체의 패드 메쉬 숨김 (물리 패드로 대체)
    body_pad = stage.GetPrimAtPath(f"{kit_path}/OnRobot_Sander_v2/tn__104327_")
    if body_pad and body_pad.IsValid():
        UsdGeom.Imageable(body_pad).CreateVisibilityAttr().Set("invisible")
        print("[INFO] Hidden body pad mesh")
    
    # 패드 위치 계산
    cache = UsdGeom.XformCache()
    pad_world = cache.GetLocalToWorldTransform(body_pad)
    l6_world = cache.GetLocalToWorldTransform(link_6_prim)
    pad_in_l6 = pad_world * l6_world.GetInverse()
    pad_pos = pad_in_l6.ExtractTranslation()
    print(f"[INFO] Pad in link_6: {pad_pos}")
    
    # ================================================================
    # 2. sander_pad RigidBody (패드 판만, m0609 flat)
    #    tn__104327_ 메쉬를 여기에 넣어서 회전 + 물리 접촉
    # ================================================================
    SP = f"{M}/sander_pad"
    sp = stage.DefinePrim(SP, "Xform")
    UsdPhysics.RigidBodyAPI.Apply(sp)
    ma = UsdPhysics.MassAPI.Apply(sp)
    ma.CreateMassAttr().Set(0.3)
    ma.CreateDensityAttr().Set(0.0)
    ma.CreateCenterOfMassAttr().Set(Gf.Vec3f(0, 0, 0))
    ma.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(0.0001, 0.0001, 0.0001))
    
    l6p = l6_world.ExtractTranslation()
    sx = UsdGeom.Xformable(sp)
    sx.ClearXformOpOrder()
    sx.AddTranslateOp().Set(Gf.Vec3d(l6p[0]+pad_pos[0], l6p[1]+pad_pos[1], l6p[2]+pad_pos[2]))
    sx.AddOrientOp().Set(Gf.Quatf(0.7071068, -0.7071068, 0, 0))
    sx.AddScaleOp().Set(Gf.Vec3d(1, 1, 1))
    
    # --- tn__104327_ 실제 메쉬 표시 (sander ref에서 패드만 보이게) ---
    vis_path = f"{SP}/pad_visual"
    vp = UsdGeom.Xform.Define(stage, vis_path)
    vx = UsdGeom.Xformable(vp.GetPrim())
    vx.ClearXformOpOrder()
    vx.AddTranslateOp().Set(Gf.Vec3d(0.0025, 0.0113, 0.020))
    vx.AddScaleOp().Set(Gf.Vec3d(0.001, 0.001, 0.001))
    
    pad_ref_path = f"{vis_path}/sander_ref"
    pr = stage.DefinePrim(pad_ref_path)
    pr.GetReferences().AddReference(TOOL_USD_PATH)
    prx = UsdGeom.Xformable(pr)
    prx.ClearXformOpOrder()
    prx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -20))
    prx.AddScaleOp().Set(Gf.Vec3d(1, 1, 1))
    
    # 패드(tn__104327_)만 보이고 물리 적용, 나머지 숨김
    sander_root = f"{pad_ref_path}/OnRobot_Sander_v2"
    sr = stage.GetPrimAtPath(sander_root)
    if sr and sr.IsValid():
        for child in sr.GetChildren():
            if child.GetName() == "tn__104327_":
                UsdGeom.Imageable(child).CreateVisibilityAttr().Set("inherited")
                # 실제 메쉬에 Collision 추가
                coll_api = UsdPhysics.CollisionAPI.Apply(child)
                coll_api.CreateCollisionEnabledAttr().Set(True)
                mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(child)
                mesh_coll.CreateApproximationAttr().Set("convexHull") # 동적 충돌을 위해 필수
                print("[INFO] Applied convexHull collision to tn__104327_")
            elif child.GetTypeName() != "Scope":
                UsdGeom.Imageable(child).CreateVisibilityAttr().Set("invisible")
    
    # ================================================================
    # 3. pad_joint + 모터
    # ================================================================
    pj = UsdPhysics.RevoluteJoint.Define(stage, f"{L6}/pad_joint")
    pj.CreateBody0Rel().SetTargets([L6])
    pj.CreateBody1Rel().SetTargets([SP])
    pj.CreateAxisAttr().Set("Y")
    pj.CreateLocalPos0Attr().Set(Gf.Vec3f(float(pad_pos[0]), float(pad_pos[1]), float(pad_pos[2])))
    pj.CreateLocalRot0Attr().Set(Gf.Quatf(0.7071068, -0.7071068, 0, 0))
    pj.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    pj.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    pj.CreateCollisionEnabledAttr().Set(False)
    pj.CreateJointEnabledAttr().Set(True)
    pj.CreateExcludeFromArticulationAttr().Set(False)
    pj.CreateBreakForceAttr().Set(3.4e+38)
    pj.CreateBreakTorqueAttr().Set(3.4e+38)
    
    d = UsdPhysics.DriveAPI.Apply(pj.GetPrim(), "angular")
    d.CreateTypeAttr().Set("force")
    d.CreateStiffnessAttr().Set(0.0)
    d.CreateDampingAttr().Set(10.0)
    d.CreateMaxForceAttr().Set(100.0)
    d.CreateTargetVelocityAttr().Set(3000.0)
    d.CreateTargetPositionAttr().Set(0.0)
    PhysxSchema.PhysxJointAPI.Apply(pj.GetPrim()).CreateMaxJointVelocityAttr().Set(6000.0)
    
    # --- 손잡이 부분 (link_6 하위) 물리 효과 완전히 제거 ---
    # mount_path 내의 모든 prim을 순회하며 CollisionAPI가 있으면 비활성화
    mp_prim = stage.GetPrimAtPath(mount_path)
    for p in Usd.PrimRange(mp_prim):
        if p.HasAPI(UsdPhysics.CollisionAPI):
            coll_api = UsdPhysics.CollisionAPI(p)
            coll_api.CreateCollisionEnabledAttr().Set(False)
    
    stage.GetRootLayer().Save()
    print(f"[INFO] Saved: {OUTPUT_USD_PATH}")
    print("[INFO] Structure: fixed body(no physics) + visible physics pad mesh(tn__104327_)")

if __name__ == "__main__":
    assemble()