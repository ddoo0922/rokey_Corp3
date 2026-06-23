from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import os
from pxr import Usd, UsdGeom, Gf

OUTPUT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/sanding_kit.usd"

def create_sanding_kit():
    print("[INFO] Creating programmatic Sanding Kit USD...")
    stage = Usd.Stage.CreateNew(OUTPUT_USD_PATH)
    
    # Root Xform
    root_path = "/sanding_kit"
    root = UsdGeom.Xform.Define(stage, root_path)
    stage.SetDefaultPrim(root.GetPrim())

    # Robotiq Sanding Kit는 위쪽에 로봇 결합부(실린더), 아래쪽에 150mm(5~6인치) 패드가 있음.
    # 크기 설정
    body_radius = 0.04
    body_height = 0.08
    pad_radius = 0.075 # 150mm 직경
    pad_height = 0.02
    
    # Body (검은색/회색 실린더)
    body_path = f"{root_path}/body"
    body = UsdGeom.Cylinder.Define(stage, body_path)
    body.GetRadiusAttr().Set(body_radius)
    body.GetHeightAttr().Set(body_height)
    body.GetDisplayColorAttr().Set([Gf.Vec3f(0.2, 0.2, 0.2)])
    # Z축 아래로 향하도록 세팅 (link_6 결합부가 상단(Z=0)이라고 가정)
    UsdGeom.Xformable(body).AddTranslateOp().Set(Gf.Vec3d(0, 0, -body_height/2))
    
    # Pad (파란색 또는 노란색 스펀지 패드)
    pad_path = f"{root_path}/pad"
    pad = UsdGeom.Cylinder.Define(stage, pad_path)
    pad.GetRadiusAttr().Set(pad_radius)
    pad.GetHeightAttr().Set(pad_height)
    pad.GetDisplayColorAttr().Set([Gf.Vec3f(0.2, 0.5, 0.8)])
    UsdGeom.Xformable(pad).AddTranslateOp().Set(Gf.Vec3d(0, 0, -body_height - pad_height/2))
    
    # Tool Center Point (TCP) (패드의 맨 아래 표면 중앙)
    tcp_path = f"{root_path}/tcp"
    tcp = UsdGeom.Xform.Define(stage, tcp_path)
    UsdGeom.Xformable(tcp).AddTranslateOp().Set(Gf.Vec3d(0, 0, -body_height - pad_height))

    stage.GetRootLayer().Save()
    print(f"[SUCCESS] Saved Dummy Sanding Kit USD at: {OUTPUT_USD_PATH}")

if __name__ == "__main__":
    if os.path.exists(OUTPUT_USD_PATH):
        os.remove(OUTPUT_USD_PATH)
    try:
        create_sanding_kit()
    finally:
        simulation_app.close()
