from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import os
from pxr import Usd, UsdGeom, Sdf

ROBOT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/M0609/doosan-robot2/usd/m0609.usd"
TOOL_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/sanding_kit.usd"
OUTPUT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/m0609_polishing.usd"

def find_prim_by_name(stage, prim_name):
    for prim in stage.TraverseAll():
        if prim.GetName() == prim_name:
            return prim
    return None

def assemble():
    print(f"[INFO] Assembling {ROBOT_USD_PATH} with {TOOL_USD_PATH}")
    
    # 1. 원본 로봇 USD 열기
    # 직접 수정하지 않기 위해 새로운 Stage 생성 후 Reference로 불러옴
    stage = Usd.Stage.CreateNew(OUTPUT_USD_PATH)
    root_path = "/World"
    UsdGeom.Xform.Define(stage, root_path)
    stage.SetDefaultPrim(stage.GetPrimAtPath(root_path))
    
    robot_path = f"{root_path}/m0609"
    robot_prim = stage.DefinePrim(robot_path)
    robot_prim.GetReferences().AddReference(ROBOT_USD_PATH)
    
    # link_6 찾기
    link_6_prim = None
    for prim in stage.TraverseAll():
        if prim.GetName() == "link_6":
            link_6_prim = prim
            break
            
    if not link_6_prim:
        print("[ERROR] Could not find 'link_6' in the robot USD.")
        return
        
    print(f"[INFO] Found link_6 at: {link_6_prim.GetPath()}")
    
    # link_6 하위에 Sanding Kit 참조 추가
    tool_path = f"{link_6_prim.GetPath()}/sanding_kit"
    tool_prim = stage.DefinePrim(tool_path)
    tool_prim.GetReferences().AddReference(TOOL_USD_PATH)
    
    # 회전 및 위치 미세조정 (필요시)
    # 샌딩 키트가 link_6의 표면에 맞게 부착되도록
    xform = UsdGeom.Xformable(tool_prim)
    xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    
    # 저장
    stage.GetRootLayer().Save()
    print(f"[SUCCESS] Saved Assembled Robot to: {OUTPUT_USD_PATH}")

if __name__ == "__main__":
    if os.path.exists(OUTPUT_USD_PATH):
        os.remove(OUTPUT_USD_PATH)
    from pxr import Gf
    try:
        assemble()
    finally:
        simulation_app.close()
