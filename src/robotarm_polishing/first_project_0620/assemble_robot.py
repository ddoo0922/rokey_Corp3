from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import os
from pxr import Usd, UsdGeom, Sdf, UsdPhysics, Gf, PhysxSchema

ROBOT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/M0609/doosan-robot2/usd/m0609.usd"
TOOL_USD_PATH = "/home/rokey/cobot4_ws/Downloads/OnRobot_Sander_v2.usd"
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
    xform = UsdGeom.Xformable(tool_prim)
    xform.ClearXformOpOrder() # 기존 레퍼런스의 꼬인 변환 순서를 초기화
    
    # 사용자님이 직접 조립하신 사진을 보고 완벽히 이해했습니다.
    # 툴의 메인 축은 원통이 아니라 "Z축(패드~커넥터)" 방향이었습니다! (긴 손잡이는 X축)
    # 원점이 패드 쪽에 있으므로, 180도 뒤집고 Z축으로 툴 길이(11.45cm)만큼 밖으로 빼주어야 커넥터가 플랜지에 정확히 붙습니다.
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.015))
    xform.AddRotateXYZOp().Set(Gf.Vec3d(-90, 0, 0))
    xform.AddScaleOp().Set(Gf.Vec3d(0.001, 0.001, 0.001))
    
    # 패드 조인트(모터) 추가
    pad_path = f"{tool_path}/OnRobot_Sander_v2/tn__104327_"
    pad_joint_path = f"{tool_path}/pad_joint"
    
    # 샌딩 패드(tn__104327_)는 물리 시뮬레이션에서 강체(RigidBody)로 설정하면 
    # 로봇 팔 끝단(link_6) 하위의 계층 구조 문제로 폭발(Snapping/Explosion)을 일으킵니다.
    # 드라이브 설정 코드는 조인트 제거와 함께 삭제되었습니다.
    
    
    # 저장
    stage.GetRootLayer().Save()
    

if __name__ == "__main__":
    assemble()
