from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": True,
})

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.kit.asset_converter.cad")


import os
import asyncio
import omni.kit.asset_converter

# 경로 설정
STEP_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/sanding-kit-robotiq-1.snapshot.2/Step/sanding_kit.step"
OUTPUT_USD_PATH = "/home/rokey/cobot4_ws/rokey_Corp3/src/robotarm_polishing/first_project_0620/sanding_kit.usd"

def progress_callback(progress, total_steps):
    percent = (progress / total_steps) * 100 if total_steps > 0 else 0
    print(f"[Conversion Progress] {percent:.1f}% ({progress}/{total_steps})")

async def convert_step_to_usd():
    print(f"[INFO] Starting conversion from {STEP_PATH}")
    print(f"[INFO] Target USD: {OUTPUT_USD_PATH}")
    
    # 변환기 매니저 인스턴스 가져오기
    converter_manager = omni.kit.asset_converter.get_instance()
    
    # 변환 컨텍스트(설정) 생성
    context = omni.kit.asset_converter.AssetConverterContext()
    context.ignore_materials = False
    context.calc_normals = True
    context.merge_all_meshes = True # 최적화를 위해 메쉬 병합
    context.use_meter_as_world_unit = True # 스케일을 미터로 맞춤
    
    # 변환 작업 생성
    task = converter_manager.create_converter_task(
        STEP_PATH,
        OUTPUT_USD_PATH,
        progress_callback,
        context
    )
    
    # 작업 완료 대기
    success = await task.wait_until_finished()
    if success:
        print("\n[SUCCESS] CAD to USD Conversion Completed!")
        print(f"[SUCCESS] Saved at: {OUTPUT_USD_PATH}")
    else:
        print("\n[ERROR] Failed to convert!")
        status = task.get_status()
        error_msg = task.get_detailed_error()
        print(f"[ERROR] Status: {status}")
        print(f"[ERROR] Detail: {error_msg}")

def main():
    if not os.path.exists(STEP_PATH):
        raise FileNotFoundError(f"STEP file not found: {STEP_PATH}")
        
    loop = asyncio.get_event_loop()
    loop.run_until_complete(convert_step_to_usd())

if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
