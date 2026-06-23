import os
import subprocess
import tkinter as tk
from tkinter import simpledialog, messagebox

BASE_DIR = "/home/rokey/rokey_Corp3/src/polishing_0623"
PYTHON_SH = "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh"

def main():
    # 1. 2D UI 팝업으로 물체 이름 입력 받기
    root = tk.Tk()
    root.withdraw() # 메인 윈도우 숨기기
    
    obj_name = simpledialog.askstring(
        title="폴리싱 물체 선택", 
        prompt="스캔 및 폴리싱할 물체의 이름을 입력하세요\n(예: car, cube):"
    )
    
    if not obj_name:
        print("[INFO] 물체 이름이 입력되지 않아 프로그램을 종료합니다.")
        return
        
    obj_name = obj_name.strip().lower()
    print(f"[INFO] 선택된 물체: {obj_name}")
    
    # 2. USD 파일 존재 여부 확인
    usd_path = os.path.join(BASE_DIR, "scan_obj", f"{obj_name}.usd")
    if not os.path.exists(usd_path):
        messagebox.showerror("오류", f"물체 파일이 존재하지 않습니다:\n{usd_path}")
        return

    # 3. 스캔 폴더 확인
    scan_dir = os.path.join(BASE_DIR, "scan_result", obj_name)
    points_dir = os.path.join(scan_dir, "points")
    ply_path = os.path.join(points_dir, "real_camera_surface_points.ply")
    path_file = os.path.join(scan_dir, "path.npy")
    
    if not os.path.exists(ply_path) or not os.path.exists(path_file):
        print(f"[INFO] '{obj_name}'에 대한 스캔 데이터가 없습니다. 스캔을 시작합니다...")
        
        # 스캔 실행 (scan.py)
        print("--------------------------------------------------")
        print(f">>> Running scan.py --obj_name {obj_name}")
        scan_result = subprocess.run([PYTHON_SH, os.path.join(BASE_DIR, "scan.py"), "--obj_name", obj_name])
        if scan_result.returncode != 0:
            print("[ERROR] 스캔 과정에서 오류가 발생했습니다.")
            return
            
        # 경로 생성 실행 (path_generator.py)
        print("--------------------------------------------------")
        print(f">>> Running path_generator.py --obj_name {obj_name}")
        # Note: path_generator.py uses system python because it doesn't need Isaac Sim, but we can just use python3
        path_result = subprocess.run(["python3", os.path.join(BASE_DIR, "path_generator.py"), "--obj_name", obj_name])
        if path_result.returncode != 0:
            print("[ERROR] 3D 경로 생성 과정에서 오류가 발생했습니다.")
            return
    else:
        print(f"[INFO] '{obj_name}'의 스캔 데이터를 찾았습니다. 스캔을 생략합니다.")
        
    # 4. 폴리싱 시뮬레이션 실행
    print("--------------------------------------------------")
    print(f">>> Running run_polishing_sim_0623.py --obj_name {obj_name}")
    sim_result = subprocess.run([PYTHON_SH, os.path.join(BASE_DIR, "run_polishing_sim_0623.py"), "--obj_name", obj_name])
    
    print("[INFO] 파이프라인 종료.")

if __name__ == "__main__":
    main()
