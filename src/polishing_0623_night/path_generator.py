"""
path_generator.py — 3D 표면 경로 생성 알고리즘
"""
import os
import argparse
import numpy as np

def load_ply_points(ply_path):
    points = []
    header_done = False
    with open(ply_path, "r") as f:
        for line in f:
            line = line.strip()
            if not header_done:
                if line == "end_header":
                    header_done = True
                continue
            parts = line.split()
            if len(parts) >= 3:
                try:
                    points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                except ValueError:
                    continue
    return np.asarray(points, dtype=np.float32)

def generate_3d_raster_path(points, step=0.03, resolution=0.01):
    """주어진 3D 점들을 기반으로 지그재그(Raster) 경로를 추출"""
    if len(points) == 0:
        return np.array([])
        
    min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
    min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])
    
    # 격자(Grid) 생성
    x_bins = np.arange(min_x, max_x, resolution)
    y_bins = np.arange(min_y, max_y, step)
    
    path_3d = []
    
    for i, y in enumerate(y_bins):
        # 현재 Y 구간에 속하는 점들 추출
        y_mask = (points[:, 1] >= y) & (points[:, 1] < y + step)
        row_points = points[y_mask]
        
        if len(row_points) == 0:
            continue
            
        # X 방향 정렬 (짝수 줄은 정방향, 홀수 줄은 역방향으로 지그재그 생성)
        row_points = row_points[np.argsort(row_points[:, 0])]
        if i % 2 == 1:
            row_points = row_points[::-1]
            
        # 해상도(resolution) 간격으로 점 샘플링
        sampled_row = []
        last_x = None
        for p in row_points:
            if last_x is None or abs(p[0] - last_x) >= resolution:
                sampled_row.append(p)
                last_x = p[0]
                
        path_3d.extend(sampled_row)
        
    return np.array(path_3d)

import sys

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--obj_name", type=str, default="car")
    args, unknown = parser.parse_known_args()

    BASE_DIR = "/home/rokey/rokey_Corp3/src/polishing_0623"
    scan_dir = os.path.join(BASE_DIR, "scan_result", args.obj_name)
    ply_path = os.path.join(scan_dir, "points", "real_camera_surface_points.ply")
    path_output = os.path.join(scan_dir, "path.npy")
    
    print(f"[path_generator] 로드 중: {ply_path}")
    if not os.path.exists(ply_path):
        print(f"[ERROR] PLY 파일을 찾을 수 없습니다: {ply_path}")
        sys.exit(1)
        
    points = load_ply_points(ply_path)
    print(f"[path_generator] 원본 점 개수: {len(points)}")
    
    # 로봇의 작업 반경 필터링 (베이스: -0.8, -0.7, 0.2 / 최대 0.9m)
    robot_base = np.array([-0.8, -0.7, 0.2])
    max_radius = 0.9
    
    # 2D 평면(XY) 기준 거리 (높이는 별도 고려 가능하나 기본적으로 로봇 팔 길이를 고려해 3D 거리 사용)
    dists = np.linalg.norm(points - robot_base, axis=1)
    reachable_points = points[dists <= max_radius]
    
    print(f"[path_generator] 로봇 반경 내 점 개수: {len(reachable_points)}")
    
    if len(reachable_points) == 0:
        print("[ERROR] 로봇 반경 내에 작업할 수 있는 점이 없습니다!")
        sys.exit(1)
        
    # Raster 경로 생성
    path_3d = generate_3d_raster_path(reachable_points, step=0.03, resolution=0.02)
    print(f"[path_generator] 생성된 3D 경로 웨이포인트 개수: {len(path_3d)}")
    
    np.save(path_output, path_3d)
    print(f"[path_generator] 저장 완료: {path_output}")

if __name__ == "__main__":
    main()
