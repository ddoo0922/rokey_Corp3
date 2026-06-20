from isaacsim import SimulationApp

# 처음 확인할 때는 False
simulation_app = SimulationApp({"headless": False})

import os
import json
import numpy as np
import omni
from pxr import Usd, UsdGeom, Gf

from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid


BASE_DIR = "/home/rokey/scan_project"
OUTPUT_DIR = os.path.join(BASE_DIR, "output")
POSE_DIR = os.path.join(OUTPUT_DIR, "pose")
MESH_DIR = os.path.join(OUTPUT_DIR, "mesh")

os.makedirs(POSE_DIR, exist_ok=True)
os.makedirs(MESH_DIR, exist_ok=True)


def save_pose_json(path: str, position: np.ndarray, orientation_wxyz: np.ndarray):
    data = {
        "position_m": position.tolist(),
        "orientation_wxyz": orientation_wxyz.tolist(),
    }

    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def save_mesh_info_json(prim_path: str, output_path: str):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)

    if not prim.IsValid():
        raise RuntimeError(f"Invalid prim path: {prim_path}")

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        includedPurposes=[UsdGeom.Tokens.default_]
    )

    bbox = bbox_cache.ComputeWorldBound(prim)
    box = bbox.ComputeAlignedBox()

    min_pt = box.GetMin()
    max_pt = box.GetMax()
    size = [
        float(max_pt[0] - min_pt[0]),
        float(max_pt[1] - min_pt[1]),
        float(max_pt[2] - min_pt[2]),
    ]

    center = [
        float((min_pt[0] + max_pt[0]) / 2.0),
        float((min_pt[1] + max_pt[1]) / 2.0),
        float((min_pt[2] + max_pt[2]) / 2.0),
    ]

    data = {
        "prim_path": prim_path,
        "bbox_min_m": [float(min_pt[0]), float(min_pt[1]), float(min_pt[2])],
        "bbox_max_m": [float(max_pt[0]), float(max_pt[1]), float(max_pt[2])],
        "bbox_size_m": size,
        "bbox_center_m": center
    }

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    scan_obj = world.scene.add(
        FixedCuboid(
            prim_path="/World/ScanObject",
            name="scan_object",
            position=np.array([0.65, 0.0, 0.20]),
            scale=np.array([0.12, 0.18, 0.08]),
            color=np.array([0.8, 0.2, 0.2]),
        )
    )

    world.reset()

    for _ in range(60):
        simulation_app.update()

    pos, quat = scan_obj.get_world_pose()

    pose_path = os.path.join(POSE_DIR, "object_pose.json")
    mesh_info_path = os.path.join(MESH_DIR, "object_mesh_info.json")

    save_pose_json(pose_path, pos, quat)
    save_mesh_info_json("/World/ScanObject", mesh_info_path)

    print(f"[OK] saved pose:      {pose_path}")
    print(f"[OK] saved mesh info: {mesh_info_path}")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
