from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
import numpy as np
import omni.replicator.core as rep

from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid


BASE_DIR = "/home/rokey/scan_project"
DEPTH_DIR = os.path.join(BASE_DIR, "output", "depth")
os.makedirs(DEPTH_DIR, exist_ok=True)

CAMERA_POSITION = (0.65, 0.0, 1.60)

# 여러 방향 테스트
ROTATION_LIST = [
    (0, 0, 0),
    (-90, 0, 0),
    (90, 0, 0),
    (0, 90, 0),
    (0, -90, 0),
    (180, 0, 0),
    (0, 180, 0),
]


def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    world.scene.add(
        FixedCuboid(
            prim_path="/World/ScanObject",
            name="scan_object",
            position=np.array([0.65, 0.0, 0.20]),
            scale=np.array([0.12, 0.18, 0.08]),
            color=np.array([0.8, 0.2, 0.2]),
        )
    )

    world.reset()

    for idx, rot in enumerate(ROTATION_LIST):
        print("\n===================================")
        print(f"[TEST {idx}] camera rotation = {rot}")
        print("===================================")

        camera = rep.create.camera(
            position=CAMERA_POSITION,
            rotation=rot
        )

        render_product = rep.create.render_product(
            camera,
            resolution=(320, 240)
        )

        depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        depth_annot.attach([render_product])

        for _ in range(20):
            simulation_app.update()
            rep.orchestrator.step()

        depth = depth_annot.get_data()
        depth = np.asarray(depth, dtype=np.float32)

        finite = depth[np.isfinite(depth)]

        if finite.size == 0:
            print("[RESULT] finite depth 없음")
        else:
            print("[RESULT] min:", float(np.min(finite)))
            print("[RESULT] max:", float(np.max(finite)))
            print("[RESULT] mean:", float(np.mean(finite)))
            print("[RESULT] finite count:", int(finite.size))

            save_path = os.path.join(DEPTH_DIR, f"test_depth_rotation_{idx}.npy")
            np.save(save_path, depth)
            print("[SAVED]", save_path)

        depth_annot.detach([render_product])


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
