from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import time
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

cube_prim = DynamicCuboid(                              # 4. Prim
    prim_path="/World/BlueCube",
    name="blue_cube",
    position=np.array([0.0, 0.0, 0.5]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([0.0, 0.0, 1.0]),
)

world.scene.add_default_ground_plane()                  # 5. Scene
world.scene.add(cube_prim)

world.reset()

# --- 이 부분이 스크린샷의 루프 제어 코드가 합쳐진 부분입니다 ---
step_count = 0

while simulation_app.is_running():                      # 6. Simulation
    world.step(render=True)
    time.sleep(0.01)
    step_count += 1
    
    # 💡 여기서 step_count 조건을 이용해 무언가를 제어할 수 있습니다!
    # 예: if step_count == 100: ...

simulation_app.close()