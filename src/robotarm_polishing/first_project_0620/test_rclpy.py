from isaacsim import SimulationApp
sim = SimulationApp({'headless': True})
from omni.isaac.core.utils.extensions import enable_extension
enable_extension('omni.isaac.ros2_bridge')
sim.update()
try:
    import rclpy
    print('OK_RCLPY')
except Exception as e:
    print('ERROR:', e)
sim.close()
