import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.replicator.core as rep
camera = rep.create.camera(position=(0, -0.75, 2.5), look_at=(0, -0.75, 0), focal_length=24.0, horizontal_aperture=20.955)
rp = rep.create.render_product(camera, (640, 480))
cam_params = rep.AnnotatorRegistry.get_annotator("camera_params")
cam_params.attach([rp])
rep.orchestrator.step()
rep.orchestrator.step()
data = cam_params.get_data()
fx = data['cameraMatrix'][0][0]
fy = data['cameraMatrix'][1][1]
cx = data['cameraMatrix'][0][2]
cy = data['cameraMatrix'][1][2]
V = data['cameraViewTransform']
V_inv = np.linalg.inv(V)

# Simulate a point at center of image (cx, cy) with depth 2.5
z_cam = 2.5
x_cam = (cx - cx) * z_cam / fx
y_cam = (cy - cy) * z_cam / fy
P_c = np.array([x_cam, y_cam, -z_cam, 1.0])
P_w = V_inv @ P_c
print("With -z_cam:", P_w[:3] / P_w[3])

P_c2 = np.array([x_cam, y_cam, z_cam, 1.0])
P_w2 = V_inv @ P_c2
print("With +z_cam:", P_w2[:3] / P_w2[3])

# Check a point to the right (+x_cam)
P_c3 = np.array([1.0, 0.0, -z_cam, 1.0])
print("Right with -z_cam:", (V_inv @ P_c3)[:3] / (V_inv @ P_c3)[3])

simulation_app.close()
