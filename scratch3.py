from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.replicator.core as rep
camera = rep.create.camera(position=(0,0,1.6), look_at=(0,0,0))
rp = rep.create.render_product(camera, (640, 480))
cam_params = rep.AnnotatorRegistry.get_annotator("camera_params")
cam_params.attach([rp])
for _ in range(5):
    rep.orchestrator.step()
print("KEYS:", cam_params.get_data().keys())
print("MATRIX:", cam_params.get_data()["cameraMatrix"])
simulation_app.close()
