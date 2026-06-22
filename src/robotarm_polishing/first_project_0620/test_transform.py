from scipy.spatial.transform import Rotation as R
import numpy as np

# Tool connector face in tool local coordinates (mm)
connector_local = np.array([191.0, 10.0, 0.0])

# We want to rotate the tool by -90 degrees around Y
rot = R.from_euler('y', -90, degrees=True)
connector_rotated = rot.apply(connector_local)

print(f"Connector position after rotation: {connector_rotated}")

# To put the connector at (0,0,0), we need to translate by -connector_rotated
translation_mm = -connector_rotated
print(f"Translation needed (mm): {translation_mm}")
print(f"Translation needed (m): {translation_mm / 1000.0}")
