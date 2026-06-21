import numpy as np

def create_magic_mouse_points(resolution):
    num_rings = resolution
    num_sectors = resolution
    
    points = []
    points.append([0.0, 0.0, 0.20])
    
    for i in range(1, num_rings + 1):
        r = float(i) / num_rings
        z = 0.20 * np.sqrt(max(0.0, 1.0 - r**2))
        xy_radius = r * 0.5
        
        for j in range(num_sectors):
            theta = 2.0 * np.pi * float(j) / num_sectors
            x = xy_radius * np.cos(theta)
            y = xy_radius * np.sin(theta)
            points.append([x, y, z])
            
    faces = []
    for j in range(num_sectors):
        next_j = (j + 1) % num_sectors
        faces.append([0, 1 + j, 1 + next_j])
        
    for i in range(1, num_rings):
        ring_start = 1 + (i - 1) * num_sectors
        next_ring_start = 1 + i * num_sectors
        for j in range(num_sectors):
            next_j = (j + 1) % num_sectors
            
            a = ring_start + j
            b = ring_start + next_j
            c = next_ring_start + next_j
            d = next_ring_start + j
            
            faces.append([a, b, c])
            faces.append([a, c, d])
            
    return np.array(points, dtype=np.float32), faces

pts, fcs = create_magic_mouse_points(80)
print(len(pts), len(fcs))
