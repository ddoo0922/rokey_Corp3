import numpy as np

def create_magic_mouse_points(resolution):
    x_min, x_max = -0.5, 0.5
    y_min, y_max = -0.5, 0.5
    z_max = 0.0

    xs = np.linspace(x_min, x_max, resolution)
    ys = np.linspace(y_min, y_max, resolution)

    x_range = x_max - x_min
    y_range = y_max - y_min
    
    points = []
    for x in xs:
        for y in ys:
            nx = x / (x_range / 2.0)
            ny = y / (y_range / 2.0)
            dome = 0.20 * (1.0 - nx**2) * (1.0 - ny**2)
            z = z_max + dome
            points.append([x, y, z])
    
    faces = []
    for ix in range(resolution - 1):
        for iy in range(resolution - 1):
            a = ix * resolution + iy
            b = (ix + 1) * resolution + iy
            c = (ix + 1) * resolution + (iy + 1)
            d = ix * resolution + (iy + 1)
            
            if points[a][2] > 0.01 and points[b][2] > 0.01 and points[c][2] > 0.01 and points[d][2] > 0.01:
                faces.append([a, b, c])
                faces.append([a, c, d])
            
    return np.array(points, dtype=np.float32), faces

pts, fcs = create_magic_mouse_points(80)
print("Points:", len(pts))
print("Faces:", len(fcs))
