# app/export_usd_mesh.py
import os
import numpy as np
import open3d as o3d
import omni
from pxr import Usd, UsdGeom, Gf

def triangulate_face(indices):
    # fan triangulation
    tris = []
    for i in range(1, len(indices) - 1):
        tris.append([indices[0], indices[i], indices[i + 1]])
    return tris

def export_first_mesh_under_prim(root_prim_path: str, output_mesh_path: str):
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_prim_path)
    if not root.IsValid():
        raise RuntimeError(f"Invalid prim path: {root_prim_path}")

    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            points_local = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)

            counts = list(mesh.GetFaceVertexCountsAttr().Get())
            indices = list(mesh.GetFaceVertexIndicesAttr().Get())

            # local -> world transform
            xform = UsdGeom.Xformable(prim)
            world_tf = xform.ComputeLocalToWorldTransform(0.0)

            points_world = []
            for p in points_local:
                p4 = Gf.Vec4d(float(p[0]), float(p[1]), float(p[2]), 1.0)
                pw = world_tf.Transform(p4)
                points_world.append([pw[0], pw[1], pw[2]])
            points_world = np.asarray(points_world)

            faces = []
            cursor = 0
            for c in counts:
                face_idx = indices[cursor: cursor + c]
                faces.extend(triangulate_face(face_idx))
                cursor += c

            o3d_mesh = o3d.geometry.TriangleMesh()
            o3d_mesh.vertices = o3d.utility.Vector3dVector(points_world)
            o3d_mesh.triangles = o3d.utility.Vector3iVector(np.asarray(faces, dtype=np.int32))
            o3d_mesh.compute_vertex_normals()

            ok = o3d.io.write_triangle_mesh(output_mesh_path, o3d_mesh)
            if not ok:
                raise RuntimeError(f"mesh write failed: {output_mesh_path}")
            return output_mesh_path

    raise RuntimeError(f"No UsdGeom.Mesh found under {root_prim_path}")

