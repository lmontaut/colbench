#
# Copyright (c) 2023 INRIA
# Author: Louis Montaut
#

import os
import hppfcl
import numpy as np
import pandas as pd

# We remove redundant meshes
meshes_to_ignore = [
    "063-b_marbles",
    "065-b_cups",
    "065-c_cups",
    "065-d_cups",
    "065-e_cups",
    "065-f_cups",
    "065-g_cups",
    "065-h_cups",
    "065-i_cups",
    "065-j_cups",
    "070-b_colored_wood_blocks",
    "072-b_toy_airplane",
    "072-c_toy_airplane",
    "072-d_toy_airplane",
    "072-e_toy_airplane",
    "073-b_lego_duplo",
    "073-c_lego_duplo",
    "073-d_lego_duplo",
    "073-e_lego_duplo",
    "073-f_lego_duplo",
    "073-g_lego_duplo",
]

def is_path_to_ignore(mesh_path: str) -> bool:
    for path in meshes_to_ignore:
        if mesh_path.count(path) > 0:
            return True
    return False

if __name__ == "__main__":
    if not os.path.exists("./benchmarks/ycb/data"):
        print("Please check that you are at the root of the repository and that the YCB dataset has been downloaded.\
              Run `python benchmarks/ycb/ycb_download.py` to download YCB.")
    else:
        path = os.path.abspath("./benchmarks/ycb/data/ycb_data")
        dirs = os.listdir(path)
        loader = hppfcl.MeshLoader()

        data = {}
        shape_path = []
        num_vertices = []
        num_faces = []
        i = 0
        for dir in dirs:
            mesh_path = os.path.join(path, dir, "google_16k/nontextured.stl")

            if not is_path_to_ignore(mesh_path):
                mesh: hppfcl.BVHModelBase = loader.load(mesh_path)
                mesh.buildConvexHull(True, "Qt")
                shape: hppfcl.ConvexBase = mesh.convex
                shape.buildDoubleDescription()
                print(f"{dir}: num vertices = {shape.num_points}")
                shape_path.append(mesh_path)
                num_vertices.append(shape.num_points)
                num_faces.append(shape.num_normals)
                i += 1
            else:
                print(f"Mesh {dir} ignored.")

        num_vertices = np.array(num_vertices)
        N = len(num_vertices)
        print(f"\n--------------------------")
        print(f"Number of shapes: {N}")
        print(f"Number of pairs: {int(N * (N-1) / 2)}")
        print(f"Min/Max number of vertices: {np.min(num_vertices)} / {np.max(num_vertices)}")
        print(f"Mean number of vertices: {int(np.mean(num_vertices))}")
        print(f"Std number of vertices: {int(np.std(num_vertices))}")
        print(f"--------------------------")

        data["shape_path"] = shape_path
        data["num_vertices"] = num_vertices
        data["num_faces"] = num_faces
        df = pd.DataFrame(data)
        df.to_csv("benchmarks/ycb/data/ycb.csv", index=False)
