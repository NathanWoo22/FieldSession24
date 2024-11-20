import numpy as np
import open3d as o3d

template = o3d.io.read_point_cloud("datasets/fiducial_plane.pcd") 
data = o3d.io.read_point_cloud("tests/easy/nothing_done.pcd") 

template.paint_uniform_color([1, 0, 0])  # Red
data.paint_uniform_color([0, 1, 0]) # Green
o3d.visualization.draw_geometries([data, template])