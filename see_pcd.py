import numpy as np
import open3d as o3d

template = o3d.io.read_point_cloud("datasets/fiducial_plane.pcd") 
data = o3d.io.read_point_cloud("tests/easy/rotation_easy.pcd") 


o3d.visualization.draw_geometries([data, template])