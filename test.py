import open3d as o3d
import feature_matches as fm
import numpy as np

# Create a point cloud object
target = o3d.geometry.PointCloud()
target = o3d.io.read_point_cloud("datasets/fiducial_plane.pcd") 

source = o3d.geometry.PointCloud()
source = o3d.io.read_point_cloud("tests/rotation_easy.pcd") 

gt = np.loadtxt("tests/rotation_easy_gt.txt")

fm.icp(target, source)
print(gt)
#Test each algorithm 