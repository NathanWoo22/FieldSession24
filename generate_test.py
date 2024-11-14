import numpy as np
import open3d as o3d
import copy

# template_file = np.load("./datasets/fiducial_plane.npz")

# template_npz = template_file['point_cloud']
# template_npz = template_npz[:, :3]
                                 
# template = o3d.geometry.PointCloud()
# template.points = o3d.utility.Vector3dVector(template_npz)

template = o3d.io.read_point_cloud("datasets/fiducial_plane.pcd") 

template_copy = copy.deepcopy(template)

initial_transform = np.array([
    [0, 1, 0, 1],  
    [1, 0, 0, 0],  
    [0, 0, 1, 0], 
    [0, 0, 0, 1]])

voxel_size = 0.005  # Adjust based on your point cloud resolution

template_copy.voxel_down_sample(voxel_size)
template_copy.transform(initial_transform)


template_transformed_np = np.asarray(template_copy.points)
template_copy.write_point_cloud()

matrix_inv = np.linalg.inv(initial_transform)
template_copy.transform(matrix_inv)

o3d.visualization.draw_geometries([template, template_copy])
