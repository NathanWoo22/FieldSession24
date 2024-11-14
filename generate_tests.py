import numpy as np
import open3d as o3d
import copy

# template_file = np.load("./datasets/fiducial_plane.npz")

# template_npz = template_file['point_cloud']
# template_npz = template_npz[:, :3]
                                 
# template = o3d.geometry.PointCloud()
# template.points = o3d.utility.Vector3dVector(template_npz)

# voxel_size = 0.005  # Adjust based on your point cloud resolution

# template_copy.voxel_down_sample(voxel_size)

def generate_simple_rotation_test():

    template = o3d.io.read_point_cloud("datasets/fiducial_plane.pcd") 

    template_copy = copy.deepcopy(template)

    initial_transform = np.array([
        [0, 1, 0, 0],  
        [1, 0, 0, 0],  
        [0, 0, 1, 0], 
        [0, 0, 0, 1]])

    template_copy.transform(initial_transform)

    template_transformed_np = np.asarray(template_copy.points)
    template_copy.write_point_cloud()

    data_xyz = template_copy[:, :3]
    template_xyz = template[:, :3]

    #matrix_inv = np.linalg.inv(initial_transform)

    o3d.visualization.draw_geometries([template, template_copy])

    return template_xyz, data_xyz, initial_transform

def generate_simple_translation_test():

    template = o3d.io.read_point_cloud("datasets/fiducial_plane.pcd") 

    template_copy = copy.deepcopy(template)

    initial_transform = np.array([
        [1, 0, 0, 1],  
        [0, 1, 0, 1],  
        [0, 0, 1, 1], 
        [0, 0, 0, 1]])

    template_copy.transform(initial_transform)

    template_transformed_np = np.asarray(template_copy.points)
    template_copy.write_point_cloud()

    data_xyz = template_copy[:, :3]
    template_xyz = template[:, :3]

    #matrix_inv = np.linalg.inv(initial_transform)

    o3d.visualization.draw_geometries([template, template_copy])

    return template_xyz, data_xyz, initial_transform

def generate_simple_translation_rotation_test():

    template = o3d.io.read_point_cloud("datasets/fiducial_plane.pcd") 

    template_copy = copy.deepcopy(template)

    initial_transform = np.array([
        [0, 1, 0, 1],  
        [1, 0, 0, 1],  
        [0, 0, 1, 1], 
        [0, 0, 0, 1]])

    template_copy.transform(initial_transform)

    template_transformed_np = np.asarray(template_copy.points)
    template_copy.write_point_cloud()

    data_xyz = template_copy[:, :3]
    template_xyz = template[:, :3]

    #matrix_inv = np.linalg.inv(initial_transform)

    o3d.visualization.draw_geometries([template, template_copy])

    return template_xyz, data_xyz, initial_transform

def generate_rotation_translation_with_noise_test():

    template = o3d.io.read_point_cloud("datasets/fiducial_plane.pcd") 

    template_copy = copy.deepcopy(template)

    initial_transform = np.array([
        [0, 1, 0, 0],  
        [1, 0, 0, 0],  
        [0, 0, 1, 0], 
        [0, 0, 0, 1]])

    template_copy.transform(initial_transform)

    points = np.asarray(template.points)
    noise = np.random.normal(0, noise_std_dev = 0.01, points.shape)
    noisy_points = points + noise

    template_transformed_np = np.asarray(template_copy.points)
    template_copy.write_point_cloud()

    noisy_pcd = o3d.geometry.PointCloud()
    noisy_pcd.points = o3d.utility.Vector3dVector(noisy_points)

    data_xyz = noisy_pcd[:, :3]
    template_xyz = template[:, :3]

    #matrix_inv = np.linalg.inv(initial_transform)

    o3d.visualization.draw_geometries([template, template_copy])

    return template_xyz, data_xyz, initial_transform