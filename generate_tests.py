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

''' create random noise
# Define the mean location for the noise
mean_location = [-1, 1, -1]  # Noise centered at (x=2, y=3, z=-1)
std_dev = 0.1                     # Spread of the noise
num_points = 1000                 # Number of noise points

# Generate 3D noise around the mean location
noise = np.random.normal(mean_location, std_dev, (num_points, 3))
'''

''' create wall of noise
# Step 1: Define wall dimensions and density
width = 5.0        # Wall width in x-axis
height = 3.0       # Wall height in y-axis
density = 100      # Points per unit length (density)

# Step 2: Generate a grid of points
x = np.linspace(0, width, int(width * density))  # x-coordinates
y = np.linspace(0, height, int(height * density))  # y-coordinates
xx, yy = np.meshgrid(x, y)  # Create a grid
z = np.zeros_like(xx)  # z-coordinate for a flat wall

# Combine into (N, 3) point cloud
points = np.vstack((xx.ravel(), yy.ravel(), z.ravel())).T
'''

template = o3d.io.read_point_cloud("datasets/fiducial_plane.pcd") 

template_copy = copy.deepcopy(template)

initial_transform = np.array([
    [0, 1, 0, 0],  
    [1, 0, 0, 1],  
    [0, 0, 1, 0], 
    [0, 0, 0, 1]
])

template_copy.transform(initial_transform)

# Define wall dimensions and density
width = 2.0        # Wall width in x-axis
height = 2.0       # Wall height in y-axis
density = 50      # Points per unit length (density)

# Generate a grid of points
x = np.linspace(-1, 1, int(width * density))  # x-coordinates
y = np.linspace(-1, 1, int(height * density))  # y-coordinates
xx, yy = np.meshgrid(x, y)  # Create a grid
z = np.zeros_like(xx)  # z-coordinate for a flat wall

# Combine into (N, 3) point cloud
wall = np.vstack((xx.ravel(), yy.ravel(), z.ravel())).T

# Create a rotation matrix
theta = np.pi / 2  # 90 degrees
rotation_matrix = np.array([
    [1, 0, 0],
    [0, np.cos(theta), -np.sin(theta)],
    [0, np.sin(theta), np.cos(theta)]
])

# Apply rotation to the points
rotated_wall = wall @ rotation_matrix.T

translation_vector = np.array([0, 1.0, 0])
translated_points = rotated_wall + translation_vector

points = np.asarray(template_copy.points)
combined_points = np.vstack((points, translated_points))

noisy_pcd = o3d.geometry.PointCloud()
noisy_pcd.points = o3d.utility.Vector3dVector(combined_points)

matrix_inv = np.linalg.inv(initial_transform)

# o3d.io.write_point_cloud("tests/hard/rotation_and_translation_med.pcd", noisy_pcd, write_ascii=True)
# np.savetxt("tests/hard/rotation_and_translation_med_gt.txt", matrix_inv, fmt="%.6f")

template.paint_uniform_color([1, 0, 0])  # Red
noisy_pcd.paint_uniform_color([0, 1, 0]) # Green
o3d.visualization.draw_geometries([template, noisy_pcd])