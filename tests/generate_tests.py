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
# Define wall dimensions and density
width = 5.0        # Wall width in x-axis
height = 3.0       # Wall height in y-axis
density = 100      # Points per unit length (density)

# Generate a grid of points
x = np.linspace(0, width, int(width * density))  # x-coordinates
y = np.linspace(0, height, int(height * density))  # y-coordinates
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

# Create a tranlation array
translation_vector = np.array([0, 1.01, 0])

# Apply translation to the points
translated_points = rotated_wall + translation_vector
'''

initial_transform = np.array([
    [0, 1, 0, 1],  
    [1, 0, 0, 1],  
    [0, 0, 1, 1], 
    [0, 0, 0, 1]
])

template = o3d.io.read_point_cloud("datasets/fiducial_plane.pcd") 

template_copy = copy.deepcopy(template)

initial_transform = np.array([
    [1, 0, 0, 0],  
    [0, 1, 0, 0],  
    [0, 0, 1, 0], 
    [0, 0, 0, 1]
])

template_copy.transform(initial_transform)

# Parameters for the curved wall
radius = 1.0       # Radius of the curved wall
angle = np.pi      # Curve spans 90 degrees (in radians)
height = 2.0       # Height of the wall
density = 60      # Points per unit length

# Step 1: Generate a grid of points in angular and vertical directions
theta = np.linspace(0, angle, int(density * angle))  # Angular points
z = np.linspace(0, height, int(density * height))    # Vertical points
theta_grid, z_grid = np.meshgrid(theta, z)

# Step 2: Convert cylindrical coordinates to Cartesian coordinates
x = radius * np.cos(theta_grid)  # x = r * cos(theta)
y = radius * np.sin(theta_grid)  # y = r * sin(theta)
z = z_grid                       # z remains as height

# Combine points into an (N, 3) array
wall = np.vstack((x.ravel(), y.ravel(), z.ravel())).T

# Create a tranlation array
translation_vector = np.array([0, -2.02, -1])

# Apply translation to the points
translated_points = wall + translation_vector

points = np.asarray(template_copy.points)
combined_points = np.vstack((points, translated_points))

noisy_pcd = o3d.geometry.PointCloud()
noisy_pcd.points = o3d.utility.Vector3dVector(combined_points)

matrix_inv = np.linalg.inv(initial_transform)

o3d.io.write_point_cloud("template.pcd", template_copy, write_ascii=True)
# np.savetxt("tests/easy/nothing_done_gt.txt", matrix_inv, fmt="%.6f")

template.paint_uniform_color([1, 0, 0])  # Red
noisy_pcd.paint_uniform_color([0, 1, 0]) # Green
o3d.visualization.draw_geometries([template, noisy_pcd])