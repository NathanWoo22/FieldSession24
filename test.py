import numpy as np
import icp_algo as icp

# Example point clouds (Nx3 numpy arrays)
template_cloud = np.random.rand(10, 3)  # Smaller template point cloud
larger_cloud = np.random.rand(100, 3)   # Larger point cloud

# Run ICP algorithm
transformation, aligned_template = icp.icp(template_cloud, larger_cloud)

print("Final Transformation Matrix:")
print(transformation)