import numpy as np
from scipy.spatial import KDTree

def icp(template_cloud, larger_cloud, max_iterations=100, tolerance=1e-20):
    
    def find_closest_points(source, target):
        kdtree = KDTree(target)
        distances, indices = kdtree.query(source)
        return target[indices], indices

    def compute_transformation(source, target):
        # Compute the centroids of both point clouds
        source_centroid = np.mean(source, axis=0)
        target_centroid = np.mean(target, axis=0)
        
        # Center the clouds around their centroids
        source_centered = source - source_centroid
        target_centered = target - target_centroid
        
        # Compute the cross-covariance matrix
        H = np.dot(source_centered.T, target_centered)
        
        # Perform Singular Value Decomposition (SVD)
        U, _, Vt = np.linalg.svd(H)
        
        # Compute the rotation matrix
        R = np.dot(Vt.T, U.T)
        
        # Ensure a proper rotation (det(R) = 1)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)
        
        # Compute the translation vector
        t = target_centroid - np.dot(R, source_centroid)
        
        # Form the transformation matrix
        transformation = np.eye(4)
        transformation[:3, :3] = R
        transformation[:3, 3] = t
        
        return transformation

    # Initialize transformation as identity matrix
    transformation = np.eye(4)
    
    # Iterative ICP process
    prev_error = float('inf')
    
    for i in range(max_iterations):
        closest_points, _ = find_closest_points(template_cloud, larger_cloud)
        
        current_transformation = compute_transformation(template_cloud, closest_points)
        
        ones_column = np.ones((template_cloud.shape[0], 1))  # Add a column of ones for homogeneous coordinates
        template_homogeneous = np.hstack((template_cloud, ones_column))  # Nx4 matrix
        template_cloud = (current_transformation @ template_homogeneous.T).T[:, :3]  # Apply transformation
        
        transformation = current_transformation @ transformation
        
        error = np.mean(np.linalg.norm(template_cloud - closest_points, axis=1)**2)
        
        if np.abs(prev_error - error) < tolerance:
            print(f"iterations: {i}")
            break
        prev_error = error
        print(transformation)

    return transformation, template_cloud
