import numpy as np
from scipy.spatial import KDTree

def icp(template_cloud, larger_cloud, max_iterations=100, tolerance=1e-20):
    
    def find_closest_points(source, target):
        kdtree = KDTree(target)
        distances, indices = kdtree.query(source)
        return target[indices], indices

    def compute_transformation(source, target):
        source_centroid = np.mean(source, axis=0)
        target_centroid = np.mean(target, axis=0)
        source_centered = source - source_centroid
        target_centered = target - target_centroid

        # H = np.dot(source_centered.T, target_centered)
        # U, _, Vt = np.linalg.svd(H)
        # R = np.dot(Vt.T, U.T)

        # print(R)
        # print(U)
        #Perform optimization of the error (straight from wikipedia )
        H = np.dot(source_centered.T, target_centered)
        U, _, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)
        
        # Compute the translation vector
        t = target_centroid - np.dot(R, source_centroid)
        transformation = np.eye(4)
        transformation[:3, :3] = R
        transformation[:3, 3] = t
        
        return transformation

    # transformation = np.eye(4)
    transformation = np.array([
    [0, -1, 0, 0.05],  
    [1, 0, 0, 1.4],  
    [0, 0, 1, 0.22], 
    [0, 0, 0, 1]])
    prev_error = float('inf')
    
    for i in range(max_iterations):
        closest_points, _ = find_closest_points(template_cloud, larger_cloud)
        current_transformation = compute_transformation(template_cloud, closest_points)
        ones_column = np.ones((template_cloud.shape[0], 1))  
        print(ones_column)
        template_homogeneous = np.hstack((template_cloud, ones_column)) 
        template_cloud = (current_transformation @ template_homogeneous.T).T[:, :3]  
        
        transformation = current_transformation @ transformation
        
        error = np.mean(np.linalg.norm(template_cloud - closest_points, axis=1)**2)
        
        if np.abs(prev_error - error) < tolerance:
            print(f"iterations: {i}")
            break
        prev_error = error
        print(transformation)

    return transformation, template_cloud
