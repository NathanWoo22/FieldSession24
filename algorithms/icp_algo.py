import numpy as np
from scipy.spatial import KDTree

def icp(template_cloud, larger_cloud, max_iterations=100, tolerance=0.01):
    
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
        # Find closest points
        closest_points, indices = find_closest_points(template_cloud, target_cloud)
        normals = target_normals[indices]
        
        # Compute the incremental transformation
        delta_transform = compute_point_to_plane_transformation(template_cloud, closest_points, normals)
        
        # Update the overall transformation
        transformation = delta_transform @ transformation
        
        # Apply the transformation to the template cloud
        ones_column = np.ones((template_cloud.shape[0], 1))
        template_homogeneous = np.hstack((template_cloud, ones_column))
        template_cloud = (delta_transform @ template_homogeneous.T).T[:, :3]
        
        # Compute error
        error = compute_point_to_plane_error(template_cloud, closest_points, normals)
        print(f"Iteration {i}, Error: {error}")
        
        if np.abs(prev_error - error) < tolerance:
            print(f"iterations: {i}, {prev_error - error}")
            break
        prev_error = error
    
    return transformation, template_cloud

def find_closest_points(source, target):
    kdtree = KDTree(target)
    distances, indices = kdtree.query(source)
    return target[indices], indices

def compute_point_to_plane_transformation(source_points, target_points, normals):
    # Build the system of equations
    A = []
    b = []
    for (p_s, p_t, n) in zip(source_points, target_points, normals):
        n = n / np.linalg.norm(n)  # Normalize the normal vector
        # Construct the A matrix and b vector for each correspondence
        A_i = np.concatenate((np.cross(p_s, n), n))
        A.append(A_i)
        b_i = n @ (p_t - p_s)
        b.append(b_i)
    
    A = np.array(A)
    b = np.array(b).reshape(-1, 1)
    
    # Solve for the transformation parameters
    x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
    # x = [alpha, beta, gamma, t_x, t_y, t_z]
    # Construct the incremental transformation matrix
    omega = x[:3].flatten()
    theta = np.linalg.norm(omega)
    if theta > 1e-12:
        k = omega / theta
        K = np.array([
            [0, -k[2], k[1]],
            [k[2], 0, -k[0]],
            [-k[1], k[0], 0]
        ])
        R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
    else:
        R = np.eye(3)
    t = x[3:].flatten()
    delta_transform = np.eye(4)
    delta_transform[:3, :3] = R
    delta_transform[:3, 3] = t
    return delta_transform

def compute_point_to_plane_error(source_points, target_points, normals):
    errors = []
    for (p_s, p_t, n) in zip(source_points, target_points, normals):
        n = n / np.linalg.norm(n)  # Normalize the normal vector
        errors.append((n @ (p_s - p_t)) ** 2)
    return np.mean(errors)
