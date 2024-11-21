import open3d as o3d
import numpy as np
import copy

# Hardcoded init transform if needed
# initial_transform = np.array([
# [0, -1, 0, 0.05],  
# [1, 0, 0, 1.4],  
# [0, 0, 1, 0.22], 
# [0, 0, 0, 1]])

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([target_temp, source_temp])

def icp(template_np, source_np, initial_transform):
    source = o3d.geometry.PointCloud()
    target = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(source_np)
    target.points = o3d.utility.Vector3dVector(template_np)
    # source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
    target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))

    threshold = 1

    # Perform ICP Point-to-Point registration   
    print("Apply point-to-point ICP")
    reg_p2p = o3d.pipelines.registration.registration_icp(
        target, source, threshold, initial_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    print("")
    draw_registration_result(target, source, reg_p2p.transformation)
    target.transform(reg_p2p.transformation)
    transformed_template_np = np.asarray(target.points)
    return reg_p2p.transformation # change to below when reverted to ros
    # return reg_p2p.transformation, transformed_template_np

    # Perform ICP Point-to-Plane registration 
    # print("Apply point-to-plane ICP")
    # reg_p2l = o3d.pipelines.registration.registration_icp(
    #     source, target, threshold, initial_transform,
    #     o3d.pipelines.registration.TransformationEstimationPointToPlane())
    # print(reg_p2l)
    # print("Transformation is:")
    # print(reg_p2l.transformation)
    # print("")
    # draw_registration_result(source, target, reg_p2l.transformation)
    # source.transform(reg_p2l.transformation)
    # transformed_template_np = np.asarray(source.points)
    # return reg_p2l.transformation, transformed_template_np