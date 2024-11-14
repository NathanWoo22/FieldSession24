import open3d as o3d
import numpy as np
import copy


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    # o3d.visualization.draw_geometries([source_temp, target_temp])
    o3d.visualization.draw_geometries([target_temp, source_temp])

def icp(template_np, source_np):

    template = o3d.geometry.PointCloud()
    scene = o3d.geometry.PointCloud()
    template.points = o3d.utility.Vector3dVector(template_np)
    scene.points = o3d.utility.Vector3dVector(source_np)
    voxel_size = 0.01  # Adjust based on your point cloud resolution

    scene = scene.voxel_down_sample(voxel_size)
    template = template.voxel_down_sample(voxel_size)

    scene.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=10))
    template.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=10))
    radius_feature = voxel_size * 5
    scene_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        scene, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
    )
    template_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        template, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
    )

    # RANSAC Registration for coarse alignment
    distance_threshold = 0.4
    ransac_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        template, scene, template_fpfh, scene_fpfh,
        mutual_filter=False,
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)
    )

    
    print("RANSAC Transformation:\n", ransac_result.transformation)
    correspondences = ransac_result.correspondence_set
    print("Number of correspondences:", len(correspondences))
    # ICP for fine alignment
    icp_result = o3d.pipelines.registration.registration_icp(
        template, scene, distance_threshold, ransac_result.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )

    print("ICP Transformation:\n", icp_result.transformation)
    # Apply the final transformation to the template for visualization
    template.transform(icp_result.transformation)
    o3d.visualization.draw_geometries([scene, template])
    print(icp_result)
    print("Transformation is:")
    print(icp_result.transformation)
    print("")
    transformed_template_np = np.asarray(template.points)
    return icp_result.transformation, transformed_template_np