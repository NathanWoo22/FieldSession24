import open3d as o3d
import numpy as np
import copy
import icp_registration as icp_reg


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def feature_matching(template_np, source_np):

    template = o3d.geometry.PointCloud()
    scene = o3d.geometry.PointCloud()
    template.points = o3d.utility.Vector3dVector(template_np)
    scene.points = o3d.utility.Vector3dVector(source_np)

    # scene = source_np
    # template = template_np
    template_original = copy.deepcopy(template)
    scene_original = copy.deepcopy(scene)

    voxel_size = 0.05

    # Down Sampling
    scene = scene.voxel_down_sample(voxel_size)
    template = template.voxel_down_sample(voxel_size)

    # Finding normals
    radius_normal = voxel_size * 1.5
    scene.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=10))
    template.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=10))

    # Finding features
    radius_feature = voxel_size * 5
    scene_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        scene, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
    )
    template_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        template, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
    )

    distance_threshold = 1.5 * voxel_size
 
    # RANSAC Registration for coarse alignment
    print("running ransac")
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
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 100)
    )
    # Print results
    print("RANSAC Transformation:\n", ransac_result.transformation)
    correspondences = ransac_result.correspondence_set
    print("Number of correspondences:", len(correspondences))

    # Transform the gas tank template
    template.transform(ransac_result.transformation)

    # Revert back to np array (not needed but have to change icp_reg)
    template_np = np.asarray(template.points)
    scene_np = np.asarray(scene_original.points)

    return(ransac_result.transformation, template)

def main(args=None):
    template = np.load("./datasets/gas_tank26.npy")
    scene = np.load("./datasets/big_scene26.npy")
    feature_matching(template, scene)

if __name__ == '__main__':
    main()