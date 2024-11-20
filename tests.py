import unittest
import time
import numpy as np
import open3d as o3d
import icp_algo as icp
import icp_registration as icp_reg
import feature_matches as icp_col


class TestMathOperations(unittest.TestCase):
    # Test cases look something like this: 

    # difference = np.linalg.norm(matrix1 - matrix2, ord='fro')

    # # Calculate threshold as 0.01% of the Frobenius norm of one of the matrices (e.g., matrix1)
    # threshold = 0.0001 * np.linalg.norm(matrix1, ord='fro')

    # # Check if the difference is within the threshold
    # if difference < threshold:

    def test_rotation(self):
        template = o3d.io.read_point_cloud("datasets/fiducial_plane.pcd") 
        data = o3d.io.read_point_cloud("tests/easy/rotation_easy.pcd") 
        matrix = np.loadtxt("tests/easy/rotation_easy_gt.txt")

        template_points = np.asarray(template.points)
        data_points = np.asarray(data.points)

        template_xyz = template_points[:, :3]
        data_xyz = data_points[:, :3]

        start_time = time.perf_counter
        transformation, transformed_xyz = icp_col.icp(template_xyz, data_xyz)
        end_time = time.perf_counter

        difference = np.linalg.norm(matrix - transformation, ord='fro')
        threshold = 0.1 * np.linalg.norm(matrix, ord='fro')

        template.paint_uniform_color([1, 0, 0])  # Red
        data.paint_uniform_color([0, 1, 0]) # Green

        o3d.visualization.draw_geometries([template, data])

        transformed_template = template.transform(transformation)

        transformed_template.paint_uniform_color([1, 0, 0])  # Red
        data.paint_uniform_color([0, 1, 0]) # Green

        o3d.visualization.draw_geometries([transformed_template, data])

        print("Final Transformation Matrix:")
        print(transformation)

        self.assertGreater(threshold, difference)
        # self.assertGreater(1.0, end_time-start_time, f"Expected time taken to be less than 1.0 secs, but got {end_time-start_time} secs")

    # def test_translation(self):
    #     template_xyz, data_xyz, matrix = rot_test.generate_simple_translation_test()

    #     start_time = time.perf_counter
    #     transformation, transformed_xyz = icp_col.icp(template_xyz, data_xyz)
    #     end_time = time.perf_counter

    #     difference = np.linalg.norm(matrix - transformation, ord='fro')
    #     threshold = 0.0001 * np.linalg.norm(matrix, ord='fro')

    #     self.assertGreater(threshold, difference)
    #     self.assertGreater(1.0, end_time-start_time, f"Expected time taken to be less than 1.0 secs, but got {end_time-start_time} secs")

    # def test_translation_rotation(self):
    #     template_xyz, data_xyz, matrix = rot_test.generate_simple_translation_rotation_test()

    #     start_time = time.perf_counter
    #     transformation, transformed_xyz = icp_col.icp(template_xyz, data_xyz)
    #     end_time = time.perf_counter

    #     difference = np.linalg.norm(matrix - transformation, ord='fro')
    #     threshold = 0.0001 * np.linalg.norm(matrix, ord='fro')

    #     self.assertGreater(threshold, difference)
    #     self.assertGreater(1.0, end_time-start_time, f"Expected time taken to be less than 1.0 secs, but got {end_time-start_time} secs")

if __name__ == '__main__':
    unittest.main()
