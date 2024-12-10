import unittest
import time
import numpy as np
import open3d as o3d

import algorithms.icp_colored as icp_col
import algorithms.icp_registration as icp_reg
import algorithms.feature_matches as feature_match
import algorithms.icp_algo as icp_algo



class TestMatchingAlgorithms(unittest.TestCase):

    def test_given_answer(self):
        #Load in data from datasets into test
        template = o3d.io.read_point_cloud("tests/template.pcd") 
        data = o3d.io.read_point_cloud("tests/easy/nothing_done.pcd") 
        matrix = np.loadtxt("tests/easy/nothing_done_gt.txt")

        #Turn data into numpy arrays
        template_points = np.asarray(template.points)
        data_points = np.asarray(data.points)

        #Turn numpy arrays into regular array structure
        template_xyz = template_points[:, :3]
        data_xyz = data_points[:, :3]

        #Perform the transformation and store the before and after times
        #You can change the algorithm creating the transformation to test different algorithms
        start_time = time.perf_counter()
        transformation, transformed_xyz = icp_col.icp(template_xyz, data_xyz)
        end_time = time.perf_counter()

        #Create a difference between the transformations and create a threshold to measure that distance
        difference = np.linalg.norm(matrix - transformation, ord='fro')
        threshold = 1 * np.linalg.norm(matrix, ord='fro')

        #Compare the results for both accuracy and timing
        self.assertGreater(threshold, difference)
        self.assertGreater(1.0, end_time-start_time, f"Expected time taken to be less than 1.0 secs, but got {end_time-start_time} secs")

    def test_rotation(self):
        #Load in data from datasets into test
        template = o3d.io.read_point_cloud("tests/template.pcd") 
        data = o3d.io.read_point_cloud("tests/easy/rotation.pcd") 
        matrix = np.loadtxt("tests/easy/rotation_gt.txt")

        #Turn data into numpy arrays
        template_points = np.asarray(template.points)
        data_points = np.asarray(data.points)

        #Turn numpy arrays into regular array structure
        template_xyz = template_points[:, :3]
        data_xyz = data_points[:, :3]

        #Perform the transformation and store the before and after times
        #You can change the algorithm creating the transformation to test different algorithms
        start_time = time.perf_counter()
        transformation, transformed_xyz = icp_col.icp(template_xyz, data_xyz)
        end_time = time.perf_counter()

        #Create a difference between the transformations and create a threshold to measure that distance
        difference = np.linalg.norm(matrix - transformation, ord='fro')
        threshold = 0.1 * np.linalg.norm(matrix, ord='fro')

        self.assertGreater(threshold, difference)
        self.assertGreater(1.0, end_time-start_time, f"Expected time taken to be less than 1.0 secs, but got {end_time-start_time} secs")

    def test_translation(self):
        #Load in data from datasets into test
        template = o3d.io.read_point_cloud("tests/template.pcd") 
        data = o3d.io.read_point_cloud("tests/easy/translation.pcd") 
        matrix = np.loadtxt("tests/easy/translation_gt.txt")

        #Turn data into numpy arrays
        template_points = np.asarray(template.points)
        data_points = np.asarray(data.points)

        #Turn numpy arrays into regular array structure
        template_xyz = template_points[:, :3]
        data_xyz = data_points[:, :3]

        #Perform the transformation and store the before and after times
        #You can change the algorithm creating the transformation to test different algorithms
        start_time = time.perf_counter()
        transformation, transformed_xyz = icp_col.icp(template_xyz, data_xyz)
        end_time = time.perf_counter()

        #Create a difference between the transformations and create a threshold to measure that distance
        difference = np.linalg.norm(matrix - transformation, ord='fro')
        threshold = 0.1 * np.linalg.norm(matrix, ord='fro')

        #Compare the results for both accuracy and timing
        self.assertGreater(threshold, difference)
        self.assertGreater(1.0, end_time-start_time, f"Expected time taken to be less than 1.0 secs, but got {end_time-start_time} secs")
    
    def test_rotation_and_translation(self):
        #Load in data from datasets into test
        template = o3d.io.read_point_cloud("tests/template.pcd") 
        data = o3d.io.read_point_cloud("tests/easy/rotation_and_translation.pcd") 
        matrix = np.loadtxt("tests/easy/rotation_and_translation_gt.txt")

        #Turn data into numpy arrays
        template_points = np.asarray(template.points)
        data_points = np.asarray(data.points)

        #Turn numpy arrays into regular array structure
        template_xyz = template_points[:, :3]
        data_xyz = data_points[:, :3]

        #Perform the transformation and store the before and after times
        #You can change the algorithm creating the transformation to test different algorithms
        start_time = time.perf_counter()
        transformation, transformed_xyz = icp_col.icp(template_xyz, data_xyz)
        end_time = time.perf_counter()

        #Create a difference between the transformations and create a threshold to measure that distance
        difference = np.linalg.norm(matrix - transformation, ord='fro')
        threshold = 0.1 * np.linalg.norm(matrix, ord='fro')

        #Compare the results for both accuracy and timing
        self.assertGreater(threshold, difference)
        self.assertGreater(1.0, end_time-start_time, f"Expected time taken to be less than 1.0 secs, but got {end_time-start_time} secs")

    def test_rotation_with_noise(self):
        #Load in data from datasets into test
        template = o3d.io.read_point_cloud("tests/template.pcd") 
        data = o3d.io.read_point_cloud("tests/medium/rotation_with_noise.pcd") 
        matrix = np.loadtxt("tests/medium/rotation_with_noise_gt.txt")

        #Turn data into numpy arrays
        template_points = np.asarray(template.points)
        data_points = np.asarray(data.points)

        #Turn numpy arrays into regular array structure
        template_xyz = template_points[:, :3]
        data_xyz = data_points[:, :3]

        #Perform the transformation and store the before and after times
        #You can change the algorithm creating the transformation to test different algorithms
        start_time = time.perf_counter()
        transformation, transformed_xyz = icp_col.icp(template_xyz, data_xyz)
        end_time = time.perf_counter()

        #Create a difference between the transformations and create a threshold to measure that distance
        difference = np.linalg.norm(matrix - transformation, ord='fro')
        threshold = 0.1 * np.linalg.norm(matrix, ord='fro')

        #Compare the results for both accuracy and timing
        self.assertGreater(threshold, difference)
        self.assertGreater(1.0, end_time-start_time, f"Expected time taken to be less than 1.0 secs, but got {end_time-start_time} secs")

    def test_translation_with_noise(self):
        #Load in data from datasets into test
        template = o3d.io.read_point_cloud("tests/template.pcd") 
        data = o3d.io.read_point_cloud("tests/medium/translation_with_noise.pcd") 
        matrix = np.loadtxt("tests/medium/translation_with_noise_gt.txt")

        #Turn data into numpy arrays
        template_points = np.asarray(template.points)
        data_points = np.asarray(data.points)

        #Turn numpy arrays into regular array structure
        template_xyz = template_points[:, :3]
        data_xyz = data_points[:, :3]

        #Perform the transformation and store the before and after times
        #You can change the algorithm creating the transformation to test different algorithms
        start_time = time.perf_counter()
        transformation, transformed_xyz = icp_col.icp(template_xyz, data_xyz)
        end_time = time.perf_counter()

        #Create a difference between the transformations and create a threshold to measure that distance
        difference = np.linalg.norm(matrix - transformation, ord='fro')
        threshold = 0.1 * np.linalg.norm(matrix, ord='fro')

        #Compare the results for both accuracy and timing
        self.assertGreater(threshold, difference)
        self.assertGreater(1.0, end_time-start_time, f"Expected time taken to be less than 1.0 secs, but got {end_time-start_time} secs")

    def test_rotation_and_translation_with_noise(self):
        #Load in data from datasets into test
        template = o3d.io.read_point_cloud("tests/template.pcd") 
        data = o3d.io.read_point_cloud("tests/medium/rotation_and_translation_with_noise.pcd") 
        matrix = np.loadtxt("tests/medium/rotation_and_translation_with_noise_gt.txt")

        #Turn data into numpy arrays
        template_points = np.asarray(template.points)
        data_points = np.asarray(data.points)

        #Turn numpy arrays into regular array structure
        template_xyz = template_points[:, :3]
        data_xyz = data_points[:, :3]

        #Perform the transformation and store the before and after times
        #You can change the algorithm creating the transformation to test different algorithms
        start_time = time.perf_counter()
        transformation, transformed_xyz = icp_col.icp(template_xyz, data_xyz)
        end_time = time.perf_counter()

        #Create a difference between the transformations and create a threshold to measure that distance
        difference = np.linalg.norm(matrix - transformation, ord='fro')
        threshold = 0.1 * np.linalg.norm(matrix, ord='fro')

        #Compare the results for both accuracy and timing
        self.assertGreater(threshold, difference)
        self.assertGreater(1.0, end_time-start_time, f"Expected time taken to be less than 1.0 secs, but got {end_time-start_time} secs")

    def test_rotation_and_translation_with_wall(self):
        #Load in data from datasets into test
        template = o3d.io.read_point_cloud("tests/template.pcd") 
        data = o3d.io.read_point_cloud("tests/hard/rotation_and_translation_with_wall.pcd") 
        matrix = np.loadtxt("tests/hard/rotation_and_translation_with_wall_gt.txt")

        #Turn data into numpy arrays
        template_points = np.asarray(template.points)
        data_points = np.asarray(data.points)

        #Turn numpy arrays into regular array structure
        template_xyz = template_points[:, :3]
        data_xyz = data_points[:, :3]

        #Perform the transformation and store the before and after times
        #You can change the algorithm creating the transformation to test different algorithms
        start_time = time.perf_counter()
        transformation, transformed_xyz = icp_col.icp(template_xyz, data_xyz)
        end_time = time.perf_counter()

        #Create a difference between the transformations and create a threshold to measure that distance
        difference = np.linalg.norm(matrix - transformation, ord='fro')
        threshold = 0.1 * np.linalg.norm(matrix, ord='fro')

        #Compare the results for both accuracy and timing
        self.assertGreater(threshold, difference)
        self.assertGreater(1.0, end_time-start_time, f"Expected time taken to be less than 1.0 secs, but got {end_time-start_time} secs")

    def test_rotation_and_translation_with_curved_wall(self):
        #Load in data from datasets into test
        template = o3d.io.read_point_cloud("tests/template.pcd") 
        data = o3d.io.read_point_cloud("tests/hard/rotation_and_translation_with_curved_wall.pcd") 
        matrix = np.loadtxt("tests/hard/rotation_and_translation_with_curved_wall_gt.txt")

        #Turn data into numpy arrays
        template_points = np.asarray(template.points)
        data_points = np.asarray(data.points)

        #Turn numpy arrays into regular array structure
        template_xyz = template_points[:, :3]
        data_xyz = data_points[:, :3]

        #Perform the transformation and store the before and after times
        #You can change the algorithm creating the transformation to test different algorithms
        start_time = time.perf_counter()
        transformation, transformed_xyz = icp_col.icp(template_xyz, data_xyz)
        end_time = time.perf_counter()

        #Create a difference between the transformations and create a threshold to measure that distance
        difference = np.linalg.norm(matrix - transformation, ord='fro')
        threshold = 0.1 * np.linalg.norm(matrix, ord='fro')

        #Compare the results for both accuracy and timing
        self.assertGreater(threshold, difference)
        self.assertGreater(1.0, end_time-start_time, f"Expected time taken to be less than 1.0 secs, but got {end_time-start_time} secs")


if __name__ == '__main__':
    unittest.main()
