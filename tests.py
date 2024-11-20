import unittest
import time
import numpy as np
import generate_tests as rot_test
import icp_algo as icp
import icp_registration as icp_reg
import feature_matches as icp_col

# Example function to test
def add(a, b):
    return a + b

class TestMathOperations(unittest.TestCase):
    # Test cases look something like this: 

    # difference = np.linalg.norm(matrix1 - matrix2, ord='fro')

    # # Calculate threshold as 0.01% of the Frobenius norm of one of the matrices (e.g., matrix1)
    # threshold = 0.0001 * np.linalg.norm(matrix1, ord='fro')

    # # Check if the difference is within the threshold
    # if difference < threshold:

    def test_rotation(self):
        template_xyz, data_xyz, matrix = rot_test.generate_simple_rotation_test()

        start_time = time.perf_counter
        transformation, transformed_xyz = icp_col.icp(template_xyz, data_xyz)
        end_time = time.perf_counter

        difference = np.linalg.norm(matrix - transformation, ord='fro')
        threshold = 0.0001 * np.linalg.norm(matrix, ord='fro')

        self.assertGreater(threshold, difference)
        self.assertGreater(1.0, end_time-start_time, f"Expected time taken to be less than 1.0 secs, but got {end_time-start_time} secs")

    def test_translation(self):
        template_xyz, data_xyz, matrix = rot_test.generate_simple_translation_test()

        start_time = time.perf_counter
        transformation, transformed_xyz = icp_col.icp(template_xyz, data_xyz)
        end_time = time.perf_counter

        difference = np.linalg.norm(matrix - transformation, ord='fro')
        threshold = 0.0001 * np.linalg.norm(matrix, ord='fro')

        self.assertGreater(threshold, difference)
        self.assertGreater(1.0, end_time-start_time, f"Expected time taken to be less than 1.0 secs, but got {end_time-start_time} secs")

    def test_translation_rotation(self):
        template_xyz, data_xyz, matrix = rot_test.generate_simple_translation_rotation_test()

        start_time = time.perf_counter
        transformation, transformed_xyz = icp_col.icp(template_xyz, data_xyz)
        end_time = time.perf_counter

        difference = np.linalg.norm(matrix - transformation, ord='fro')
        threshold = 0.0001 * np.linalg.norm(matrix, ord='fro')

        self.assertGreater(threshold, difference)
        self.assertGreater(1.0, end_time-start_time, f"Expected time taken to be less than 1.0 secs, but got {end_time-start_time} secs")

if __name__ == '__main__':
    unittest.main()

