import unittest

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
    
    def test_add(self):
        self.assertEqual(add(2, 3), 5)
        self.assertEqual(add(-1, 1), 0)
        self.assertEqual(add(0, 0), 0)


if __name__ == '__main__':
    unittest.main()

