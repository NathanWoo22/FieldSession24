import numpy as np

def read_pcd_to_numpy(file_path):

    data_start_index = 11
    point_data = []

    with open(file_path, 'r') as file:
        for i, line in enumerate(file):
            if i >= data_start_index:
                # Split each line into 7 float values and append to the list
                point_data.append(list(map(float, line.split())))
    
    # Convert the list of points into a numpy array
    point_cloud = np.array(point_data)
    
    return point_cloud

# Path to your PCD file
file_path = 'datasets/fiducial_plane.pcd'

# Call the function and store the point cloud data
point_cloud_array = read_pcd_to_numpy(file_path)

# Verify the shape and some points (optional)
print("Shape of point cloud array:", point_cloud_array.shape)
print("First 5 points:\n", point_cloud_array[:5])


np.savez("./datasets/fiducial_plane.npz", point_cloud=point_cloud_array)