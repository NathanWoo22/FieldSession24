import pcl

# Create a point cloud object
target = pcl.PointCloud()
target.from_file("datasets/fiducial_plane.pcd")

source = pcl.PointCloud()
source.from_file("tests/rotation_easy.pcd")

#Test each algorithm 