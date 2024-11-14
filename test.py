import pcl

# Create a point cloud object
cloud = pcl.PointCloud()

# Load the point cloud from a file (e.g., in PCD format)
cloud.from_file("datasets/fiducial_plane.pcd")

# Estimate normals for the point cloud
ne = pcl.NormalEstimation()
ne.set_InputCloud(cloud)
ne.set_KSearch(10)  # Number of neighbors to use for normal estimation
normals = ne.compute()

# Create an FPFH estimation object
fpfh = pcl.FPFHEstimation()
fpfh.set_InputCloud(cloud)
fpfh.set_InputNormals(normals)
fpfh.set_RadiusSearch(0.05)  # Search radius for FPFH computation
fpfhs = fpfh.compute()

# Access the FPFH features
print(fpfhs.size)
print(fpfhs[0])