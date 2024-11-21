import numpy as np
import open3d as o3d
import feature_matches as icp_col
import icp_registration as icp_reg

template_file = np.load("./datasets/fiducial_plane.npz")
template = template_file['point_cloud']
cap_np = template[:, :3]
tank_np = np.load("./datasets/gas_tank.npy")

# GOOD GOOD
"""[[-0.03183934 -0.9990636   0.02929486  0.08033926]
 [ 0.99828519 -0.03322768 -0.04819339  1.46561941]
 [ 0.04912167  0.02771017  0.99840834  0.1827135 ]
 [ 0.          0.          0.          1.        ]]"""

initial_transform = np.array([
    [0, -1, 0, 0.05],  
    [1, 0, 0, 1.4],  
    [0, 0, 1, 0.22], 
    [0, 0, 0, 1]])

transformation = icp_reg.icp(cap_np, tank_np, initial_transform)

tank = o3d.geometry.PointCloud()
cap = o3d.geometry.PointCloud()
tank.points = o3d.utility.Vector3dVector(tank_np)
cap.points = o3d.utility.Vector3dVector(cap_np)

cap.transform(transformation)
o3d.visualization.draw_geometries([cap, tank])
print("here")
# cap_np = np.asarray(cap.points)
# np.save("./datasets/aligned_cap", cap_np)

