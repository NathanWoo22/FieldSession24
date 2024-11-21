import numpy as np
import open3d as o3d
import copy

def zfilter(array):
    fillArr = np.array([x for x in array if x[2] >= 0])
    fillArr = np.array([x for x in fillArr if x[2] <= 0.5])

    fillArr = np.array([x for x in fillArr if x[1] <= 1.7])
    fillArr = np.array([x for x in fillArr if x[1] >= 1.45])

    fillArr = np.array([x for x in fillArr if x[0] >= -0.2])
    # testPoint = np.array([2.000,2.000,2.000])
    # fillArr = np.append(fillArr, testPoint)
    return fillArr

array = np.load("./datasets/small_scene26.npy")
array2 = copy.deepcopy(array)

newArr = zfilter(array2)
print(array2.shape)
print(newArr.shape)

draw = o3d.geometry.PointCloud()
draw.points = o3d.utility.Vector3dVector(newArr)
o3d.visualization.draw_geometries([draw])

np.save("./datasets/gas_tank26.npy", newArr)
