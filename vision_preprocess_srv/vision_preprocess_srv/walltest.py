import numpy as np
from sklearn.cluster import DBSCAN

import numpy as np
from sklearn.neighbors import KDTree

import open3d as o3d

# def compute_normals(points, k=10):
#     """
#     计算点云中每个点的法向量
#     :param points: 点云数组，大小为(N,3)
#     :param k: 最近邻的数目, 默认为10
#     :return: 法向量数组，大小为(N,3)
#     """
#     # 构建KD树
#     tree = KDTree(points)

#     # 计算每个点的最近邻
#     distances, indices = tree.query(points, k=k)

#     # 计算法向量
#     normals = []
#     for i in range(points.shape[0]):
#         # 获取最近邻点的坐标
#         nn_points = points[indices[i]]
#         # 计算最小二乘平面拟合
#         centroid = np.mean(nn_points, axis=0)
#         nn_points = nn_points - centroid
#         _, _, V = np.linalg.svd(nn_points)
#         normal = V[-1]
#         # 确定法向量的方向
#         if np.dot(normal, points[i] - centroid) < 0:
#             normal = -normal
#         normals.append(normal)

#     return np.array(normals)

# # 设置DBSCAN的参数
# dbscan_eps = 0.2 # 邻域半径
# dbscan_min_samples = 10 # 最小样本数

# # 加载点云数据
# pcd = o3d.io.read_point_cloud("/home/zyadan/ros2_ws_pro/src/pcdori.pcd")
# o3d.visualization.draw_geometries([pcd])

# pcd_np = np.array(pcd.points)
# # 计算法向量
# normals = compute_normals(pcd_np)

# # 将点云转换为水平平面
# points_xy = pcd_np[:, [0, 1]]
# db = DBSCAN(eps=dbscan_eps, min_samples=dbscan_min_samples).fit(points_xy)
# labels = db.labels_
# unique_labels = set(labels)
# max_label = max(labels)

# # 找到垂直墙面的点
# wall_points = []
# for i in range(max_label + 1):
#     cluster_points = pcd_np[labels == i]
#     cluster_normals = normals[labels == i]
#     if len(cluster_points) < 10:
#         continue
#     _, _, variances = np.linalg.svd(cluster_normals)
#     if np.max(variances) < 0.1:
#         wall_points.append(cluster_points)


# wall = o3d.geometry.PointCloud()
# wall.points = o3d.utility.Vector3dVector(wall_points)
# o3d.visualization.draw_geometries([wall])

# # 将墙面点云保存到文件
# np.savetxt('wall_points.txt', np.concatenate(wall_points, axis=0))

Observepcd = o3d.io.read_point_cloud("/home/zyadan/ros2_ws_pro/src/3.pcd")
Observepcd2 = o3d.io.read_point_cloud("/home/zyadan/ros2_ws_pro/src/2.pcd")
# Observepcd = o3d.io.read_point_cloud("/home/zyadan/zyd/pcdori.pcd")
# plane_model, inliers = pcd.segment_plane(distance_threshold=0.05,
#                                          ransac_n=10,
#                                          num_iterations=1000)
# [a, b, c, d] = plane_model
# print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

# # 提取平面
# inlier_cloud = pcd.select_by_index(inliers)
# outlier_cloud = pcd.select_by_index(inliers, invert=True)

# # 平面分割，提取所有平面
# plane_model_list, inliers_list = pcd.segment_plane_multi(distance_threshold=0.01, 
#                                                          ransac_n=3, 
#                                                          num_iterations=1000)

# # 创建平面几何对象并显示
# plane_meshes = []
# for i, plane_model in enumerate(plane_model_list):
#     plane_inliers = pcd.select_by_index(inliers_list[i])
#     plane_mesh = plane_inliers.create_mesh_coordinate_frame(size=0.5, origin=plane_model[:3])
#     plane_meshes.append(plane_mesh)

# o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud] + plane_meshes)


# o3d.visualization.draw_geometries([Observepcd, Observepcd2])


planes = []
wall_normal = []
wall_pcd_np = np.zeros((3, 3))
wall_pcd = o3d.geometry.PointCloud()
wall_pcd.points = o3d.utility.Vector3dVector(Observepcd.points)
# o3d.visualization.draw_geometries([wall_pcd])
print("jioooooooooo",np.array(Observepcd.points))
# np.savetxt('/home/zyadan/ros2_ws_pro/src/wall_points.txt', np.array(Observepcd.points))
plane_num = 0
while len(Observepcd.points) > 50000 and plane_num <6:
    # 点云平面分割
    plane_model, inliers = Observepcd.segment_plane(distance_threshold=0.005, ransac_n=3, num_iterations=200)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    
    # 从点云中提取出平面
    inlier_cloud = Observepcd.select_by_index(inliers)
    outlier_cloud = Observepcd.select_by_index(inliers, invert=True)
    planes.append(inlier_cloud)
    wall_normal.append(plane_model[:3])
    plane_num +=1
    # 继续分割剩余点云
    Observepcd = outlier_cloud

angle_threshold = np.pi / 3 # 设置夹角阈值为60度
vertical_direction = np.array([0, 0, 1])
print(len(planes))
for i in range(len(planes)):
    print("wall_normal[i]", wall_normal[i])
    cos_theta = np.abs(np.dot(wall_normal[i], vertical_direction))
    print("cos_theta", cos_theta)
    if cos_theta < np.cos(angle_threshold):
        wall_pcd_np = np.concatenate([np.asarray(wall_pcd.points), np.asarray(planes[i].points)], axis=0)
        # wall_pcd.points = o3d.utility.Vector3dVector(wall_pcd_np)
        print('hereeeeeeeeeeee')
        wall_pcd.points = o3d.utility.Vector3dVector(wall_pcd_np)
# wall_pcd_filtered, _ = wall_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=4.0)
o3d.visualization.draw_geometries([wall_pcd])

# inlier_cloud = pcd.select_by_index(inliers)

# # o3d.visualization.draw_geometries([inlier_cloud])

# wall_points = np.array(pcd.points)[inliers]
# wall_normal = plane_model[:3]

# # 检查平面法向量是否与竖直方向夹角较小
# angle_threshold = np.pi / 4  # 设置夹角阈值为45度
# vertical_direction = np.array([0, 0, 1])
# cos_theta = np.abs(np.dot(wall_normal, vertical_direction))
# if cos_theta > np.cos(angle_threshold):
#     print("提取的点云是墙面")
# else:
#     print("提取的点云不是墙面")

# # 可视化结果
# wall_pcd = o3d.geometry.PointCloud()
# wall_pcd.points = o3d.utility.Vector3dVector(wall_points)
# o3d.visualization.draw_geometries([wall_pcd])