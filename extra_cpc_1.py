import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import cv2

# 读取点云数据
ply_file_path = "/Users/liangchaodeng/Documents/VScode/canopypc/site_20230623_02.ply"
pcd = o3d.io.read_point_cloud(ply_file_path)

# 可视化原始点云
o3d.visualization.draw_geometries([pcd])

# 去除离群点
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

# 可视化去噪点云
o3d.visualization.draw_geometries([pcd])

# 创建 3D 坐标系
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

# 计算点云的中心点
center = pcd.get_center()
print(f"点云中心坐标: {center}")

# 平移点云，使点云中心对齐到原点
pcd.translate(-center)


# 将点云投影到 XY 平面（Z 设为 0）
points_xy = np.asarray(pcd.points).copy()
points_xy[:, 2] = 0.001 # Open3D 可能无法很好地渲染这些数据,给 Z 轴添加微小高度

# 计算点云的最大高度（Z 轴）
z_max = np.max(points_xy[:, 2])

# 创建投影后的点云
pcd_xy = o3d.geometry.PointCloud()
pcd_xy.points = o3d.utility.Vector3dVector(points_xy)

# 计算最小外接矩形
rect = cv2.minAreaRect(points_xy[:, :2].astype(np.float32))  # 仅使用 XY 维度
box = cv2.boxPoints(rect)  # 获取矩形的四个角点
box = np.array(box)

# 创建 MBR 的 3D 线段
mbr_3d = np.zeros((8, 3))
mbr_3d[:4, :2] = box  # 底部 4 个点
mbr_3d[4:, :2] = box  # 顶部 4 个点
mbr_3d[4:, 2] = np.max(np.asarray(pcd.points)[:, 2])  # 设置顶部高度
mbr_3d[:4, 2] = np.min(np.asarray(pcd.points)[:, 2])  # 设置底部高度

# 可视化平面点云
o3d.visualization.draw_geometries([pcd_xy, axis])

# 创建 3D MBR 包围盒的线段
lines_3d = [
    [0, 1], [1, 2], [2, 3], [3, 0],  # 底部矩形
    [4, 5], [5, 6], [6, 7], [7, 4],  # 顶部矩形
    [0, 4], [1, 5], [2, 6], [3, 7]   # 连接底部和顶部
]
colors_3d = [[1, 0, 0] for _ in range(len(lines_3d))]  # 红色边框
line_set_3d = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(mbr_3d),
    lines=o3d.utility.Vector2iVector(lines_3d),
)
line_set_3d.colors = o3d.utility.Vector3dVector(colors_3d)

# 可视化点云 + 3D MBR + 坐标轴
o3d.visualization.draw_geometries([pcd, line_set_3d, axis])







































# 计算投影后的 OBB
obb_xy = pcd_xy.get_oriented_bounding_box()
obb_xy.color = (1, 0, 0)  # 红色 OBB

# 可视化原始点云 + OBB + 坐标轴
o3d.visualization.draw_geometries([pcd, obb_xy, axis])

# 重新获取 OBB 并与点云对齐
obb = pcd.get_oriented_bounding_box()
obb.color = (1, 0, 0)  # 设置 OBB 为红色

# 创建 3D 坐标系
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

# 可视化对齐到原点后的点云 + OBB + 坐标轴
o3d.visualization.draw_geometries([pcd, obb, axis])


# 计算点云的主方向（PCA）
def compute_pca_alignment(pcd):
    points = np.asarray(pcd.points)
    center = points.mean(axis=0)
    cov_matrix = np.cov(points.T)  # 计算协方差矩阵
    eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)  # 计算特征值和特征向量

    # 对特征向量进行排序，从最大到最小
    order = eigenvalues.argsort()[::-1]
    eigenvectors = eigenvectors[:, order]

    return eigenvectors

# 获取主方向
eigenvectors = compute_pca_alignment(pcd)

# 生成旋转矩阵，将主方向与 XYZ 轴对齐
R = eigenvectors.T

# 旋转点云到主方向
pcd.rotate(R, center=pcd.get_center())

# 重新获取 OBB（主方向包围盒）
obb = pcd.get_oriented_bounding_box()
obb.color = (1, 0, 0)  # 设置包围盒为红色

# 可视化对齐后的点云和 OBB
o3d.visualization.draw_geometries([pcd, obb, axis])

# 计算旋转矩阵，将最大平面与 XY 平面对齐
center = aabb.get_center()
extent = aabb.get_extent()

# 选择最大面的方向（通常是 X 或 Y 方向）
if extent[0] > extent[1]:
    R = pcd.get_rotation_matrix_from_xyz((0, 0, -np.pi / 2))  # 旋转 90 度
else:
    R = np.eye(3)  # 不旋转

# 旋转点云，使最大平面对齐到 XY 平面
pcd.rotate(R, center=center)

# 可视化放平后的点云和 AABB 包围盒
o3d.visualization.draw_geometries([pcd, aabb])

# 获取点云数据
points = np.asarray(pcd.points)

# 重新设定地面阈值（最低 2% 作为地面）
ground_threshold = np.percentile(points[:, 2], 30)

# 删除地面点
canopy_points = points[points[:, 2] > ground_threshold]
canopy_pcd = o3d.geometry.PointCloud()
canopy_pcd.points = o3d.utility.Vector3dVector(canopy_points)

# 可视化处理后的冠层点云
o3d.visualization.draw_geometries([canopy_pcd])

# 获取点云数据的坐标
points = np.asarray(pcd.points)

# 设定地面高度阈值（根据实际情况调整）
ground_threshold = np.percentile(points[:, 2], 10)  # 取最低 5% 高度作为地面

# 删除地面点
canopy_points = points[points[:, 2] > ground_threshold]
canopy_pcd = o3d.geometry.PointCloud()
canopy_pcd.points = o3d.utility.Vector3dVector(canopy_points)

# 可视化处理后的冠层点云
o3d.visualization.draw_geometries([canopy_pcd])

# 根据 X 或 Y 坐标将点云分为三部分（假设按 X 坐标分）
x_min, x_max = np.min(canopy_points[:, 0]), np.max(canopy_points[:, 0])
step = (x_max - x_min) / 3  # 将 X 坐标分为三部分

# 提取三大行
row_1 = canopy_points[(canopy_points[:, 0] >= x_min) & (canopy_points[:, 0] < x_min + step)]
row_2 = canopy_points[(canopy_points[:, 0] >= x_min + step) & (canopy_points[:, 0] < x_min + 2*step)]
row_3 = canopy_points[(canopy_points[:, 0] >= x_min + 2*step) & (canopy_points[:, 0] <= x_max)]

# 创建三大行的点云
row_1_pcd = o3d.geometry.PointCloud()
row_2_pcd = o3d.geometry.PointCloud()
row_3_pcd = o3d.geometry.PointCloud()
row_1_pcd.points = o3d.utility.Vector3dVector(row_1)
row_2_pcd.points = o3d.utility.Vector3dVector(row_2)
row_3_pcd.points = o3d.utility.Vector3dVector(row_3)

# 可视化三个大行的点云
o3d.visualization.draw_geometries([row_1_pcd, row_2_pcd, row_3_pcd])

# 计算冠层高度（最大 z 坐标）
def compute_canopy_height(points):
    return np.max(points[:, 2])

# 计算体积（通过凸包）
def compute_canopy_volume(points):
    from scipy.spatial import ConvexHull
    hull = ConvexHull(points)
    return hull.volume

# 计算每个大行的冠层高度和体积
height_1 = compute_canopy_height(row_1)
height_2 = compute_canopy_height(row_2)
height_3 = compute_canopy_height(row_3)

volume_1 = compute_canopy_volume(row_1)
volume_2 = compute_canopy_volume(row_2)
volume_3 = compute_canopy_volume(row_3)

print(f"Row 1 - Height: {height_1}, Volume: {volume_1}")
print(f"Row 2 - Height: {height_2}, Volume: {volume_2}")
print(f"Row 3 - Height: {height_3}, Volume: {volume_3}")

# 保存处理后的点云
o3d.io.write_point_cloud("canopy_row_1.ply", row_1_pcd)
o3d.io.write_point_cloud("canopy_row_2.ply", row_2_pcd)
o3d.io.write_point_cloud("canopy_row_3.ply", row_3_pcd)