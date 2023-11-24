import open3d as o3d
import numpy as np

def create_ground_plane_from_pcd(pcd, density):
    """
    創建一個基於原始點雲長寬的地面平面點雲

    :param pcd: 原始點雲
    :param density: 平面點雲的密度（每單位面積的點數）
    :return: 地面平面點雲
    """
    # 獲取點雲的邊界
    min_bound = pcd.get_min_bound()
    max_bound = pcd.get_max_bound()

    # 計算平面的長寬
    width = max_bound[0] - min_bound[0]
    depth = max_bound[1] - min_bound[1]

    # 根據密度計算點的間距
    spacing = np.sqrt(1 / density)

    # 生成平面點
    x = np.arange(min_bound[0], max_bound[0], spacing)
    y = np.arange(min_bound[1], max_bound[1], spacing)
    xx, yy = np.meshgrid(x, y)
    zz = np.zeros_like(xx)

    # 將點轉換為點雲
    points = np.vstack((xx.ravel(), yy.ravel(), zz.ravel())).T
    ground_plane = o3d.geometry.PointCloud()
    ground_plane.points = o3d.utility.Vector3dVector(points)

    return ground_plane
# 假設 pcd 是您已經載入的原始室內點雲
file_path = "map.pcd"
pcd = o3d.io.read_point_cloud(file_path)
# 自定義平面的密度，例如每平方單位 100 個點
density = 500

# 創建地面平面
ground_plane = create_ground_plane_from_pcd(pcd, density=density)
combined_pcd = pcd + ground_plane
# 可視化結果
o3d.visualization.draw_geometries([combined_pcd])
# 保存合併後的點雲為 PCD 檔案
output_filename = "combined_point_cloud.pcd"
o3d.io.write_point_cloud(output_filename, ground_plane)
print(f"點雲已保存為 {output_filename}")