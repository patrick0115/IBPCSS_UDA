import open3d as o3d
import numpy as np

def load_point_cloud(filename):
    """載入點雲檔案"""
    pcd = o3d.io.read_point_cloud(filename)
    return pcd


def display_point_clouds(original_pcd, filtered_pcd):
    """顯示原始點雲和濾波後的點雲，並將背景設為黑色，濾波後的點設為紅色，並減小點的尺寸"""
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # 將原始點雲的點設為白色 (或其他顏色)
    original_pcd.paint_uniform_color([1, 1, 1])  # 白色
    vis.add_geometry(original_pcd)

    # 將濾波後的點雲的點設為紅色
    filtered_pcd.paint_uniform_color([1, 0, 0])  # 紅色
    vis.add_geometry(filtered_pcd)

    # 設定渲染選項
    opt = vis.get_render_option()
    opt.background_color = np.array([0, 0, 0])  # 背景設為黑色
    opt.point_size = 2.0  # 調整點的尺寸

    vis.run()
    vis.destroy_window()

def remove_statistical_outliers(pcd, nb_neighbors=20, std_ratio=2.0):
    """統計離群值移除"""
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return pcd.select_by_index(ind)

def remove_radius_outliers(pcd, nb_points=16, radius=0.05):
    """半徑離群值移除"""
    cl, ind = pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)
    return pcd.select_by_index(ind)

def voxel_downsample(pcd, voxel_size=0.02):
    """體素下採樣"""
    return pcd.voxel_down_sample(voxel_size=voxel_size)

def uniform_downsample(pcd, every_k_points=10):
    """均勻下採樣"""
    return pcd.uniform_down_sample(every_k_points=every_k_points)

def estimate_normals(pcd, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)):
    """估計法向量"""
    pcd.estimate_normals(search_param=search_param)
    return pcd

def remove_points_within_cylinder(pcd, center, radius, height):
    """
    移除位於指定圓柱形內的點

    :param pcd: 要處理的點雲對象
    :param center: 圓柱體的中心原點（x, y, z）
    :param radius: 圓柱體的半徑
    :param height: 圓柱體的高度
    :return: 處理後的點雲
    """
    # 轉換點雲為 NumPy 數組
    points = np.asarray(pcd.points)
    
    # 計算每個點到圓柱中心的 XY 平面距離
    distances = np.sqrt((points[:, 0] - center[0]) ** 2 + (points[:, 1] - center[1]) ** 2)
    
    # 判斷點是否位於圓柱內
    in_cylinder = np.logical_and(distances <= radius, np.abs(points[:, 2] - center[2]) <= height / 2)

    # 過濾掉位於圓柱內的點
    filtered_points = points[~in_cylinder]

    # 創建新的點雲對象
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)

    return filtered_pcd
# 主程式
def main():
    # 載入點雲檔案的路徑
    file_path = "./pcss_map.pcd"

    # 統計離群值移除的參數
    nb_neighbors_statistical = 10  # 鄰居數量：用於確定每個點的鄰居範圍
    std_ratio_statistical = 3.0    # 標準差比率：較高的值將保留更多的點

    # 半徑離群值移除的參數
    nb_points_radius = 10   # 鄰居點數量閾值：半徑內小於此數量的點將被移除
    radius = 0.01                   # 半徑大小：確定每個點的鄰居搜索範圍

    # 體素下採樣的參數
    voxel_size = 0.002              # 體素大小：較大的體素將更多地減少點雲密度

    # 均勻下採樣的參數
    every_k_points = 10            # 下採樣間隔：每隔 k 個點選取一個點

    # 法向量估計的參數
    radius_normal = 0.1            # 搜索半徑：用於估計法線的鄰居搜索範圍
    max_nn_normal = 30             # 最大鄰居數：用於估計法線的最大鄰居數量

    center = [0, 0, 0.13]  # 圓柱體中心原點的坐標
    radius = 1.0  # 圓柱體的半徑
    height = 1.0  # 圓柱體的高度

    # 載入點雲檔案
    pcd = load_point_cloud(file_path)

    # 複製點雲用於後續比較
    original_pcd = pcd


    # 調用函數移除圓柱形區域內的點
    # pcd = remove_points_within_cylinder(pcd, center, radius, height)

    # 統計離群值移除
    pcd = remove_statistical_outliers(pcd, nb_neighbors=nb_neighbors_statistical, std_ratio=std_ratio_statistical)

    # # 半徑離群值移除
    # pcd = remove_radius_outliers(pcd, nb_points=nb_points_radius, radius=radius)

    # # 體素下採樣
    # pcd = voxel_downsample(pcd, voxel_size=voxel_size)

    # # 均勻下採樣
    # pcd = uniform_downsample(pcd, every_k_points=every_k_points)

    # # 估計法向量
    # search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=max_nn_normal)
    # pcd = estimate_normals(pcd, search_param=search_param)

    # 顯示濾除前後的對比
    display_point_clouds(original_pcd, pcd)

if __name__ == "__main__":
    main()
