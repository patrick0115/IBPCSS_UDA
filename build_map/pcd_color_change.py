import open3d as o3d
import numpy as np
from collections import Counter
from sklearn.cluster import KMeans


def make_points_black(pcd):
    # 獲取點雲中點的數量
    num_points = np.asarray(pcd.points).shape[0]

    # 創建一個全黑色的顏色數組
    black_colors = np.zeros((num_points, 3))

    # 更新點雲的顏色
    pcd.colors = o3d.utility.Vector3dVector(black_colors)

    return pcd

def display_point_cloud(pcd, point_size=1.0):
    # 創建視覺化窗口
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # 添加點雲到視覺化窗口
    vis.add_geometry(pcd)

    # 獲取渲染選項並設定點的大小
    render_option = vis.get_render_option()
    render_option.point_size = point_size

    # 顯示點雲
    vis.run()
    vis.destroy_window()

def process_pcd(file_path):
    
    # 讀取PCD檔案
    pcd = o3d.io.read_point_cloud(file_path)


    # 獲取點和顏色
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)



    # 進行顏色替換
    colors[np.all(colors == [8/255, 255/255, 214/255], axis=1)] = [1/255, 1/255, 1/255]
    colors[np.all(colors == [11/255, 102/255, 255/255], axis=1)] = [1/255, 1/255, 1/255]
    colors[np.all(colors == [0/255, 122/255, 255/255], axis=1)] = [1/255, 1/255, 1/255]
    colors[np.all(colors == [8/255, 255/255, 51/255], axis=1)] = [1/255, 1/255, 1/255]  # 新增的轉換
    colors[np.logical_and(points[:, 2] > 0.1, np.all(colors == [80/255, 50/255, 50/255], axis=1))] = [120/255, 120/255, 120/255]
    colors[np.all(colors == [120/255, 120/255, 80/255], axis=1)] = [120/255, 120/255, 120/255]
    colors[np.all(colors == [1/255, 1/255, 1/255], axis=1)] = [120/255, 120/255, 120/255]  # 將黑色改為灰色

    # 更新點雲顏色
    pcd.colors = o3d.utility.Vector3dVector(colors)


    # 顯示點雲
    # o3d.visualization.draw_geometries([pcd])
    return pcd


def smooth_colors(pcd, radius=0.05):
    # 使用KDTree尋找鄰近點
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    new_colors = np.zeros_like(colors)

    for i, point in enumerate(points):
        # 尋找鄰近點
        [k, idx, _] = pcd_tree.search_radius_vector_3d(point, radius)
        if k > 0:
            # 計算鄰近點的顏色平均值
            new_colors[i] = np.mean(colors[idx, :], axis=0)
        else:
            new_colors[i] = colors[i]

    # 更新點雲顏色
    pcd.colors = o3d.utility.Vector3dVector(new_colors)
    # o3d.visualization.draw_geometries([pcd])
    return pcd

def filter_isolated_points(pcd, radius=0.05, min_neighbors=10):
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    points = np.asarray(pcd.points)
    filtered_indices = []

    for i, point in enumerate(points):
        [k, idx, _] = pcd_tree.search_radius_vector_3d(point, radius)
        if k >= min_neighbors:
            filtered_indices.append(i)

    # 創建一個新的點雲，只包含非懸空點
    filtered_pcd = pcd.select_by_index(filtered_indices)
    # o3d.visualization.draw_geometries([filtered_pcd])
    return filtered_pcd


def adjust_color_to_dominant(pcd, radius=0.5, dominance_threshold=0.6):
    points = np.asarray(pcd.points).astype(np.float64)  # 確保點數據是浮點型
    colors = np.asarray(pcd.colors)
    new_colors = colors.copy()
    # 檢查點數據是否包含無效值
    if np.any(np.isnan(points)) or np.any(np.isinf(points)):
        raise ValueError("Point data contains NaN or Inf values.")

    # 使用點數據創建KDTree
    pcd_tree = o3d.geometry.KDTreeFlann(points)

    for i, point in enumerate(points):
        try:
            [k, idx, _] = pcd_tree.search_radius_vector_3d(point, radius)
            if k > 1:  # 排除自己
                neighbor_colors = colors[idx[1:], :]  # 排除自己
                # 使用KMeans或其他聚類方法找到最常見的顏色
                kmeans = KMeans(n_clusters=1).fit(neighbor_colors)
                dominant_color = kmeans.cluster_centers_[0]

                # 如果主導顏色佔比超過閾值，則將點的顏色更改為該顏色
                color_counts = Counter(map(tuple, neighbor_colors))
                dominant_color_count = color_counts[tuple(dominant_color)]
                if (dominant_color_count / (k - 1)) > dominance_threshold:
                    new_colors[i] = dominant_color
        except RuntimeError as e:
            
            print(f"Error at point {i}: {e}")
            break
    # 更新點雲顏色
    pcd.colors = o3d.utility.Vector3dVector(new_colors)
    pcd.colors = o3d.utility.Vector3dVector(new_colors)
    # o3d.visualization.draw_geometries([pcd])
    return pcd
# 呼叫函數並傳入您的PCD檔案路徑
pcd=process_pcd("pcss_map.pcd")

# pcd_smoothed = smooth_colors(pcd)

pcd_filter=filter_isolated_points(pcd)
pcd_majority_color = adjust_color_to_dominant(pcd_filter)
pcd_black = make_points_black(pcd_majority_color)
display_point_cloud(pcd_black, point_size=0.5)




