import open3d as o3d
import numpy as np

# 讀取PCD檔案
pcd = o3d.io.read_point_cloud("./pcss_map.pcd")

# 檢查是否含有顏色數據
if pcd.has_colors():
    # 提取顏色數據
    colors = np.asarray(pcd.colors)
    colors_255 = np.round(colors * 255).astype(np.uint8)

    # 顯示轉換後的顏色數據

    # 計算唯一顏色的數量
    unique_colors = np.unique(colors_255, axis=0)
    print("獨特顏色的數量:", unique_colors.shape[0])
    print("獨特顏色:", unique_colors)
else:
    print("這個檔案不包含顏色數據")