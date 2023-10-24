# import open3d as o3d
# import os
# path="/home/icalab/UDA_PCSS/src/IBPCSS_UDA/LeGO-LOAM/static_map/20230906_172122"
# cornerMap=os.path.join(path,"cornerMap.pcd")
# finalCloud=os.path.join(path,"finalCloud.pcd")
# surfaceMap=os.path.join(path,"surfaceMap.pcd")
# trajectory=os.path.join(path,"trajectory.pcd")
# # 讀取三個PCD文件
# pcd1 = o3d.io.read_point_cloud(cornerMap)
# pcd2 = o3d.io.read_point_cloud(finalCloud)
# pcd3 = o3d.io.read_point_cloud(surfaceMap)
# pcd4 = o3d.io.read_point_cloud(trajectory)

# # 將它們添加到同一個視窗
# o3d.visualization.draw_geometries([pcd1, pcd2, pcd3,pcd4])
import open3d as o3d
import os

# 設定PCD檔的路徑
path = "/home/icalab/UDA_PCSS/src/IBPCSS_UDA/LeGO-LOAM/static_map/20230906_172122"
cornerMap = os.path.join(path, "cornerMap.pcd")
finalCloud = os.path.join(path, "finalCloud.pcd")
surfaceMap = os.path.join(path, "surfaceMap.pcd")
trajectory = os.path.join(path, "trajectory.pcd")

# 讀取PCD檔
cloud1 = o3d.io.read_point_cloud(cornerMap)
cloud2 = o3d.io.read_point_cloud(finalCloud)
cloud3 = o3d.io.read_point_cloud(surfaceMap)
cloud4 = o3d.io.read_point_cloud(trajectory)

# 設定點雲顏色
cloud1.paint_uniform_color([1, 0, 0])  # 紅色
cloud2.paint_uniform_color([0, 1, 0])  # 綠色
cloud3.paint_uniform_color([0, 0, 1])  # 藍色
cloud4.paint_uniform_color([1, 1, 0])  # 黃色

# 顯示所有點雲
# o3d.visualization.draw_geometries([cloud1, cloud2, cloud3, cloud4])
o3d.visualization.draw_geometries([cloud1])
