import open3d as o3d

def filter_pcd(input_file, output_file):
    # 讀取PCD文件
    pcd = o3d.io.read_point_cloud(input_file)

    # 過濾點雲中z值不在-0.1到0.1之間的點
    filtered_points = [point for point in pcd.points if  point[2] > 0.1]

    # 創建一個新的點雲對象
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)

    # 保存過濾後的點雲到新文件
    o3d.io.write_point_cloud(output_file, filtered_pcd)
    print(f"Filtered PCD saved to {output_file}")

# 使用範例
input_file = "./map_plane.pcd"
output_file = "./map_wo_plane.pcd"
filter_pcd(input_file, output_file)
