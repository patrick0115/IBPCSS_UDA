import cv2
import numpy as np
import open3d as o3d
from func import *
import argparse

def project_points(point_cloud, mtx, dist, rvecs, tvecs,cv):
    # print(point_cloud.shape)
    # print(mtx.shape)
    # print(dist.shape)
    # print(rvecs.shape)
    if cv:
        projected_points, _ = cv2.projectPoints(point_cloud, rvecs, tvecs, mtx,dist,aspectRatio=1)
        
        projected_points = np.squeeze(projected_points, axis=1)

    else:
        R, _ = cv2.Rodrigues(rvecs)
        RT = np.column_stack((R, tvecs))
        P = np.dot(mtx, RT)
        point_cloud_homogeneous = np.column_stack((point_cloud, np.ones(point_cloud.shape[0])))
        projected_points = np.dot(P, point_cloud_homogeneous.T).T
        projected_points = projected_points[:, :2] / projected_points[:, 2, np.newaxis]
        # 計算旋轉矩陣
        # R, _ = cv2.Rodrigues(rvecs)
        
        # # 應用旋轉和平移
        # rotated_points = np.dot(R, point_cloud.T).T + tvecs.T
        # print("應用旋轉和平移完成。")
        
        # # 分別取出x, y, z
        # x = rotated_points[:, 0]
        # y = rotated_points[:, 1]
        # z = rotated_points[:, 2]
        
        # # 進行投影
        # x_proj = x / z
        # y_proj = y / z
        # xy_proj = np.stack([x_proj, y_proj], axis=1)
        
        # # 應用相機內參和失真
        # r2 = x_proj ** 2 + y_proj ** 2
        # radial_dist = 1 + dist[0, 0] * r2 + dist[0, 1] * (r2 ** 2) + dist[0, 4] * (r2 ** 3)
        # x_proj *= radial_dist
        # y_proj *= radial_dist
        
        # # 更新失真
        # x_proj += 2 * dist[0, 2] * x_proj * y_proj + dist[0, 3] * (r2 + 2 * x_proj ** 2)
        # y_proj += dist[0, 2] * (r2 + 2 * y_proj ** 2) + 2 * dist[0, 3] * x_proj * y_proj
        
        # # 應用內參矩陣
        # u = mtx[0, 0] * x_proj + mtx[0, 2]
        # v = mtx[1, 1] * y_proj + mtx[1, 2]
        
        # projected_points = np.stack([u, v], axis=1)
        # print("投影和失真完成。")

    return projected_points

def create_img(image, points):

    height, width, _ = image.shape
    print(height, width)
    print("過濾後的點數量：", len(points))
    filtered_points = [(x, y) for x, y in points if 0 <= x < width and 0 <= y < height]
    print("過濾後的點數量：", len(filtered_points))
    
    for x, y in filtered_points:
        # cv2.circle(image, (round(width-x), round(height-y)), 1, (0, 255, 0), -1)
        cv2.circle(image, (round(x), round(y)), 1, (0, 255, 0), -1)

    # 保存或顯示標記後的圖像
    # cv2.imwrite('q.jpg', image)
    cv2.imshow('Marked Image', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def load_image_and_point_cloud(img_path, pcd_path):
    img = cv2.imread(img_path)
    pcd_np = pcd_to_numpy(pcd_path)
    return img, pcd_np
def parse_args():
    parse = argparse.ArgumentParser()
    # parse.add_argument('--pcd_path', type=str, default="../raw_data/pcd/pcd_20231023_173205.pcd")
    # parse.add_argument('--img_path', type=str, default="../raw_data/img/img_20231023_173205.jpg")
    # parse.add_argument('--pcd_path', type=str, default="../raw_data/1020/pcd/pcd_20231020_194528.pcd")
    # parse.add_argument('--img_path', type=str, default="../raw_data/1020/img/img_20231020_194528.jpg")
    parse.add_argument('--pcd_path', type=str, default="../raw_data/pcd/pcd_20231023_222352.pcd")
    parse.add_argument('--img_path', type=str, default="../raw_data/img/img_20231023_222352.jpg")
    
    return parse.parse_args()

if __name__ == '__main__':
    # Load data
    args = parse_args()
    
    for key, value in vars(args).items():
        print(f"{key}: {value}")

    img, pcd_np = load_image_and_point_cloud(args.img_path, args.pcd_path)
    height, width, _ = img.shape
    print("img height, width:",height, width)
    # pcd_name = args.pcd_path.split("/")[-1][0:-4]
    mtx, dist = read_camera_params('./cal_file/cam_cal/ost.txt')
    rvecs, tvecs = load_rvecs_tvecs(os.path.join("cal_file", "lidar_cam_cal",'rvecs_tvecs.txt'))

    print(mtx)
    print(dist)
    print(rvecs)
    print(tvecs)
    # rvecs, tvecs = load_rvecs_tvecs(os.path.join("cal_file", "lidar_cam_cal", pcd_name,pcd_name+'_extrinsic.txt'))

    img_copy = np.copy(img)
    # Project points

    points_2d = project_points(pcd_np, mtx, dist, rvecs, tvecs, False)
    print(type(points_2d))
    print(points_2d.shape)
    img_pro=create_img(img,points_2d)

    # 假設你已經有了點雲和2D點的坐標

    points_2d = np.round(points_2d).astype(int)  # 取整數像素坐標
    # # points_2d = np.round(points_2d)
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (width, height), 0)
        # 進行圖像校正
    undistorted_img = cv2.undistort(img_copy, mtx, dist, None, new_camera_matrix)
    # undistorted_img
    height, width, _ = undistorted_img.shape
    print("img height, width:",height, width)
    img = cv2.imread(args.img_path)
    image_rgb = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2RGB)
    image_rgb = image_rgb.reshape(-1, 3) / 255.0 
    image_rgb = image_rgb.reshape(height, width , 3) 
    
    # # mask 是一個(點雲數量, 2)布林陣列
    mask = (0 <= points_2d[:, 0]) & (points_2d[:, 0] < width) & (0 <= points_2d[:, 1]) & (points_2d[:, 1] < height)
    # # print(mask.shape)

    # # 從圖像中提取顏色
    colors = np.zeros_like(pcd_np)  # 創建一個與點雲相同大小的0數組，用來儲存顏色
    print(points_2d.shape)

    # for i in range(len(points_2d[mask, 0])):
    #     print(points_2d[mask, 0][i])
    colors[mask] = image_rgb[points_2d[mask, 1], points_2d[mask, 0]]  # 只對影像範圍內的點設定顏色
    # print(colors)
    # 建立點雲並設置顏色
    x_range = [ 0, 10]
    y_range = [-10, 10]
    z_range = [-5, 5]
    bound_min = [x_range[0], y_range[0], z_range[0]]
    bound_max = [x_range[1], y_range[1], z_range[1]]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcd_np)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    pcd = pcd.crop(
        o3d.geometry.AxisAlignedBoundingBox(min_bound=bound_min, max_bound=bound_max))
    # 顯示彩色點雲
    o3d.visualization.draw_geometries([pcd])

