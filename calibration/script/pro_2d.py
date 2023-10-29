import cv2
import numpy as np
import open3d as o3d
from func import *
import argparse
import os
def project_points(point_cloud, mtx, dist, rvecs, tvecs,cv):
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


    return projected_points

def pro_on_img(image, points):

    height, width, _ = image.shape
    print(height, width)
    print("全部的點雲數：", len(points))
    filtered_points = [(x, y) for x, y in points if 0 <= x < width and 0 <= y < height]
    print("圖片範圍的點雲數：", len(filtered_points))
    
    for x, y in filtered_points:
        cv2.circle(image, (round(x), round(y)), 1, (0, 255, 0), -1)
    cv2.imshow('Marked Image', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def load_image_and_point_cloud(img_path, pcd_path):
    img = cv2.imread(img_path)
    pcd_np = pcd_to_numpy(pcd_path)
    return img, pcd_np

def pcd_to_numpy(pcd_file):
    # 讀取pcd檔
    pcd = o3d.io.read_point_cloud(pcd_file)

    # 轉換為 numpy 陣列
    np_points = np.asarray(pcd.points)

    return np_points

def img_cal(img):
    height, width, _ = img.shape
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (width, height), 0)
    undistorted_img = cv2.undistort(img, mtx, dist, None, new_camera_matrix)   
    image_rgb = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2RGB)
    image_rgb = image_rgb.reshape(-1, 3) / 255.0 
    image_rgb = image_rgb.reshape(height, width , 3) 
    return image_rgb

def parse_args():
    parse = argparse.ArgumentParser()
    parse.add_argument('--pcd_path', type=str, default="../raw_data/pcd/pcd_20231029_133301.pcd")
    parse.add_argument('--img_path', type=str, default="../raw_data/img/img_20231029_133301.jpg")
    parse.add_argument('--img_size',  type=str, default="960_540")
    return parse.parse_args()

if __name__ == '__main__':

    # Show parameter
    args = parse_args()
    for key, value in vars(args).items():
        print(f"{key}: {value}")

    # Load data
    img, pcd_np = load_image_and_point_cloud(args.img_path, args.pcd_path)
    height, width, _ = img.shape
    print("img height, width:",height, width)

    # Load mtx, dist , rvecs, tvecs
    mtx, dist = read_camera_params(os.path.join('./cal_file/cam_cal/',args.img_size,'ost.txt'))
    rvecs, tvecs = load_rvecs_tvecs(os.path.join("cal_file", "lidar_cam_cal",args.img_size,'rvecs_tvecs.txt'))

    # Project points on image
    image_rgb=img_cal(img)
    points_2d = project_points(pcd_np, mtx, dist, rvecs, tvecs, False)
    img_pro=pro_on_img(img,points_2d)


    # Project image on points
    points_2d = np.round(points_2d).astype(int)  # 取整數像素坐標
    mask = (0 <= points_2d[:, 0]) & (points_2d[:, 0] < width) & (0 <= points_2d[:, 1]) & (points_2d[:, 1] < height)
    colors = np.zeros_like(pcd_np)  # 創建一個與點雲相同大小的0數組，用來儲存顏色
    colors[mask] = image_rgb[points_2d[mask, 1], points_2d[mask, 0]]  # 只對影像範圍內的點設定顏色
    
    
    # Show pointcloud
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
    o3d.visualization.draw_geometries([pcd])

