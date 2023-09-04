import cv2
import numpy as np
import open3d as o3d
from func import *



def project_points(point_cloud, mtx, dist, rvecs, tvecs,cv):
    if cv:
        projected_points, _ = cv2.projectPoints(point_cloud, rvecs, tvecs, mtx,dist)
        projected_points = np.squeeze(projected_points, axis=1)
    else:
        R, _ = cv2.Rodrigues(rvecs)
        RT = np.column_stack((R, tvecs))
        P = np.dot(mtx, RT)
        point_cloud_homogeneous = np.column_stack((point_cloud, np.ones(point_cloud.shape[0])))
        projected_points = np.dot(P, point_cloud_homogeneous.T).T
        projected_points = projected_points[:, :2] / projected_points[:, 2, np.newaxis]

    return projected_points



def create_img(image, points,word):
    cv2.putText(image, word, (80, 450), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 0), 2, cv2.LINE_AA)

    points = np.round(points).astype(int)
    for pt in points:
        cv2.circle(image, tuple(pt), 3, (0, 0, 255), -1)
    return image

def draw_image(image):
    cv2.imshow('Image with Points', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def load_image_and_point_cloud(img_path, pcd_path):
    img = cv2.imread(img_path)
    pcd_np = pcd_to_numpy(pcd_path)
    return img, pcd_np

if __name__ == '__main__':
    # Load data
    pcd_path="../raw_data/pcd/pcd_20230824_170141.pcd"
    img_path="../raw_data/img/img_20230824_170141.jpg"
    img, pcd_np = load_image_and_point_cloud(img_path, pcd_path)
    pcd_name = pcd_path.split("/")[-1][0:-4]

    mtx, dist = load_txt('cal_file.txt')
    rvecs, tvecs = load_rvecs_tvecs(os.path.join("cal_file", "lidar_cam_cal", pcd_name,pcd_name+'_extrinsic.txt'))
    
    img_copy = np.copy(img)
    # Project points
    points_2d = project_points(pcd_np, mtx, dist, rvecs, tvecs,False)
    img_pro_cv=create_img(img,points_2d,"Use cv2.projectPoints")
    img_pro=create_img(img_copy,points_2d,"Project")
    img_fusion = np.hstack((img_pro_cv, img_pro))
    # draw_image(img3)


    # 假設你已經有了點雲和2D點的坐標
    points_2d = np.round(points_2d).astype(int)  # 取整數像素坐標
    img = cv2.imread(img_path)
    height, width = img.shape[:2]
    image_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = image_rgb.reshape(-1, 3) / 255.0 
    img = img.reshape(height, width , 3) 
    

    height, width = img.shape[:2]
    mask = (0 <= points_2d[:, 0]) & (points_2d[:, 0] < width) & (0 <= points_2d[:, 1]) & (points_2d[:, 1] < height)


    # 從圖像中提取顏色
    colors = np.zeros_like(pcd_np)  # 創建一個與點雲相同大小的0數組，用來儲存顏色
    colors[mask] = img[points_2d[mask, 1], points_2d[mask, 0]]  # 只對影像範圍內的點設定顏色

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

