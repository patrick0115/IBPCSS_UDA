import os
import open3d as o3d
import argparse
import cv2
import numpy as np
from func import *

def vec_to_mat(rotation_vec, translation_vec):
    # 使用cv2.Rodrigues將旋轉向量轉換為旋轉矩陣
    rotation_mat, _ = cv2.Rodrigues(rotation_vec)
    print(f'旋轉矩陣:\n{rotation_mat}')


    # 建立4x4的轉換矩陣
    transformation_mat = np.eye(4)
    transformation_mat[:3, :3] = rotation_mat
    transformation_mat[:3, 3] = translation_vec.flatten()

    print(f'轉換矩陣:\n{transformation_mat}')
    return transformation_mat ,rotation_mat , translation_vec

def find_pose(img_path, mtx, dist, square_size, grid_size,objp ,show=False):
    print('正在找尋旋轉向量和平移向量...')
     # 讀取影像
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    img = cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 尋找棋盤格角點
    ret, corners = cv2.findChessboardCorners(gray, (grid_size[0], grid_size[1]), flags=flags)

    if ret == True:  
        _, rvecs, tvecs = cv2.solvePnP(objp.reshape(-1,1,3), corners, mtx, dist)

        # 如果show參數為True，則繪製並顯示角點
        if show:
            # 繪製角點
            for i,corner in enumerate(corners):
                # 將角點座標轉換為整數
                x, y = tuple(map(int, corner[0]))
                # 使用cv2.circle在每個角點位置畫一個小圓圈
                if i ==0:
                    cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
                elif i ==5:
                    cv2.circle(img, (x, y), 5, (0, 255, 0), -1)
                elif i ==8:
                    cv2.circle(img, (x, y), 5, (0, 255, 255), -1)
                else:
                    cv2.circle(img, (x, y), 5, (255, 0, 0), -1)
            cv2.imshow('Chessboard Corners', img)
            cv2.waitKey(0)
  
        print('找尋完成。')
        return rvecs, tvecs
    else:
        print('找尋失敗，請確認棋盤格角點是否能被準確偵測。')
        return None, None
    
def parse_args():
    parse = argparse.ArgumentParser()
    parse.add_argument('--img_path', type=str, default="../raw_data/img/img_20231029_133301.jpg")
    parse.add_argument('--square_size', type=float, default=0.0485)
    parse.add_argument('--square_column', type=int, default=7)
    parse.add_argument('--square_row', type=int, default=4)
    parse.add_argument('--show','-sh', action="store_true")
    parse.add_argument('--img_size',  type=str, default="960_540")
    return parse.parse_args()

if __name__ == '__main__':
    
    # Show parameter
    args = parse_args()
    for key, value in vars(args).items():
        print(f"{key}: {value}")

    # Load corner points mtx, dist
    save_path= os.path.join('./cal_file/lidar_cam_cal',args.img_size)
    corner_points_np = load_array(os.path.join(save_path,"3dcorner.npy"))
    mtx, dist = read_camera_params(os.path.join('./cal_file/cam_cal',args.img_size,'ost.txt'))
    print('Camera matrix (mtx) : \n', mtx)
    print('Distortion coefficient (dist) : \n', dist)
 
    # Find rvecs, tvecs and calculate RT
    rvecs, tvecs = find_pose(args.img_path, mtx, dist, args.square_size, (args.square_column, args.square_row),corner_points_np,show=args.show)
    print('Rotation Vectors : \n', rvecs)
    print('Translation Vectors : \n', tvecs)
    RT ,R , T=vec_to_mat(rvecs, tvecs )

    # save rvecs, tvecs 
    with open(os.path.join(save_path,"rvecs_tvecs.txt") , "w") as file:
        np.savetxt(file, rvecs,fmt='%.8f')
        np.savetxt(file, tvecs,fmt='%.8f')







