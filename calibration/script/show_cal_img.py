import cv2
import numpy as np
import yaml
from func import *



def overlay_images(camera_matrix, dist_coeff, img):
    # 自定義標點顏色和標線顏色（以 BGR 格式）
    # board_size=(12,9)
    board_size=(17,12)

    ret, corners = cv2.findChessboardCorners(img, board_size, None)
    height, width = img.shape

    # 如果找到角點，則標記並進行校正
    if ret:
        # 標記棋盤格角點
        cv2.drawChessboardCorners(img, board_size, corners, ret)
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeff, (width, height), 0)
        print(new_camera_matrix)
        print(camera_matrix)
        # 進行圖像校正
        undistorted_img = cv2.undistort(img, camera_matrix, dist_coeff, None, new_camera_matrix)
        
        # 將校正後的圖像和原始圖像疊合
        alpha = 0.2  # 設定透明度
        overlay_img = cv2.addWeighted(undistorted_img, alpha, img, 1 - alpha, 0)
     
        # 顯示疊合後的圖像
        cv2.imshow('Overlay Image', undistorted_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("找不到足夠的棋盤格角點")



if __name__ == '__main__':
    camera_matrix, dist_coeff = read_camera_params("./cal_file/cam_cal/ost.txt")
    image = cv2.imread("./cal_file/cam_cal/left-0024.png",cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


    overlay_images(camera_matrix, dist_coeff, gray)

