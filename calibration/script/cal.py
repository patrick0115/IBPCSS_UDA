from func import *
import cv2
import matplotlib.pyplot as plt
img_folder_path = "cal_file/cam_cal/raw_img"
marked_img_folder_path = "cal_file/cam_cal/marked_img"

square_size = 0.0245
pattern_size = (8, 6)

# Perform camera calibration
mtx, dist, rvecs, tvecs , objpoints, imgpoints= camera_calibration(img_folder_path,marked_img_folder_path, square_size, pattern_size)
image = cv2.imread("cal_file/cam_cal/raw_img/left-0000.png")

# 儲存相機校正參數
write_txt(mtx, dist, 'cal_file.txt')

print("Camera Matrix : \n")
print(mtx)
print("Distortion Coefficients : \n")
print(dist)
print("Rotation Vectors : \n")
print(rvecs)
print("Translation Vectors : \n")
print(tvecs)
