import cv2
import numpy as np
import glob
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d
from scipy.spatial.distance import euclidean

def create_path_if_not_exists(path):
    # Check if the path exists
    if os.path.exists(path):
        print(f"{path} path already exists.")
    else:
        print(f"{path} path does not exist, creating...")
        os.makedirs(path)
        print(f"{path} path has been successfully created.")

def load_array(filename):
    # 使用numpy的load函式來讀取陣列
    array = np.load(filename)
    print(f'已從 {filename} 讀取陣列')
    return array

def load_rvecs_tvecs(filepath):
    data = np.loadtxt(filepath)
    rvecs = data[:3].reshape(-1, 1)
    tvecs = data[3:].reshape(-1, 1)
    return rvecs, tvecs


def read_camera_params(file_path):
    # 初始化空的列表來存儲camera matrix和distortion的值
    camera_matrix_list = []
    distortion_list = []

    # 打開和讀取txt檔
    with open(file_path, "r") as f:
        lines = f.readlines()

    # 解析每一行
    in_camera_matrix_section = False
    in_distortion_section = False

    for line in lines:
        line = line.strip()  # 去掉首尾的空白字符

        # 判斷是否進入特定的section
        if line == "camera matrix":
            in_camera_matrix_section = True
            continue
        elif line == "distortion":
            in_distortion_section = True
            continue

        # 讀取數據並加入到對應的列表中
        if in_camera_matrix_section and line:
            row = list(map(float, line.split()))
            camera_matrix_list.append(row)

            # 如果camera matrix已經有3行了，那就退出這個section
            if len(camera_matrix_list) == 3:
                in_camera_matrix_section = False

        if in_distortion_section and line:
            distortion_list = list(map(float, line.split()))

            # 退出這個section
            in_distortion_section = False

    # 將列表轉換為numpy.ndarray
    camera_matrix_np = np.array(camera_matrix_list)
    distortion_np = np.array([distortion_list])

    print("讀取完畢")
    return camera_matrix_np, distortion_np


