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



def save_array(array, filename):
    # 使用numpy的save函式來儲存陣列
    np.save(filename, array)
    print(f'陣列已儲存到 {filename}')

def load_array(filename):
    # 使用numpy的load函式來讀取陣列
    array = np.load(filename)
    print(f'已從 {filename} 讀取陣列')
    return array


def get_objp( square_size, pattern_size):
    objp = np.zeros((np.prod(pattern_size), 3), dtype=np.float32)
    objp[:,:2] = np.mgrid[0:pattern_size[0],0:pattern_size[1]].T.reshape(-1,2)
   # 將物件點與方格尺寸進行縮放
    objp *= square_size

    return objp

def load_rvecs_tvecs(filepath):
    data = np.loadtxt(filepath)
    rvecs = data[:3].reshape(-1, 1)
    tvecs = data[3:].reshape(-1, 1)
    return rvecs, tvecs

def load_txt(filename):
    """
    Load camera matrix (mtx) and distortion coefficients (dist) from a txt file.

    Args:
        file_path (str): The path of the txt file.

    Returns:
        tuple: The camera matrix and distortion coefficients.
    """
    print('開始讀取檔案...')
    with open(filename, 'r') as f:
        lines = f.readlines()
        
    # Get the index of lines that start a new block
    indices = [i for i, line in enumerate(lines) if line.startswith('Camera') or line.startswith('Distortion')]
    
    # Load mtx
    mtx = []
    for line in lines[indices[0]+1:indices[1]-1]:
        mtx.append([float(val) for val in line.split()])
    mtx = np.array(mtx)
    
    # Load dist
    dist = np.array([float(val) for val in lines[indices[1]+1].split()]).reshape(1,-1)
    
    print('檔案讀取完成。')

    mtx = mtx.reshape(3, 3)
    
    print("Camera Matrix (mtx): ", mtx)
    print("Distortion Coefficients (dist): ", dist)

    return mtx, dist

def camera_calibration(img_folder_path,marked_img_folder_path, square_size, pattern_size):
    print('開始進行相機校正...')
    # 準備物件點，例如(0,0,0)，(1,0,0)，(2,0,0)...，(6,5,0)
    objp=get_objp(square_size, pattern_size)
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    # Get list of image files
    images = glob.glob(os.path.join(img_folder_path, "*.png"))

    for fname in images:
        img = cv2.imread(fname, cv2.IMREAD_GRAYSCALE)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
 
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(img, pattern_size, None)      
  
        # If found, add object points, image points
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, pattern_size, corners, ret)
            filename = os.path.basename(fname)
            cv2.imwrite(marked_img_folder_path+"/marked_" + filename, img)
        else:
            print("ret == False: ",fname)
            
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img.shape[::-1], None, None)

    return mtx, dist, rvecs, tvecs, objpoints, imgpoints

def write_txt(mtx, dist, filename):
    print('正在寫入檔案...')
    with open(filename, 'w') as f:
        f.write("Camera matrix (mtx): \n")
        for line in mtx:
            np.savetxt(f, line, fmt="%f")
        f.write("\n")
        
        f.write("Distortion coefficient (dist): \n")
        np.savetxt(f, dist, fmt="%f")
    print('檔案寫入完成。')
    
def test_corner(img_path,square_column,square_row,show=False):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    # 讀取影像
    img = cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 尋找棋盤格角點
    ret, corners = cv2.findChessboardCorners(gray, (square_column, square_row), flags=flags)
    if ret == True:
        if show:
            cv2.drawChessboardCorners(img, (square_column, square_row), corners, ret)
            cv2.imshow('Chessboard Corners', img)
            cv2.waitKey(0)
    
    return ret

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
            img = cv2.resize(img, (640, 640))
            cv2.imshow('Chessboard Corners', img)
            cv2.waitKey(0)
            print('角點圖像已保存為corners_image.png')
  
        print('找尋完成。')
        return rvecs, tvecs
    else:
        print('找尋失敗，請確認棋盤格角點是否能被準確偵測。')
        return None, None

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


def read_edit_pt(file_path,voxel_size=None,color=[0, 0, 0],x_range = None,y_range = None,z_range = None):
    # 讀取點雲數據
    pcd = o3d.io.read_point_cloud(file_path)
    print(f'原始點雲數量: {len(pcd.points)}')

    # 使用Voxel downsampling進行濾波
    if voxel_size != None:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        print(f'濾波後點雲數量: {len(pcd.points)}')

    # 設置點雲為黑色
    pcd.paint_uniform_color(color)

    # 裁剪點雲
    if x_range !=None and y_range !=None and z_range !=None :
        bound_min = [x_range[0], y_range[0], z_range[0]]
        bound_max = [x_range[1], y_range[1], z_range[1]]
        pcd = pcd.crop(
            o3d.geometry.AxisAlignedBoundingBox(min_bound=bound_min, max_bound=bound_max))

    return pcd
def pcd_to_numpy(pcd_file):
    # 讀取pcd檔
    pcd = o3d.io.read_point_cloud(pcd_file)

    # 轉換為 numpy 陣列
    np_points = np.asarray(pcd.points)

    return np_points
def pcd_to_bin(pcd_file, bin_file):
    # 讀取pcd檔
    pcd = o3d.io.read_point_cloud(pcd_file)

    # 轉換為 numpy 陣列
    np_points = np.asarray(pcd.points)

    # 儲存為 bin 檔
    np_points.astype('float32').tofile(bin_file)
def show_pcd(pcd_ls):
    # xyz軸
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.3, origin=[0, 0, 0])
    # 建立視窗物件並設置點的大小
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.get_render_option().background_color = np.asarray([0, 0, 0])  # 白色背景

    for i in range(len(pcd_ls)):
        vis.add_geometry(pcd_ls[i])
    vis.add_geometry(coordinate_frame)
    # 獲取渲染選項並設置點的大小
    render_option = vis.get_render_option()
    render_option.point_size = 5.0  # 設置點的大小

    # 顯示點雲
    vis.run()
    vis.destroy_window()
