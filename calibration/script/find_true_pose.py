from func import *
import numpy as np
import open3d as o3d
import os
import argparse

def create_rectangle(length, width):
    # 定義4個點
    points = np.array([
        [0,-length/2, -width/2],
        [0,length/2, -width/2],
        [0,length/2, width/2],
        [0,-length/2, width/2]
    ])
    return points

def create_corner(rows=9, columns=6, spacing=0.0245):
    points = np.zeros((rows*columns, 3))
    y, z = np.meshgrid(np.linspace(-spacing*(rows-1)/2, spacing*(rows-1)/2, rows),
                    np.linspace(-spacing*(columns-1)/2, spacing*(columns-1)/2, columns))
    points[:, 1:3] = np.stack((y.reshape(-1), z.reshape(-1)), axis=-1)

    return points 
def move_points(points,origin=[0,0,0],rotate_angle=[85,90,90]):

    points += np.array(origin)

    points = rotate_points(points, 'x', np.radians(rotate_angle[0]),origin)
    points = rotate_points(points, 'y', np.radians(rotate_angle[1]),origin)
    points = rotate_points(points, 'z', np.radians(rotate_angle[2]),origin)

    return points    

def rotate_points(points, axis, angle ,center):
    # 將點雲的坐標系移動到旋轉中心
    points_centered = points - center
    # 創建旋轉矩陣
    c, s = np.cos(angle), np.sin(angle)
    if axis == 'x':
        R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    elif axis == 'y':
        R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    elif axis == 'z':
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    # 將旋轉矩陣應用於每個點
    points_rot_centered = np.dot(points_centered, R.T)

    # 將點雲的坐標系移動回原來的位置
    points_rot = points_rot_centered + center
    return points_rot
def draw_rectangle(points):
    # 定義線段，連接4個點形成矩形
    lines = [[0, 1], [1, 2], [2, 3], [3, 0]]

    # 創建LineSet對象
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_color=[1, 0, 0]
    line_set.colors = o3d.utility.Vector3dVector([line_color for _ in range(len(lines))])

    return line_set

def draw_corner(points):

    ## show pcd
    created_pcd = o3d.geometry.PointCloud()
    created_pcd.points = o3d.utility.Vector3dVector(corner_points)
    # 將第一組點表示為小球
    colors = np.zeros((args.square_column*args.square_row, 3)) 
    colors[:] = [0, 0, 1]  # 將所有點設置為藍色
    colors[0] = [1, 0, 0]  # 將索引為 0 的點設置為紅色
    colors[5] = [0, 1, 0]  # 將索引為 5 的點設置為綠色
    colors[8] = [0, 1, 1]  # 將索引為 8 的點設置為青色
    
    created_pcd.colors = o3d.utility.Vector3dVector(colors)

    return created_pcd

def parse_args():
    parse = argparse.ArgumentParser()
    parse.add_argument('--pcd_path', type=str, default="../raw_data/pcd/pcd_20230824_170141.pcd")
    parse.add_argument('--img_path', type=str, default="../raw_data/img/img_20230824_170141.jpg")
    parse.add_argument('--square_size', type=float, default=0.04855)
    parse.add_argument('--square_column', type=int, default=7)
    parse.add_argument('--square_row', type=int, default=4)
    parse.add_argument('--length', type=float, default=0.42)
    parse.add_argument('--width', type=float, default=0.297)
    parse.add_argument('--save','-s', action="store_true")
    parse.add_argument('--show','-sh', action="store_true")
    return parse.parse_args()

if __name__ == '__main__':
    args = parse_args()

    assert test_corner(args.img_path,args.square_column,args.square_row,args.show) ,"Unable to find corners. Please use another image."


    # 設定原始點雲 x、y、z三軸的範圍，範圍的格式為[min, max]
    x_range = [ 0.4, 1.5]
    y_range = [-0.4, 0.4]
    z_range = [0, 1.5]
    
    # 設定原校正版位置
    origin=[1.29,-0.06,0.24]
    rotate_angle=[0, 5,-3]
    
    # 創建校正板點雲numpy
    corner_points_np = create_corner(rows=args.square_column, columns=args.square_row, spacing=args.square_size)
    line_points_np = create_rectangle(length=args.length,width=args.width)
    
    # 移動並旋轉
    corner_points=move_points(points=corner_points_np,origin=origin,rotate_angle=rotate_angle)
    line_points=move_points(points=line_points_np,origin=origin,rotate_angle=rotate_angle)

    # 創建為open3d
    line_points=draw_rectangle(line_points)
    corner_points=draw_corner(corner_points)

    # 創建原始點雲
    pcd_path=args.pcd_path
    pcd_name=pcd_path.split("/")[-1][0:-4]
    org_pcd=read_edit_pt(pcd_path,color=[1, 1, 1],x_range = x_range,y_range = y_range,z_range = z_range)

    # 顯示點雲
    pcd_ls=[org_pcd,corner_points,line_points]
    show_pcd(pcd_ls)

    
    if  args.save :
        save_path=os.path.join("./cal_file/lidar_cam_cal",pcd_name)
        create_path_if_not_exists(save_path)
        
        with open(os.path.join(save_path,pcd_name+"_org_rot.txt") , "w") as file:
            np.savetxt(file, origin,fmt='%.2f')
            np.savetxt(file, rotate_angle,fmt='%.2f')

        save_array(corner_points_np, os.path.join(save_path,pcd_name+"_3dcorner.npy"))


