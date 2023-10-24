from func import *
import open3d as o3d
import argparse

def parse_args():
    parse = argparse.ArgumentParser()
    # parse.add_argument('--pcd_path', type=str, default="../raw_data/1020/pcd/pcd_20231020_174517.pcd")
    parse.add_argument('--img_path', type=str, default="../raw_data/img/img_20231023_171126.jpg")
    parse.add_argument('--square_size', type=float, default=0.0485)
    parse.add_argument('--square_column', type=int, default=7)
    parse.add_argument('--square_row', type=int, default=4)

    return parse.parse_args()

if __name__ == '__main__':
    args = parse_args()

    save_path= os.path.join('./cal_file/lidar_cam_cal')
    corner_points_np = load_array(os.path.join(save_path,"3dcorner.npy"))


    # 1. Loadmtx, dist
    mtx, dist = read_camera_params('./cal_file/cam_cal/ost.txt')


    print('Camera matrix (mtx) : \n', mtx)
    print('Distortion coefficient (dist) : \n', dist)
    print(corner_points_np)
    # 2. Find rvecs, tvecs 
    rvecs, tvecs = find_pose(args.img_path, mtx, dist, args.square_size, (args.square_column, args.square_row),corner_points_np,show=True)
    print('Rotation Vectors : \n', rvecs)
    print('Translation Vectors : \n', tvecs)
    RT ,R , T=vec_to_mat(rvecs, tvecs )
    print(save_path)
    with open(os.path.join(save_path,"rvecs_tvecs.txt") , "w") as file:
        
        np.savetxt(file, rvecs,fmt='%.8f')
        np.savetxt(file, tvecs,fmt='%.8f')







