from func import *
import open3d as o3d

# Parameter  and file
square_size = 0.0218
pattern_size = (17, 12)


img_path="../raw_data/img/S__179929090.jpg"

corner_points_np=get_objp( square_size, pattern_size)

# 1. Loadmtx, dist
mtx, dist = load_txt('cal_file.txt')
print('Camera matrix (mtx) : \n', mtx)
print('Distortion coefficient (dist) : \n', dist)
# 2. Find rvecs, tvecs 
rvecs, tvecs = find_pose(img_path, mtx, dist, square_size, pattern_size,corner_points_np,show=True)



