#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge
import rospkg
import os
import time

# 全局變量來存儲最新的數據
latest_point_cloud = None
latest_image = None

def save_files(event):
    global latest_point_cloud, latest_image

    timestamp = time.strftime("%m%d%H%M%S")
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('calibration')
    img_save_path = os.path.join(package_path,"raw_data","img","img_"+timestamp+".jpg")
    pcd_save_path = os.path.join(package_path,"raw_data","pcd","pcd_"+timestamp+".pcd")

    # 檢查latest_point_cloud是否不為None
    if latest_point_cloud is not None:
        rospy.loginfo("Saving pcd data to %s", pcd_save_path)
        with open(pcd_save_path, 'w') as file:
            file.write('VERSION .7\n')
            file.write('FIELDS x y z\n')
            file.write('SIZE 4 4 4\n')
            file.write('TYPE F F F\n')
            file.write('COUNT 1 1 1\n')
            file.write('WIDTH {}\n'.format(len(latest_point_cloud)))
            file.write('HEIGHT 1\n')
            file.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            file.write('POINTS {}\n'.format(len(latest_point_cloud)))
            file.write('DATA ascii\n')
            for point in latest_point_cloud:
                file.write('{} {} {}\n'.format(point[0], point[1], point[2]))
        rospy.loginfo("PCD file saved successfully!")

    # 檢查latest_image是否不為None
    if latest_image is not None:
        rospy.loginfo("Saving image data to %s", img_save_path)
        cv2.imwrite(img_save_path, latest_image)
        rospy.loginfo("JPG file saved successfully!")

def point_cloud_callback(data):
    global latest_point_cloud
    latest_point_cloud = list(pc2.read_points(data, skip_nans=True))

def image_callback(data):
    global latest_image
    bridge = CvBridge()
    latest_image = bridge.imgmsg_to_cv2(data, "bgr8")


def main():


    rospy.init_node('save_files', anonymous=True, log_level=rospy.INFO)

    # 訂閱PointCloud2 topic
    rospy.Subscriber('/assemble_yrl', PointCloud2, point_cloud_callback)

    # 訂閱Image topic
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

    # 設置計時器，每5秒調用一次save_files函數
    rospy.Timer(rospy.Duration(1), save_files)

    # 主循環
    rospy.spin()

if __name__ == '__main__':
    main()