#!/usr/bin/env python
# coding: utf-8

import rospy
import cv2
import pcl
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from datetime import datetime
import os
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
image_filename = "img_" + timestamp + ".jpg"
cloud_filename = "pcd_" + timestamp + ".pcd"


current_image = None
current_cloud = None

def image_callback(msg):
    global current_image
    try:
        bridge = CvBridge()
        current_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

def cloud_callback(msg):
    global current_cloud
    points_list = []

    for data in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
        points_list.append([data[0], data[1], data[2]])

    current_cloud = pcl.PointCloud()
    current_cloud.from_list(points_list)

def main():
    rospy.init_node('save_image_and_pcd', anonymous=True)

    # 從參數伺服器獲取儲存路徑
    img_save_path = rospy.get_param('img_save_path')
    pcd_save_path = rospy.get_param('pcd_save_path')

    # 訂閱相機的影像訊息
    image_topic = "/usb_cam/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)

    # 訂閱點雲數據
    cloud_topic = "/assemble_yrl"
    rospy.Subscriber(cloud_topic, PointCloud2, cloud_callback)

    print("按下Enter鍵保存當前的影像和點雲數據...")
    while not rospy.is_shutdown():
        input("按下Enter鍵保存...")
        if current_image is not None:
            if not os.path.exists(img_save_path):
                os.makedirs(img_save_path) # 創建目錄

            image_filename = os.path.join(img_save_path, "img_" + timestamp + ".jpg")
            cv2.imwrite(image_filename, current_image)
            print("影像已保存為", image_filename)

        if current_cloud is not None:
            if not os.path.exists(pcd_save_path):
                os.makedirs(pcd_save_path) # 創建目錄

            cloud_filename = os.path.join(pcd_save_path, "pcd_" + timestamp + ".pcd")
            pcl.save(current_cloud, cloud_filename)
            print("點雲數據已保存為", cloud_filename)
if __name__ == '__main__':
    main()
