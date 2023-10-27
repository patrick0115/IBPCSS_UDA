#!/usr/bin/env python
# coding: utf-8

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

def point_cloud_callback(msg):
    # 讀取點雲資料
    gen = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
    
    # 過濾intensity太小的點
    # print("開始過濾點雲...")
    filtered_points = [point for point in gen if point[3] > intensity_threshold]
    
    # 創建一個新的PointCloud2訊息
    filtered_cloud = pc2.create_cloud_xyz32(msg.header, [(point[0], point[1], point[2]) for point in filtered_points])
    
    # 發布過濾後的點雲
    pub.publish(filtered_cloud)
    # print("過濾完成，已重新發布到新的topic。")

if __name__ == "__main__":
    # 初始化節點
    rospy.init_node('point_cloud_filter', anonymous=True)
    
    # 讀取參數
    input_topic = rospy.get_param("~input_topic", "/input_point_cloud")
    output_topic = rospy.get_param("~output_topic", "/filtered_point_cloud")
    intensity_threshold = rospy.get_param("~intensity_threshold", 50.0)
    
    # 設定訂閱者和發布者
    rospy.Subscriber(input_topic, PointCloud2, point_cloud_callback)
    pub = rospy.Publisher(output_topic, PointCloud2, queue_size=1)
    
    print("節點正在運行，等待接收點雲資料...")
    rospy.spin()
