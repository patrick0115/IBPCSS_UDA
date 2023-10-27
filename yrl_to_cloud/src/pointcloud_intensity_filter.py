#!/usr/bin/env python

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

def callback(data):
    points = []
    intensities = []
    
    # 讀取點雲數據和強度
    for point in pc2.read_points(data, field_names=("x", "y", "z", "intensity"), skip_nans=True):
        x, y, z, intensity = point
        points.append([x, y, z, intensity])
        intensities.append(intensity)
    
    if len(intensities) == 0:
        return

    # 計算強度的平均值和標準差
    mean_intensity = np.mean(intensities)
    std_intensity = np.std(intensities)
    
    lower_bound = mean_intensity - std_intensity
    upper_bound = mean_intensity + std_intensity
    
    # 過濾點雲
    filtered_points = [point for point in points if lower_bound <= point[3] <= upper_bound]
    
    # 創建新的 PointCloud2 消息
    fields = [pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
              pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
              pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
              pc2.PointField('intensity', 12, pc2.PointField.FLOAT32, 1)]
    
    header = data.header
    filtered_cloud = pc2.create_cloud(header, fields, filtered_points)

    # 發布過濾後的點雲
    pub.publish(filtered_cloud)

if __name__ == '__main__':
    rospy.init_node('pointcloud_intensity_filter', anonymous=True)
    
    # 訂閱原始的 PointCloud2 主題
    rospy.Subscriber("/assemble_yrl", PointCloud2, callback)
    
    # 發布過濾後的 PointCloud2 主題
    pub = rospy.Publisher("/filtered_intensity_pointcloud", PointCloud2, queue_size=1)

    rospy.spin()
