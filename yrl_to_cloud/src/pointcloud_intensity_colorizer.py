#!/usr/bin/env python

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

def callback(data):
    points = []
    intensities = []
    
    # 讀取點雲數據和強度
    for point in pc2.read_points(data, field_names=("x", "y", "z", "intensity"), skip_nans=True):
        x, y, z, intensity = point
        points.append([x, y, z, intensity])
        intensities.append(intensity)
    if len(intensities) > 0:
        # 計算強度的平均值和標準差
        mean_intensity = np.mean(intensities)
        std_intensity = np.std(intensities)
        
        lower_bound = mean_intensity - std_intensity
        upper_bound = mean_intensity + std_intensity
    
    # 根据強度上色
    colored_points = []
    for x, y, z, intensity in points:
        rgb = 0  # 默認為黑色
        if intensity < lower_bound:
            rgb = 0x0000FF  # 藍色
        elif intensity > upper_bound:
            rgb = 0xFF0000  # 紅色
        else:
            rgb = 0x00FF00  # 綠色
            
        colored_points.append([x, y, z, rgb])
    
    # 創建新的 PointCloud2 消息
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgb', 12, PointField.UINT32, 1)]

    header = data.header
    colored_cloud = pc2.create_cloud(header, fields, colored_points)

    # 發布上色後的點雲
    pub.publish(colored_cloud)

if __name__ == '__main__':
    rospy.init_node('pointcloud_intensity_std_colorizer', anonymous=True)
    
    # 訂閱原始的 PointCloud2 主題
    rospy.Subscriber("/assemble_yrl", PointCloud2, callback)
    
    # 發布上色後的 PointCloud2 主題
    pub = rospy.Publisher("/std_colored_pointcloud", PointCloud2, queue_size=1)

    rospy.spin()
