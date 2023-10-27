#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

def callback(data):
    # 創建一個新的 PointCloud2 消息
    filtered_points = []
    
    # 定義點雲保留範圍
    x_min, x_max = 0.0, 0.5
    y_min, y_max = -0.3, 0.3
    z_min, z_max = 0.2, 1.0
    
    # 讀取點雲數據
    for point in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = point
        # 長方體過濾條件
        if x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max:
            filtered_points.append([x, y, z])
    
    # 輸出過濾後的點雲數量
    print("過濾後的點雲數量：", len(filtered_points))

    # 創建過濾後的 PointCloud2
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]

    header = data.header
    filtered_data = pc2.create_cloud(header, fields, filtered_points)

    # 發布過濾後的 PointCloud2
    pub.publish(filtered_data)

if __name__ == '__main__':
    rospy.init_node('pointcloud_range_filter', anonymous=True)
    
    # 訂閱原始的 PointCloud2 主題
    rospy.Subscriber("/assemble_yrl", PointCloud2, callback)
    
    # 發布過濾後的 PointCloud2 主題
    pub = rospy.Publisher("/range_filtered_pointcloud", PointCloud2, queue_size=1)

    rospy.spin()
