#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

def callback(data):
    # 創建一個新的 PointCloud2 消息
    filtered_points = []
    
    # 讀取點雲數據
    for point in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = point
        # 長方體過濾區域的邊界
        x_min, x_max = -2.8, 0.05
        y_min, y_max = -0.5, 0.5
        z_min, z_max = -0.7, 3.0
        # 長方體過濾條件
        if not (x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max):
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
    rospy.init_node('pointcloud_filter', anonymous=True)
    
    # 訂閱原始的 PointCloud2 主題
    rospy.Subscriber("/assemble_yrl", PointCloud2, callback)
    
    # 發布過濾後的 PointCloud2 主題
    pub = rospy.Publisher("/vehicle_filtered_pointcloud", PointCloud2, queue_size=1)

    rospy.spin()
