#!/usr/bin/env python
# coding: utf-8

import rospy
from sensor_msgs.msg import PointCloud2
def publish_empty_msg(event):
    empty_msg = PointCloud2()
    pub.publish(empty_msg)
def callback(data):
    # 打印進度
    
    # print("接收到點雲資料，時間戳：{}".format(data.header.stamp))

    # 檢查是否包含 'intensity' 字段
    has_intensity = any(field.name == 'intensity' for field in data.fields)

    # if not has_intensity:
    #     print("此點雲資料缺少 'intensity' 字段，將被過濾掉。")
    #     return
    # 這裡處理包含 'intensity' 的點雲資料
    # ...

    # 可以選擇重新發布到新的 topic
    pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('filter_pointcloud', anonymous=True)
    
    # 訂閱原始的點雲資訊
    rospy.Subscriber("/assemble_yrl", PointCloud2, callback)
    
    # 創建一個新的發布者，用於發布過濾後的資料
    pub = rospy.Publisher("/filtered_assemble_yrl", PointCloud2, queue_size=10)
    rospy.Timer(rospy.Duration(1), publish_empty_msg)

    rospy.spin()
