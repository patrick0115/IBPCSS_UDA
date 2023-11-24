#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_srvs.srv import Trigger, TriggerResponse
import pcl
import os
from datetime import datetime

class PointCloudSaver:
    def __init__(self):
        # 從 ROS 參數伺服器獲取設定參數
        self.topic = rospy.get_param('~topic', 'colored_cloud')
        self.output_file = rospy.get_param('~output_file', 'colored_map.pcd')
        self.save_directory = rospy.get_param('~save_directory', '/tmp/')

        self.point_cloud_sub = rospy.Subscriber(self.topic, PointCloud2, self.callback)
        self.point_cloud_data = None
        self.service = rospy.Service('save_point_cloud', Trigger, self.handle_save_request)
        rospy.loginfo("服務已啟動。使用 'rosservice call /save_point_cloud' 來儲存點雲資料。")

    def callback(self, data):
        self.point_cloud_data = data
        rospy.loginfo("接收到點雲資料。")

    def handle_save_request(self, req):
        if self.point_cloud_data is None:
            return TriggerResponse(success=False, message="沒有可用的點雲資料。")
        
        try:
            # Convert PointCloud2 to PCL
            cloud_points = pc2.read_points(self.point_cloud_data, skip_nans=True, field_names=("x", "y", "z", "rgb"))
            cloud_pcl = pcl.PointCloud_PointXYZRGB()
            cloud_pcl.from_list(list(cloud_points))

            # Generate a timestamped filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            timestamped_filename = f"colored_map_{timestamp}.pcd"
            full_path = os.path.join(self.save_directory, timestamped_filename)

            # Save to PCD
            pcl.save(cloud_pcl, full_path)
            rospy.loginfo(f"點雲已成功儲存於 {full_path}")
            return TriggerResponse(success=True, message=f"點雲已成功儲存於 {full_path}")
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

if __name__ == '__main__':
    rospy.init_node('point_cloud_saver')
    saver = PointCloudSaver()
    rospy.spin()
