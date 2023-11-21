#!/usr/bin/env python
# coding: utf-8

import rospy
import subprocess
from std_srvs.srv import Trigger, TriggerResponse
from datetime import datetime
import os
class RosbagRecorder:
    def __init__(self):
        self.process = None
        assemble_yrl_topic = rospy.get_param('~assemble_yrl', '/assemble_yrl')
        image_raw_topic = rospy.get_param('~image_raw', '/usb_cam/image_raw')
        tf_topic = rospy.get_param('~tf', '/tf')
        yrl_cloud_topic = rospy.get_param('~/yrl_cloud', '/yrl_pub/yrl_cloud')
        enable_seg_record = rospy.get_param('~enable_seg_record', 'true')
        if enable_seg_record:
            segmented_image_topic = rospy.get_param('~segmented_image', '/segmented_image')
            self.topics = [assemble_yrl_topic, image_raw_topic, tf_topic,yrl_cloud_topic,segmented_image_topic]
        else:
            self.topics = [assemble_yrl_topic, image_raw_topic, tf_topic,yrl_cloud_topic]
        
        print("要錄製的主題：", self.topics)
        
        self.service = rospy.Service('start_stop_record', Trigger, self.handle_record_request)
        print("服務初始化完成，等待錄製請求，請執行rosservice call /start_stop_record ")
        # 從參數伺服器獲取儲存資料夾
        self.bag_file_path = rospy.get_param('~bag_file_path')
        print("儲存資料夾：", self.bag_file_path)
    def handle_record_request(self, req):
        response = TriggerResponse()
        
        if self.process is None:
            # 生成當前的時間戳
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            bag_file_name = "bag_" + timestamp + ".bag"
            
            # 完整的儲存路徑
            full_bag_file_path = os.path.join(self.bag_file_path, bag_file_name)
            
            # 啟動 rosbag record
            self.process = subprocess.Popen(["rosbag", "record", "-O", full_bag_file_path] + self.topics)
            response.success = True
            response.message = "錄製已開始，保存到 " + full_bag_file_path
            print("錄製已開始，保存到：", full_bag_file_path)
        else:
            # 停止 rosbag record
            self.process.terminate()
            self.process = None
            response.success = True
            response.message = "錄製已停止。"
            print("錄製已停止。")
        
        return response

if __name__ == '__main__':
    rospy.init_node('rosbag_record_service')
    recorder = RosbagRecorder()
    rospy.spin()
