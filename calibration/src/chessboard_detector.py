#!/usr/bin/env python
# coding: utf-8

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def image_callback(msg):
    try:
        # 將ROS影像訊息轉換為OpenCV格式
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # 轉換為灰度圖像
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 使用高斯平滑減少噪聲
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # 使用自適應閾值進行二值化
    gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

    # 使用cv2.findChessboardCorners檢測校正板角點
    found, corners = cv2.findChessboardCorners(gray, chessboard_size)

    if found:
        # print("找到校正板的角點")
        # 畫出角點
        cv2.drawChessboardCorners(image, chessboard_size, corners, found)
        # 將帶有角點的圖片轉換為ROS影像訊息並發布
        image_msg = bridge.cv2_to_imgmsg(image, "bgr8")
        pub.publish(image_msg)

    # # 顯示影像
    # cv2.imshow('Camera', image)
    # cv2.waitKey(1)





def main():
    global pub, chessboard_size
    rospy.init_node('chessboard_detector', anonymous=True)

    # 從參數伺服器獲取校正板尺寸
    chessboard_size = (rospy.get_param('~rows'), rospy.get_param('~cols'))

    # 訂閱相機的影像訊息
    image_topic = "/usb_cam/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)

    # 創建一個新的主題來發布帶有角點的校正板圖片
    pub = rospy.Publisher('/image_chess', Image, queue_size=10)

    print("正在檢測校正板的角點...")
    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()