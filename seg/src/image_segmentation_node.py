#!/usr/bin/env python
import rospkg
import sys
rospack = rospkg.RosPack()
package_path = rospack.get_path('seg')
sys.path.append(package_path)
import torch
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from torchvision import transforms
from model.net.bisenetv2 import BiSeNetV2
from tools.palette import get_palette
import math
import numpy as np

# 初始化
bridge = CvBridge()
palette = get_palette()[0]
# 載入預訓練模型
net = BiSeNetV2(150)
net.load_state_dict(torch.load('/home/icalab/real_time_seg/src/rtseg/model/weight/bnv2_train_indoor_it_100000_b_6.pth', map_location='cpu'))
net.eval()
net.cuda()

# 圖像預處理設定
mean = [0.49343230, 0.46819794, 0.43106043]  # ade20k, rgb
std = [0.25680755, 0.25506608, 0.27422913]
transform = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(mean, std)
])

def callback(data):
    # 轉換ROS Image到OpenCV Image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    # 圖像預處理
    im = transform(cv_image).unsqueeze(0).cuda()
    org_size = im.size()[2:]
    new_size = [math.ceil(el / 32) * 32 for el in org_size]
    im = torch.nn.functional.interpolate(im, size=new_size, align_corners=False, mode='bilinear')

    # 執行分割
    out = net(im)[0]
    out = torch.nn.functional.interpolate(out, size=org_size, align_corners=False, mode='bilinear')
    out = out.argmax(dim=1).squeeze().detach().cpu().numpy()
   
    # print(np.unique(out))
 
    out = palette[out]
  
    # 將分割結果發布
    segmented_image_msg = bridge.cv2_to_imgmsg(out.astype(np.uint8), encoding="bgr8")
    segmented_image_msg.header = data.header
    pub.publish(segmented_image_msg)

if __name__ == '__main__':
    rospy.init_node('image_segmentation_node')
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    pub = rospy.Publisher('segmented_image', Image, queue_size=1)
    rospy.spin()
