---
tags: research
---


# UDA_PCSS 

[TOC]

## 簡介

`UDA_PCSS` 是一個ROS工作區，專為機器人操作和控制系統開發。這個工作區包含從[IBPCSS_UDA](https://github.com/patrick0115/IBPCSS_UDA) GitHub倉庫匯入的多個ROS包。

### 主要目錄和包

- `LeGO-LOAM`: 用於LiDAR點雲處理和地圖構建。
- `calibration`: 包含用於感測器校準的腳本和配置文件。
- `yrl_to_cloud`: 專為點雲數據轉換設計的ROS包。
- `yujin_yrl_v2_package`: 包含Yujin Robot的第二版YRL驅動和相關配置。

---

## 硬體需求

- 電腦：具有至少8GB RAM和Intel i5處理器（或等效）的電腦。
- 感測器：LiDAR、RGB相機等（如果適用）。
- 其他：足夠的USB端口、網絡連接等。

---

## 安裝

### 環境設置

1. 安裝ROS（Robot Operating System）。

    - [ubuntu 16.04 -> kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
    - [ubuntu 18.04 -> melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
    - [ubuntu 20.04 -> noetic ](http://wiki.ros.org/noetic/Installation/Ubuntu)

### 下載和編譯

1. 克隆這個倉庫到您的ROS工作區的`src`目錄。
    ```bash
    cd ~/UDA_PCSS/src
    git clone https://github.com/patrick0115/IBPCSS_UDA.git
    ```

2. 編譯工作區。
    ```bash
    cd ~/UDA_PCSS
    catkin_make
    ```

---

## 運行

### 運行AMR

這部分將會提供如何運行自主移動機器人（AMR）的指南。具體的運行命令和參數將會在這裡詳細說明。
1. AMR 開機
2. 將筆電網路線接到AMR
3. SSH
- 有線
```
ssh c01@192.168.10.200
```
- 無線
```
ssh c01@192.168.50.16
```
4. AMR啟動
```
roslaunch BaseNode base_connect.launch
```
6. 開啟搖桿控制
```
roslaunch BaseNode joy_control.launch
```
![](https://hackmd.io/_uploads/SyESytVC2.png)
![](https://hackmd.io/_uploads/BJjS1tNC3.png)

### 運行相機

這部分將會提供如何運行相機並獲取影像數據的指南。包括如何設置相機參數，以及如何觀察相機的實時輸出。

### 運行光達

這部分將會提供如何運行LiDAR並獲取點雲數據的指南。這裡會解釋如何配置和校準LiDAR，以及如何使用ROS來接收點雲數據。

#### 安裝及下載
[yujin_lidar_v2 github](https://github.com/yujinrobot/yujin_lidar_v2)

#### 光達運行

- 用ros運行
```
roscore 
rosrun yujin_yrl_v2_package yrl_pub
```
- 用ros運行
```b!
roslaunch yujin_yrl_v2_package yujin_yrl_v2.launch
```

- 用內建viewer運行
```
sudo -H ./Yujin_Lidar_Viewer.sh
```

- 看點雲資訊 /yrl_pub/yrl_cloud
```
rostopic echo /yrl_pub/yrl_cloud
```


#### 光達TF
- 單運行光達的TF
![](https://hackmd.io/_uploads/SkbFdgP1a.png)

### 紀錄資料
#### 紀錄點雲照片為rosbag檔
1. 啟動光達,相機,光達累積程式及rosbag紀錄：
    ```bash!
    roslaunch yrl_to_cloud assemble_to_bag.launch assemble_yrl assemble_yrl:=/assemble_yrl  mage_raw:=/usb_cam/image_raw
    ```
2. 開始紀錄rosbag/停止紀錄rosbag
    ```
    rosservice call /start_stop_record
    ```
3. 參數可在 assemble_to_bag.launch內調整
:::success
bag_file_path : bag檔儲存位置 
assemble_yrl : 累積後的光達
image_raw : 圖片
yrl_cloud : 原始光達
tf : TF座標
topics_to_record : bag檔要記錄的topic
:::
#### 儲存rosbag檔內特定時間的照片及點雲
1. **啟動ROS和rviz視覺化工具**
    這將會播放指定的bag檔案，並在rviz中進行視覺化。

    ```bash!
    roslaunch calibration bag_to_img_pcd.launch bag_file:=<your_bag_file_path>
    ```

2. **儲存當前的 pcd 和 jpg 檔案**

    在另一個終端機視窗中，執行以下命令。這將會觸發一個ROS服務，將當前播放的bag檔案中的pcd和jpg檔案儲存到預設的目錄。

    ```bash!
    rosservice call /save_data
    ```

:::success
**注意：** 預設的儲存位置為：
`/UDA_PCSS/src/IBPCSS_UDA/calibration/raw_data`
:::


#### 運行光達,相機並儲存特定時間的影像及點雲
1.  **啟動ROS和rviz視覺化工具**
    這將會開啟光達及相機，並在rviz中進行視覺化。
    ```bash!
    roslaunch calibration capture_image_and_cloud.launch
    ```
:::success
這會執行校正板偵測節點，若畫面中有指定大小校正板則會顯示
capture_image_and_cloud.launch內rows,cols參數為校正板行列
:::
2. **儲存當前的 pcd 和 jpg 檔案**

    在另一個終端機視窗中，執行以下命令。這將會觸發一個ROS服務，將當前播放的bag檔案中的pcd和jpg檔案儲存到預設的目錄。

    ```bash!
    rosservice call /save_data
    ```
### 相機校正

這一部分提供了如何校正您的相機的詳細步驟。校正是確保相機輸出可用於後續處理和分析的重要步驟。

#### 校正板下載
下載並印出這個PDF文件，確保尺寸正確
- [校正板PDF_1](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=view&target=check-108.pdf)
- [校正板PDF_2](https://markhedleyjones.com/projects/calibration-checkerboard-collection)

#### 校正步驟

1. **確定校正板尺寸**

    測量校正板上的方格尺寸，這將用於校正腳本。

2. **插上相機**

    確保您的相機已正確連接並開啟。

3. **確認校正板可被偵測（可略過）**

    在這一步，您可以先確認相機是否能正確偵測到校正板。`cols` 和 `rows` 是校正板的列數和行數。

    ```bash!
    roslaunch calibration chessboard_detector.launch cols:=9 rows:=6
    ```

4. **開始正式校正**

    使用以下命令進行校正。請根據您的實際情況調整 `camera_topic`、`image_topic`、`size` 和 `square` 參數。

    ```bash!
    roslaunch calibration camera_calibrator.launch camera_topic:=/usb_cam image_topic:=/usb_cam/image_raw size:=8x6 square:=0.02485
    ```

### 相機-光達校正

這部分將會提供如何進行相機和LiDAR之間的校正的指南。

#### 從bag檔案擷取 pcd 和 jpg 檔案
1. [運行光達,相機並儲存特定時間的影像及點雲](####運行光達,相機並儲存特定時間的影像及點雲)
    先將指定大小校正板放在相機拍攝範圍，並確保收集足夠密集的點雲資料後，將其儲存為影像及點雲檔
2. **尋找相機外參**
    讀取1.收集到的pcd及jpg並進行外參校正
```
cd ./src/IBPCSS_UDA/calibration/script/
```
```
python find_true_pose.py 
```

- 參數調整

```bash!
pcd_path: ../raw_data/pcd/pcd_20230824_170141.pcd
img_path: ../raw_data/img/img_20230824_170141.jpg
square_size: 0.04855
square_column: 7
square_row: 4
length: 0.42
width: 0.297
save: False
show: False
```

4. **投影**
```
python pro_2d.py
```

- 參數調整
```
pcd_path: ../raw_data/pcd/pcd_20230824_170141.pcd
img_path: ../raw_data/img/img_20230824_170141.jpg

```

- 位置校正

```b!
# 設定原始點雲 x、y、z三軸的範圍，範圍的格式為[min, max]
x_range = [ 0.4, 1.5]
y_range = [-0.4, 0.4]
z_range = [0, 1.5]

# 設定原校正版位置
origin=[1.29,-0.06,0.24]
rotate_angle=[0, 5,-3]
```




---
## 參考資料
### [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)
#### LeGO-LOAM 參數


1. N_SCAN
線數
2. groundScanInd 
照到地面的線數
3. ang_res_x ,ang_res_y
水平,垂直解析度 
4. Horizon_SCAN
旋轉一週採樣次數
5. ang_bottom
水平往下的視角角度



 

## 聯繫方式


---
