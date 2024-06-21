# target-based-calibration

## 1. 软件介绍

基于矩形标定板能够准确地确定激光雷达和相机之间的空间位置关系，从而实现精准的外参标定。在当前的版本中，提供了人工辅助的简单操作过程。

### 1.1 依赖安装

```bash
# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

### 1.2 编译

```bash
mkdir build && cd build
cmake ..
make -j8
```

### 1.3 运行

```bash
# demo
bin/lidar_camera_calibrator data/data-velo-sim
```




### 2. 标定流程
![image](doc/a9907fce31514ad7b62549bb8872ccf43067.png)

## 3.标定实操

### 3.1 标定数据采集 (rosbag)

用户需要倾斜地拿着标定板进行数据采集。在进行数据采集时，用户可能需要变换若干次方位(3-5次)，确保在整个共同视野范围内都有充分的数据被捕获，为外参标定提供更为准确的结果。

### 3.2 标定数据筛选

挑选出图像清晰的数据（3-5组）用于标定，如下图所示。

![image](doc/55d61adf-9a12-45ed-b9cb-29651ca57e59.jpeg)
![image](doc/66d8e21f-8cb6-4750-a453-d03005e80bce.jpeg)
![image](doc/e9f9bfc1-cc06-44e0-a7b4-246038d235e4.jpeg)

### 3.3 配置文件填写

修改配置文件（config.json）中的畸变系数、相机内参和标定板大小。

![image](doc/d5ecfd46-49ee-4aaa-bba8-0131a39cf9b6.png)

![image](doc/d112c018-2122-4a0b-a4f3-fefc2181ebd6.png)

### 3.4 外参标定

1.  打开标定软件，并指定标定数据根目录。

![image](doc/e2cd60cd-0ee1-4ce0-bbe2-c13a7d2e0a1b.png)

2.  图像和点云中的标定板的角点提取。

图像中的标定板角点将自动提取，如下图所示。若无marker，需要手动选择（start开始选择，finish结束）。

![image](doc/c0f824a4-9029-4721-9d48-c25f5ae71a05.png)

点云中需人工操作将标定板过滤出来，随后点击extract，即可完成角点提取。如下图所示。

![image](doc/037789a2-4529-4bd8-9cdd-ca95e72a55c8.png)
![image](doc/e3d8fd98-4b6e-4128-90d5-bef081cafdbc.png)

### 3.5 标定结果输出及可视化验证

待完成所有数据的角点提取后，点击标定（calibrate）按钮即可完成标定，此时终端将输出标定结果，可视化界面将显示点云投影至图像以及点云着色的效果。标定结果也将写入至配置文件当中。

![image](doc/de71298d-7272-467b-998f-5d0cc2305621.png)
![image](doc/643d8a13-24b2-4219-8885-27aedaccd42f.png)



![image](doc/7a104b19-89ed-46b0-8bd9-2bc0b4ff98b7.png)
![image](doc/f2734f5b-e02d-4ee8-8327-8538b60c56d8.png)



## 4. 致谢

感谢以下开源仓库~

https://github.com/HITSZ-NRSL/lidar_camera_calibrator

https://github.com/beltransen/velo2cam_calibration
