# Calibration_Tutorial
## 1. Introduction
This tutorial of calibration is for the beginners. We propose a multi-sensor system on a car and implement the state-of-art calibration methods. The steps and results are shown below.
<div align="center">
    <img src="pic/1.jpg" width = 40% >  
    <font color=#a0a0a0 size=2><br>(a) Proposed multi-sensor system (b) Robosense bpearl blind spot LiDAR (c) Livox Avia solite-state dense LiDAR (d) Intel Realsense D455 depth camera (e) Intel Realsense L515 depth camera</font>
</div>


## 2. Calibration of binocular cameras
This work is based on [MATLAB_Stereo_camera_calibrator](https://www.mathworks.com/help/vision/ref/stereocameracalibrator-app.html)
1. Prepare the chess board and measure the square size
2. Keep the system stable and record >20 ROS bags containing the  images from the camera D455 and camera L515 

```
rosbag record -O xxx.bag /camera_topic1 /camera_topic2
```

3. Extract images from the bags (run the command below and play the rosbag)
```
rosrun image_view extract_images _sec_per_frame:=0.5 image:=/camera_topic
```
4. After repeating the steps above, we get >20 pairs of macthed images. Our sample data is in /data/binocular

<div align="center">
    <img src="pic/2.png" width = 40% >
    <font color=#a0a0a0 size=2><br>Image from D455 camera</font>
</div>

<div align="center">
    <img src="pic/3.png" width = 40% >
    <font color=#a0a0a0 size=2><br>Image from L515 camera</font>
</div>

5. Use the MATLAB toolbox to calibrate the two cameras
6. Remove the image pairs with large error


## 3. Targetless calibration of Livox LiDAR and D455 camera
This work is based on [Targetless lidar_camera_calib](https://github.com/hku-mars/livox_camera_calib)
1. Set up the environment and follow the [steps](https://github.com/hku-mars/livox_camera_calib)

2. Record ROS bag containing Lidar and camera messages for more than 20s to accumulate dense point cloud. Then, change the bag file into pcd file. The tool bag_to_pcd can be found in[Targetless lidar_camera_calib](https://github.com/hku-mars/livox_camera_calib). After modify the path config file,
```
roslaunch livox_camera_calib bag_to_pcd.launch
```
We can get the accumulated point cloud. Our sample data set is in  /data/livox_camera_targetless. Because limited upload file size, we down sample the uploaded point cloud.

3. Edit the file path in the yaml file, then run the program and wait the result
<div align="center">
    <img src="pic/4.jpg" width = 40% >
    <font color=#a0a0a0 size=2><br>Targetless calibration scene</font>
</div>
<div align="center">
    <img src="pic/5.png" width = 40% >
    <font color=#a0a0a0 size=2><br>LiDAR point cloud edges</font>
</div>
<div align="center">
    <img src="pic/6.png" width = 40% >
    <font color=#a0a0a0 size=2><br>Image edges (blue) and LiDAR point cloud edges (red), green lines represent their corresponding relations</font>
</div>

4. Calibration result 
<div align="center">
    <img src="pic/7.png" width = 40% >
    <font color=#a0a0a0 size=2><br>Projection of point cloud onto the image using calibrated extrinsic matrix</font>
</div>
 
5. Multi-scene calibration
The targetless method needs scene with rich edge features. If not, the calibration result will be false because of false matched edges.
<div align="center">
    <img src="pic/8.png" width = 40% >
    <font color=#a0a0a0 size=2><br>False matched edge leading to bad calibration result</font>
</div>
When use multi-scene to do the calibration, the problem can be allievated.
<div align="center">
    <img src="pic/9.png" width = 40% >
    <font color=#a0a0a0 size=2><br>After multi-scene calibration result</font>
</div>
 

## 4. Target-based calibration of RsLiDAR and D455 camera
This work is based on [Target-based lidar_camera_calibrator](https://github.com/HITSZ-NRSL/lidar_camera_calibrator)

1. Prepare calibration board
<div align="center">
    <img src="pic/10.png" width = 40% >
    <font color=#a0a0a0 size=2><br>Calibration board target</font>
</div>

2. We uphold the target and record the ROS bag. Then extract pcd files and images from the bag. We use get_sync_data.launch in our repository to ensure the pcd and image obtained are sychronized. Our sample data set is in file /data/rslidar_camera

3. Select corner points of the target manually, by clicking on the image viewer
<div align="center">
    <img src="pic/11.png" width = 40% >
    <font color=#a0a0a0 size=2><br>Corner points of the target in the image</font>
</div>

4. Select the ROI from the LiDAR point cloud by adjusting the parameters in the UI
<div align="center">
    <img src="pic/18.png" width =25% >
    <font color=#a0a0a0 size=2><br>Corner points of the target in the point cloud</font>
</div>

5. Solve the PnP problem and get the result by the toolbox
<div align="center">
    <img src="pic/13.png" width = 40% >
    <font color=#a0a0a0 size=2><br>Projection of point cloud onto the image using calibrated extrinsic matrix</font>
</div>


## 5. Target-based calibration of Livox LiDAR and D455 camera
1. Our sample data set is in file /data/livox_camera

2. The steps are the same as above. We stable the system and record rosbag 1~2 seconds. Then obtain the accumulated pcd file from the bag. Finally, we use the toolbox and calibrate the dense LiDAR.
<div align="center">
    <img src="pic/14.png" width = 40% >
    <font color=#a0a0a0 size=2><br>Target pose</font>
</div>
<div align="center">
    <img src="pic/15.png" width = 40% >
    <font color=#a0a0a0 size=2><br>Colored point cloud</font>
</div>


## 6. Target-based calibration using MATLAB toolbox
This method is based on [MATLAB_Lidar_camera_calibrator](https://www.mathworks.com/help/lidar/ug/lidar-and-camera-calibration.html)
1. Prepare a chessboard and measure the size of it. The size of chessboard  square in this experiment is 10.7cmx10.7cm with 0.6cm boarder to pad.

2. To get the input pcd and image files. Same step as shown in 4. Our sample data set is in file /data/rslidar_camera_matlab

3. Input data in MATLAB. The image corner and LiDAR point cloud target plane can be detected.
<div align="center">
    <img src="pic/16.png" width = 60% >
    <font color=#a0a0a0 size=2><br>Detected features</font>
</div>

4. Calibrate and reject the image-point cloud pairs with large error
<div align="center">
    <img src="pic/17.png" width = 60% >
    <font color=#a0a0a0 size=2><br>Projection of point cloud onto the image using calibrated extrinsic matrix</font>
</div>

## 7. Publish transformation in ROS
The obtained transformations can be publish in ROS using [static_transform_publisher](http://wiki.ros.org/tf#static_transform_publisher). Finally, the sensor fusion is realized.
```
static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period_in_ms
```

## 8. Calibration results for reference

## 9. References
[1] Z. Zhang, "A flexible new technique for camera calibration," in IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 22, no. 11, pp. 1330-1334, Nov. 2000, doi: 10.1109/34.888718.
<br>
[2] J. -K. Huang and J. W. Grizzle, "Improvements to Target-Based 3D LiDAR to Camera Calibration," in IEEE Access, vol. 8, pp. 134101-134110, 2020, doi: 10.1109/ACCESS.2020.3010734.
<br>
[3] C. Yuan, X. Liu, X. Hong and F. Zhang, "Pixel-Level Extrinsic Self Calibration of High Resolution LiDAR and Camera in Targetless Environments," in IEEE Robotics and Automation Letters, vol. 6, no. 4, pp. 7517-7524, Oct. 2021, doi: 10.1109/LRA.2021.3098923.

## 10. Acknowledgement
1. Binocular camera calibration: Matlab [Stereo_camera_calibrator](https://www.mathworks.com/help/vision/ref/stereocameracalibrator-app.html)
2. Targetless calibration of Livox LiDAR and D455 camera: [lidar_camera_calib](https://github.com/hku-mars/livox_camera_calib)
3. Target-based method 1:  [lidar_camera_calibrator](https://github.com/HITSZ-NRSL/lidar_camera_calibrator)
4. Target-based method 2: MATLAB [Lidar_camera_calibrator](https://www.mathworks.com/help/lidar/ug/lidar-and-camera-calibration.html)




