# LiDAR-SFM (paper under review)
Point cloud registration is a prerequisite for many applications in computer vision and robotics. Most existing methods focus on pairwise registration of two point clouds with high overlap. Although there have been some learning-based methods for low-overlap cases, they do not generalize well to unseen scenes and cannot handle degraded scenarios. This paper introduces a novel framework named L-PR, designed to register unordered, low overlap multiview point clouds by leveraging LiDAR fiducial markers. We refer to them as LiDAR fiducial markers, but they are the same as the popular AprilTag and ArUco markers—thin sheets of paper that do not affect the 3D geometry of the environment. We first propose an improved adaptive threshold marker detection method to provide robust detection results when the viewpoints among point clouds change dramatically. Then, we formulate the unordered multiview point cloud registration problem as a maximum a-posteriori (MAP) problem and develop a framework consisting of two levels of graphs to address it. The first-level graph, constructed as a weighted graph, is designed to efficiently and optimally infer initial values of scan poses from the unordered set. The second-level graph is created as a factor graph. By globally optimizing the variables on the graph, including scan poses, marker poses, and marker corner positions, we tackle the MAP problem.
![github1](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/66a9c4a3-02bb-4d3e-9a77-b644411fa6d5)

# Point Cloud Registration
![newdata](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/fef4d71f-3ff3-4bb9-96ca-150a54d7b076)

# Instance Reconstruction Evaluation
We use the same [rosbag](https://drive.google.com/file/d/1WpoWz7d5rv1s7l6DpmfL7u7jyJ3XLOmj/view?usp=sharing) to evaluate the proposed method against  [Livox Mapping](https://github.com/Livox-SDK/livox_mapping) and [LOAM Livox](https://github.com/hku-mars/loam_livox). The rosbag was recorded in a garage. The LiDAR follows an elliptical trajectory to scan a vehicle (Mercedes-Benz GLB).  Please refer to the following videos for the details of the rosbag. <br>

*  This video (×24) shows the mapping procedure of [Traj_LO](https://github.com/kevin2431/Traj-LO).  <br>
![trajlo1](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/809eeaf4-1b2b-4466-916a-73f585e6c724)

*  This video (×24) shows the mapping procedure of [Livox Mapping](https://github.com/Livox-SDK/livox_mapping).  <br>
![sdk](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/1bd0ec6b-c086-4b53-a252-d9babfbaa6df)  <br>
*  This video (×24) shows the mapping procedure of [LOAM Livox](https://github.com/hku-mars/loam_livox). <br>
![loam](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/6bfeaa09-8a47-4818-904d-7ea4fe851de4) <br>
* This video shows the pipeline of our framework. <br>
![ourspipeline](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/79f8d989-bc41-45f9-a1f0-e46cdf860257)

* Ground Truth (Mercedes-Benz GLB). <br>
![ezgif-1-375ecca334](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/a1eba0cf-f41f-4d7a-89e3-4e31194c628a) <br>
In the following, the point clouds are normalized into a unit sphere (i.e. [-1,1]). The first metric is the Chamfer Distance (CD). Given two point sets, the CD is the sum of the squared distance of each point to the nearest point in the other point set: <br>
![image](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/8d8f31d4-5bf2-4f58-b0ca-3e1c3cc5380b)<br>
That is, a smaller CD value indicates a higher fidelity.<br>
The second metric is  the recall of the ground truth points from the reconstructed shape, which is defined as:
![image](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/db61932e-8bee-4bb6-9c09-89f159e6c149) <br>
A higher Recall indicates a higher fidelity. <br>
* Ours. <br>
CD: 0.0030. Recall: 96.22% <br>
![ezgif-1-3eaeb864b3](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/939e7ca6-b916-4831-a24d-869b6dc61686)
* [Livox Mapping](https://github.com/Livox-SDK/livox_mapping) <br>
CD: 0.0106. Recall: 78.8264% <br>
![ezgif-1-5774539aa0](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/cdaa5904-da4d-46ed-9f7a-9b063fd5c1df)
* [LOAM Livox](https://github.com/hku-mars/loam_livox) <br>
CD: 0.0335. Recall: 75.2704%<br>
![ezgif-5-417764747f](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/264ba542-7c4e-4f93-b0d3-6430ed96a920)<br>

# Evaluation in a Degraded Scene
* While the quality of instance reconstruction indirectly reflects the localization accuracy, we also directly compare the localization accuracy of different methods. The following figure shows the setup. The ground truth
trajectory is given by an OptiTrack Motion Capture (MoCap) system. <br>
<img width="600" height="300" src="https://github.com/yorklyb/LiDAR-SFM/assets/58899542/640dec4f-64a6-4136-966c-483df4a9412b"/> <br>

*  This video (×24) shows the mapping procedure of [Traj_LO](https://github.com/kevin2431/Traj-LO).  <br>
![trajlo2](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/183f7558-8c09-48a2-9e39-0e2b95619735)

*  This video (×24) shows the mapping procedure of [Livox Mapping](https://github.com/Livox-SDK/livox_mapping).  <br>
![sdk](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/cacded49-3a75-4f83-a1f8-64c22c6f39c9)<br>
*  This video (×24) shows the mapping procedure of [LOAM Livox](https://github.com/hku-mars/loam_livox). <br>
![loam](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/62611bab-44fc-4f15-b72b-edbb043aea41)<br>
*  This video shows the mapping result of our method. <br>
![ours](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/e89f96dd-fecf-4eae-9b5c-07a2fc340d41) <br>
This figure shows the comparison of the mapping results. 
![labpic](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/81d7f4fb-6b81-4dee-8490-1410906b4284)


## Requirements
* GTSAM <br>
First, please ensure you can run the python demos given by [GTSAM](https://github.com/borglab/gtsam/tree/develop).<br>
We found that GTSAM cannot be installed appropriately in a conda environment. Thus, a conda environment is not recommended. Otherwise, errors will be reported when you use BearingRangeFactor3D or BetweenFactorPose3.

* IILFM <br>
A light and insertable version of [IILFM](https://github.com/York-SDCNLab/IILFM) is included in the files. To build and run it, you need to install basic tools like cmake.<br>
If you'd like to give it a quick try, just follow the commands below. However, please note that the default settings correspond to the test reconstructing the scene of a lab from 11 frames with added AprilTags. (i.e. the default detector is an AprilTag detector and the marker size is 16.4cm × 16.4cm). If you want to try the ArUco demo (marker size: 69.2cm × 69.2cm), which involves reconstructing a vehicle, you will need to adjust the settings of the detector and marker size as guided by the readme in ```iilfm```. Also, do not forget to change in marker size in ```main.py```.
* Python Packages <br>
The following packages are required. <br>
[AprilTag](https://pypi.org/project/apriltag/) <br>
[opencv-python](https://pypi.org/project/opencv-python/)<br>
[networkx](https://pypi.org/project/networkx/) <br>
[open3d](https://pypi.org/project/open3d/) <br>
[scipy](https://scipy.org/install/) <br>
[matplotlib](https://pypi.org/project/matplotlib/)<br>
opencv-python(cv2) has a built-in ArUco detector. Please ensure you can run the python demos of [ArUco detection](https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/). Again, a conda environment is not recommended. <br>

## Resources
* Raw [rosbag](https://drive.google.com/file/d/1WpoWz7d5rv1s7l6DpmfL7u7jyJ3XLOmj/view?usp=sharing) of the instance reconstruction evaluation.
* Raw [rosbag](https://drive.google.com/file/d/1mD_iukNYWuMu_6VKfMzh-utSH37x2Nzp/view?usp=sharing) of the mapping and localization evaluation.
* The extracted point clouds of vehicles and the script to run the quantitative evaluation are available [here](https://drive.google.com/drive/folders/1YU-PE9-gMEJdje15EafGrvCsyj_rjSE7?usp=sharing). 
* The [LiDAR frames]( https://drive.google.com/drive/folders/1oNh-m1SjBqDn8nn_UA-1GsP4ciWXqXe2?usp=sharing) utilized by our method. You need to download the LiDAR frames, rename the interested folder name to 'pc', and put it in the same level directory as the 'main.py'.


## Commands
```git clone https://github.com/yorklyb/LiDAR-SFM.git```<br>
```cd LiDAR-SFM```<br>
```cd lidar-sfm```<br>
```cd iilfm```<br>
```mkdir build```<br>
```cd build```<br>
```cmake ..```<br>
```make```<br>
```mv tag_detection ../../```<br>
Ensure that the point clouds are named '1.pcd', '2.pcd', etc., and are placed into a folder named 'pc'. The 'pc' folder should be located in the same level directory as the 'main.py' file. <br>
```python3 main.py```<br>
After processing all the point clouds, you will see a graph plot. Close it by pressing 'q' or using the close button. Finally, an output file named 'out.pcd' will be generated.

## How to collect your own LiDAR frames
First, you need to record the rostopic of the point cloud as rosbags. If you are using Livox MID-40, run ```rosbag record /livox/lidar``` in the terminal while the [Livox-ros-driver](https://github.com/Livox-SDK/livox_ros_driver) is running. Then, assume that you placed the LiDAR at N viewpoints and obtained N rosbags. You need to put all of them in a folder named 'data'. Check [this](https://drive.google.com/drive/folders/1oIFrRUfthl8H2kJgLHibkD6SlCBlg2uB?usp=sharing) out as an example. <br>
Suppose that you have done ```git clone https://github.com/yorklyb/LiDAR-SFM.git```<br>
```cd LiDAR-SFM```<br>
```cd merge```<br>
```mkdir build```<br>
```cd build```<br>
```cmake ..```<br>
```make```<br>
Put ```process.py``` into build. Also, put 'data' into build.<br>
Open a terminal and run ```roscore```.
Open a new terminal and run ```python3 process.py```. You will find the rosbags are transformed into pcd files in the folder 'processed'. Rename the folder as 'pc'.<br>
If you are using other LiDAR models, you need to change the rostopic name when recording rosbags using ```rosbag record your_topic_name``` and also update the topic in ```process.py``` accordingly.
