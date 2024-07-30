# L-PR: Exploiting LiDAR Fiducial Marker for Unordered Low Overlap Multiview Point Cloud Registration
:mega: :mega: :mega: The [paper](https://arxiv.org/abs/2406.03298) is now available. <br>


Point cloud registration is a prerequisite for many applications in computer vision and robotics. 
Most existing methods focus on pairwise registration of two point clouds with high overlap. Although there have been some methods for low overlap cases, they struggle in degraded scenarios.
This paper introduces a novel framework dubbed L-PR, designed to register unordered low overlap multiview point clouds leveraging LiDAR fiducial markers.
We refer to them as LiDAR fiducial markers, but they are the same as the popular AprilTag and ArUco markers—thin sheets of paper that do not affect the 3D geometry of the environment.
We first propose an improved adaptive threshold marker detection method to provide robust detection results when the viewpoints among point clouds change dramatically.
Then, we formulate the unordered multiview point cloud registration problem as a maximum a-posteriori (MAP) problem and develop a framework consisting of two levels of graphs to address it.
The first-level graph, constructed as a weighted graph, is designed to efficiently and optimally infer initial values of scan poses from the unordered set.
The second-level graph is constructed as a factor graph. By globally optimizing the variables on the graph, including scan poses, marker poses, and marker corner positions, we tackle the MAP problem.
We conduct both qualitative and quantitative experiments to demonstrate that the proposed method surpasses previous state-of-the-art (SOTA) methods and to showcase that L-PR can serve as a low-cost and efficient tool for 3D asset collection and training data collection. 
In particular, we collect a new dataset named Livox-3DMatch using L-PR and incorporate it into the training of the SOTA learning-based method, [SGHR](https://github.com/WHU-USI3DV/SGHR), which brings evident improvements for SGHR on various benchmarks.

# Improved LiDAR Fiducial Marker Detection
We develop an improved adaptive threshold marker detection method to provide <br> robust detection 
results when the viewpoints among point clouds change dramatically. <br>
![image](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/9f879077-00cc-424b-838d-276e419390ea) ![image](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/2d9af120-c31b-4707-bcd8-bf11e43a7247)

# Point Cloud Registration
Given that the existing point cloud registration benchmark lacks fiducial markers in the scenes, we construct a new test dataset, as shown in the following figure, with the Livox MID-40. The competitors are [SGHR](https://github.com/WHU-USI3DV/SGHR) and [Teaser++](https://github.com/MIT-SPARK/TEASER-plusplus).
![newdata](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/fef4d71f-3ff3-4bb9-96ca-150a54d7b076)
![image](https://github.com/user-attachments/assets/41606165-4493-427a-9aa6-1d924c24c5d5)

# Application 1: 3D Asset Collection
* Ground Truth (Mercedes-Benz GLB). <br>
![ezgif-1-375ecca334](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/a1eba0cf-f41f-4d7a-89e3-4e31194c628a) <br>
In the following, the point cloud is normalized into a unit sphere (i.e. [-1,1]). The first metric is the Chamfer Distance (CD). Given two point sets, the CD is the sum of the squared distance of each point to the nearest point in the other point set: <br>
![image](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/8d8f31d4-5bf2-4f58-b0ca-3e1c3cc5380b)<br>
That is, a smaller CD value indicates a higher fidelity.<br>
The second metric is  the recall of the ground truth points from the reconstructed shape, which is defined as:
![image](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/db61932e-8bee-4bb6-9c09-89f159e6c149) <br>
A higher Recall indicates a higher fidelity. <br>
* Ours. <br>
CD: 0.0030. Recall: 96.22% <br>
![ezgif-1-3eaeb864b3](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/939e7ca6-b916-4831-a24d-869b6dc61686)
# Application 2: Training Data Collection
We collect a new training dataset called [Livox3DMatch]( https://drive.google.com/file/d/1zt9liSOxcERJ6jKVxvr4WWVXeDJ8BfqH/view?usp=sharing) using the proposed L-PR. Livox-3DMatch augments the original 3DMatch training data from 14,400 pairs to 17,700 pairs (a 22.91% increase). By training on this augmented dataset, the performance of [SGHR](https://github.com/WHU-USI3DV/SGHR) is improved by 2.90% on 3DMatch, 4.29% on ETH, and 22.72% (translation) / 11.19% (rotation) on ScanNet.
![livox3dmatch](https://github.com/user-attachments/assets/716bd4e1-9a5d-4f4c-a1c9-71127ee0037e)
![image](https://github.com/user-attachments/assets/f286aa80-9715-40df-a7a2-ef547f482fe9)
To reproduce the results in the above table, we recommend reproducing [SGHR](https://github.com/WHU-USI3DV/SGHR) first, including its training and testing. Then, download our Livox3DMatch dataset (it is already mixed with the original 3DMatch_train) and extract the file into ./data. Rename the file to 3DMatch_train if you do not want to make any modifications to SGHR. Finally, train and test the model. Before running Train.py, remember to replace the [dataset.py](https://github.com/WHU-USI3DV/SGHR/blob/master/dataops/dataset.py) with our dataset.py. 
Note that the SGHR [weight](https://github.com/WHU-USI3DV/SGHR/tree/master/checkpoints/yoho) (trained on 3DMatch_train only) is already downloaded when you git clone SGHR. The model trained on Livox3DMatch, named yoho_my, is available in this repository. You could simply replace the original weights with ours and do a quick test.

# Application 3: Reconstructing a Degraded Scene
The competitor is [An Efficient Visual SfM Framework Using Planar Markers](https://ieeexplore.ieee.org/document/10041830) (SfM-M). This scenario has repetitive structures and weak geometric features. We attach thirteen 16.4 cm x 16.4 cm AprilTags to the wall. The LiDAR scans the scene from 11 viewpoints. We also captured 72 images with an iPhone 13 to use as input for SfM-M. The ground truth trajectories are given by an OptiTrack Motion Capture system. The proposed approach achieves better localization accuracy, which is expected given that LiDAR is a ranging sensor. 
![labpic](https://github.com/user-attachments/assets/b924df15-6cd8-4341-96c4-be81ae5887ae)

# Application 4: 3D Map Merging
You need to apply the [this algortihm](https://github.com/York-SDCNLab/Marker-Detection-General) to localize fiducials on a 3D map.
<img width="600" height="380" src="https://github.com/user-attachments/assets/b54302a9-298a-411c-b076-aca6ea217eaa"/> <br>
<img width="600" height="120" src="https://github.com/user-attachments/assets/ae38fa64-b457-4709-8b15-03b985acaf05"/> <br>

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

## Data
* The extracted point clouds of vehicles and the script to run the quantitative evaluation are available [here](https://drive.google.com/drive/folders/1YU-PE9-gMEJdje15EafGrvCsyj_rjSE7?usp=sharing). 
* All the LiDAR scans used in this work are available [here]( https://drive.google.com/drive/folders/1oNh-m1SjBqDn8nn_UA-1GsP4ciWXqXe2?usp=sharing).
* You need to download the LiDAR scans, rename the interested folder name to 'pc', and put it in the same level directory as the 'main.py'. Please read the [instructions of IILFM](https://github.com/yorklyb/LiDAR-SFM/blob/master/lidar-sfm/iilfm/readme.txt) carefully to ensure that you are using the correct scripts, as some scans contain AprilTag while others contain ArUco.
* Raw [rosbag](https://drive.google.com/file/d/1WpoWz7d5rv1s7l6DpmfL7u7jyJ3XLOmj/view?usp=sharing) of the instance reconstruction evaluation.
* Raw [rosbag](https://drive.google.com/file/d/1mD_iukNYWuMu_6VKfMzh-utSH37x2Nzp/view?usp=sharing) collected in the degraded scene.

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
## ACKNOWLEDGMENT
We would like to express our gratitude to Haiping Wang for helping us reproduce SGHR, Xin Zheng for helping us reproduce Traj LO, and Jie Li and Hao Fan for helping us reproduce [An Efficient Visual SfM Framework Using Planar Markers](https://ieeexplore.ieee.org/document/10041830) (SfM-M). We also thank Honpei Yin and Jiahe Cui for helping us reproduce LOAM Livox, and Hao Wang, Yida Zang, Hunter Schofield, and Hassan Alkomy for their assistance in experiments. Additionally, we are grateful to Han Wang, Binbin Xu, Yuan Ren, Jianping Li, Yicong Fu, and Brian Lynch for constructive discussions.

