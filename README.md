# LiDAR-SFM

# Instance Reconstruction Evaluation
We use the same [rosbag](https://drive.google.com/file/d/1WpoWz7d5rv1s7l6DpmfL7u7jyJ3XLOmj/view?usp=sharing) to evaluate the proposed method against  [LOAM Livox](https://github.com/hku-mars/loam_livox) and [Livox Mapping](https://github.com/Livox-SDK/livox_mapping). The rosbag was recorded in a garage. The LiDAR follows an elliptical trajectory to scan a vehicle (Mercedes-Benz GLB).  Please refer to the following videos for the details of the rosbag.


https://github.com/yorklyb/LiDAR-SFM/assets/58899542/e5ef8125-e5ff-4b1e-95bd-034628be595c



https://github.com/yorklyb/LiDAR-SFM/assets/58899542/6da9dfb8-39fd-4448-8095-e6cc0a2fa00e



* Ground Truth (Mercedes-Benz GLB). <br>
![ezgif-1-375ecca334](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/a1eba0cf-f41f-4d7a-89e3-4e31194c628a) <br>
The utilized metric is the Chamfer Distance (CD). Given two point sets, the CD is the sum of the squared distance of each point to the nearest point in the other point set: <br>
![image](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/8d8f31d4-5bf2-4f58-b0ca-3e1c3cc5380b)<br>
That is, a smaller CD value indicates a higher fidelity. In the following, the point clouds are normalized into a unit sphere (i.e. [-1,1]). The script to run the quantitative comparison and the PCD files for the point clouds are available [here](https://drive.google.com/file/d/108GugB5e8sS_BZuOAMgFwMsDRQvkD0qE/view?usp=sharing).
* Ours. <br>
CD: 0.003029<br>
![ezgif-1-3eaeb864b3](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/939e7ca6-b916-4831-a24d-869b6dc61686)
* [Livox Mapping](https://github.com/Livox-SDK/livox_mapping) <br>
CD: 0.011128<br>
![ezgif-1-5774539aa0](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/cdaa5904-da4d-46ed-9f7a-9b063fd5c1df)
* [LOAM Livox](https://github.com/hku-mars/loam_livox) <br>
CD: 0.033571<br>
![ezgif-5-417764747f](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/264ba542-7c4e-4f93-b0d3-6430ed96a920)<br>
[rosbag](https://drive.google.com/file/d/1mD_iukNYWuMu_6VKfMzh-utSH37x2Nzp/view?usp=sharing)
