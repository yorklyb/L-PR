# LiDAR-SFM
* Ground Truth (Mercedes-Benz GLB) <br>
![ezgif-1-375ecca334](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/a1eba0cf-f41f-4d7a-89e3-4e31194c628a) <br>
The utilized metric is the Chamfer Distance (CD). Given two point sets, the CD is the sum of the squared distance of each point to the nearest point in the other point set: <br>
![image](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/8d8f31d4-5bf2-4f58-b0ca-3e1c3cc5380b)<br>
That is, a smaller CD value indicates a higher fidelity.
* Ours. <br>
CD: 0.003029<br>
![ezgif-1-3eaeb864b3](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/939e7ca6-b916-4831-a24d-869b6dc61686)
* [Livox Mapping](https://github.com/Livox-SDK/livox_mapping) <br>
CD: 0.011128<br>
![ezgif-1-5774539aa0](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/cdaa5904-da4d-46ed-9f7a-9b063fd5c1df)
* [LOAM Livox](https://github.com/hku-mars/loam_livox) <br>
CD: 0.033571<br>
![ezgif-1-5b279fb800](https://github.com/yorklyb/LiDAR-SFM/assets/58899542/f3f95f46-6b44-456c-8d3a-a4afd557b088)
