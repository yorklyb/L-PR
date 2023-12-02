import os
import shutil
path_0 = "./data"
files = os.walk(path_0)
count = 0
path_1 = "./processed"
path_2 = "./pcds"
if os.path.exists(path_1):
    shutil.rmtree(path_1)
if os.path.exists(path_2):
    shutil.rmtree(path_2)
os.mkdir("./processed")
os.mkdir("./pcds")

for i,j,k in files:
    for file in k:
        bag_file = os.path.join(path_0,file)
        count = count +1
        pcd_file = os.path.join(path_2,str(count))
        os.mkdir(pcd_file)
        # print(pcd_file)
        cmd = "rosrun pcl_ros bag_to_pcd " +str(bag_file) + " "+"/livox/lidar" + " "+str(pcd_file)
        # print('cmd',cmd)
        os.system(cmd)
        path_3 = os.path.join(pcd_file,"out")
        os.mkdir(path_3)
        cmd2 = "./merge"+ " "+pcd_file+"/"
        # print(cmd2)
        os.system(cmd2)

        merged_pcd = os.path.join(pcd_file+"/out/out.pcd")
        processed_pcd = os.path.join(path_1,str(count)+".pcd")
        print(merged_pcd,processed_pcd )
        shutil.copyfile(merged_pcd,processed_pcd)



