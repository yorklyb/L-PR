import os
import shutil
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X
import networkx as nx
import matplotlib.pyplot as plt
import open3d as o3d

#initialization
#####################################################################
path_1 = "./poses"
path_2 = "./points"
path_3 = "./pics"

marker_size = 0.164 #0.164,0.692
if os.path.exists(path_1):
    shutil.rmtree(path_1)
if os.path.exists(path_2):
    shutil.rmtree(path_2)
if os.path.exists(path_3):
    shutil.rmtree(path_3)
os.mkdir(path_1)
os.mkdir(path_2)
os.mkdir(path_3)


root_path = "./pc"
pc_path = os.walk(root_path)
relative_poses = [] #every pose will be saved in this list
ids = []
frames = []
G = nx.Graph()
poses = []
num_frames = 0

color = np.linspace(0.3,1,30)
colormap = []
for i in range(color.shape[0]):
    colormap.append([0,0,color[i]])
colormap = np.array(colormap)

# colormap = []
# #red to yellow
# color_1 = np.linspace(0,255,20)
#
# for i in range(color_1.shape[0]):
#     # colormap.append([0,0,color_1[i]])
#     colormap.append([255, color_1[i], 0])
# #orange to yellow
# color_2 = np.linspace(255,0,20)
# for i in range(color_2.shape[0]):
#     # colormap.append([0,0,color_1[i]])
#     colormap.append([color_2[i], 255, 0])
#
# for i in range(color_2.shape[0]):
#     # colormap.append([0,0,color_1[i]])
#     colormap.append([0,color_2[i], color_1[i]])
# colormap = np.array(colormap)


#colormap = np.random.randint(0,255,(200,3))/255
#colormao = np.array([[128,130,120],[235,0,205],[0,215,0],[235,155,0]])/255


#####################################################################

def load_pcd_to_ndarray(pcd_path):
    #note that open3d does not support XYZI type pc reading. Thus, we need to use this function.
    with open(pcd_path) as f:
        while True:
            ln = f.readline().strip()
            if ln.startswith('DATA'):
                break

        points = np.loadtxt(f)
        points = points[:, 0:4]
        return points


#1st graph
#####################################################################

for i,j,k in pc_path: #loop for processing point clouds
    num_frames = len(k)

    #k.sort(key=lambda x:int(x[:]))
    for frame_name in sorted(k):
        
        print("processing", frame_name)
        frame_path = os.path.join(root_path,frame_name)
        temp_path = "./this.pcd"
        shutil.copy(frame_path,temp_path)
        os.system("./tag_detection")
        temp_pose_path = str("./poses/") + str(frame_name.split(".")[0]) +str(".txt")
        shutil.copy("./pose.txt",temp_pose_path)
        temp_pose_path_2 = str("./points/") + str(frame_name.split(".")[0]) +str(".txt")
        shutil.copy("./points.txt",temp_pose_path_2)
        temp_pose_path_3 = str("./pics/") + str(frame_name.split(".")[0]) + str(".png")
        shutil.copy("./this.png", temp_pose_path_3)
        frame_id = int(frame_name.split(".")[0])
        frames.append(frame_id)

        f = open(temp_pose_path)
        line = f.readline()

        while line:
            id = line.split(',')[0]
            e = line.split(',')[13]
            e = e.replace("\n","")
            e = float(e)
            id =int(id)+num_frames+100
            G.add_weighted_edges_from([(frame_id,id,e)])
            pose = {}
            # R,t: marker frame --> local LiDAR frame
            pose['pose'] = np.matrix([ 
                [float(line.split(',')[1]),float(line.split(',')[2]),float(line.split(',')[3]),float(line.split(',')[10])],
                [float(line.split(',')[4]),float(line.split(',')[5]),float(line.split(',')[6]),float(line.split(',')[11])],
                [float(line.split(',')[7]),float(line.split(',')[8]),float(line.split(',')[9]),float(line.split(',')[12])],
                [0.0,0.0,0.0,1.0]])
            pose['from'] = id
            pose['to'] = frame_id
            poses.append(pose)
            # print('test',poses)

            ids.append(id)
            line = f.readline()

ids = sorted(set(ids),key=ids.index)
print(ids)
print(frames)

pos = nx.random_layout(G)
nx.draw(G, pos, with_labels=True, alpha=0.8)
labels = nx.get_edge_attributes(G,'weight')
nx.draw_networkx_edge_labels(G,pos,edge_labels=labels, font_color='c')
nx.draw_networkx_nodes(G,pos,nodelist=frames,node_color='yellow')
plt.show() # this is the frist graph of the pupeline. It shows the relations between frames and markers.


re_poses = []
for i in frames:
    minwpath = nx.dijkstra_path(G,source = 1, target = i)
    print('minwpath from 1 to',i,minwpath)

    for k in minwpath:
        # the first local lidar frame is set as the global frame
        if k ==1:
            re_pose = np.matrix([ 
                [1.0,0.0,0.0,0.0],
                [0.0,1.0,0.0,0.0],
                [0.0,0.0,1.0,0.0],
                [0.0,0.0,0.0,1.0]])
            s = 1
        if k !=1:       
            to = k
            if k in ids:
                to = s
                s = k
                switched = 1

            for j in poses:
                if j['from'] == s and j["to"] == to:
                    if switched ==0:
                        re_pose = j["pose"]*re_pose
                        s = to
                    if switched ==1:
                        re_pose =  j["pose"].I*re_pose
                        switched = 0
                        # s = to
    f_re_pose = {}            
    f_re_pose['to'] = i
    f_re_pose['re_pose'] = re_pose
    re_poses.append(f_re_pose)


for i in ids:
    minwpath = nx.dijkstra_path(G,source = 1, target = i)
    print('minwpath from 1 to',i,minwpath)

    for k in minwpath:
        # the first local lidar frame is set as the global frame
        if k ==1:
            re_pose = np.matrix([
                [1.0,0.0,0.0,0.0],
                [0.0,1.0,0.0,0.0],
                [0.0,0.0,1.0,0.0],
                [0.0,0.0,0.0,1.0]])
            s = 1
        if k !=1:
            to = k
            if k in ids:
                to = s
                s = k
                switched = 1

            for j in poses:
                if j['from'] == s and j["to"] == to:
                    if switched ==0:
                        re_pose = j["pose"]*re_pose
                        s = to
                    if switched ==1:
                        re_pose =  j["pose"].I*re_pose
                        switched = 0
                        # s = to
    f_re_pose = {}
    f_re_pose['to'] = i
    f_re_pose['re_pose'] = re_pose
    re_poses.append(f_re_pose)
#
# print(poses)
# print(re_poses)
with open('re_pose.txt', 'w') as f:
    f.write(str(re_poses))

#1st graph ends
#####################################################################



#2nd graph
#####################################################################
for i,j,k in pc_path: #loop for processing point clouds
    
    for frame_name in sorted(k):
        frame_path = os.path.join(root_path,frame_name)
        points = load_pcd_to_ndarray(frame_path)
        points = points.copy()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        points_intensity = points[:, 3] # intensity
        points_colors = [colormap[int(points_intensity[i]) % colormap.shape[0]] for i in range(points_intensity.shape[0])]
        pcd.colors = o3d.utility.Vector3dVector(points_colors) # 根据 intensity 为点云着色
        frame_id = int(frame_name.split(".")[0])

prior_xyz_sigma = 0.01
    # Declare the 3D rotational standard deviations of the prior factor's Gaussian model, in degrees.
prior_rpy_sigma = 0.01
    # Declare the 3D translational standard deviations of the odometry factor's Gaussian model, in meters.
frame_xyz_sigma = 0.5
     # Declare the 3D rotational standard deviations of the odometry factor's Gaussian model, in degrees.
frame_rpy_sigma = 0.05

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([prior_rpy_sigma*np.pi/180,
                                                                prior_rpy_sigma*np.pi/180,
                                                                prior_rpy_sigma*np.pi/180,
                                                                prior_xyz_sigma,
                                                                prior_xyz_sigma,
                                                                prior_xyz_sigma]))

FRAME_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([frame_rpy_sigma*np.pi/180,
                                                                frame_rpy_sigma*np.pi/180,
                                                                frame_rpy_sigma*np.pi/180,
                                                                frame_xyz_sigma,
                                                                frame_xyz_sigma,
                                                                frame_xyz_sigma]))



graph = gtsam.NonlinearFactorGraph()
P0 = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
graph = gtsam.NonlinearFactorGraph()
#prior factor
X_prior = X(int(1))
graph.add(gtsam.PriorFactorPose3(X_prior, gtsam.Pose3(P0), PRIOR_NOISE))

initial_estimate = gtsam.Values()
initial_estimate.insert(X_prior, gtsam.Pose3(P0))


for ii in poses:
    from_id = ii['from']
    to_id=ii['to']
    # print('test',to_id)
    X_from = X(int(from_id))
    X_to = X(int(to_id))
    POSE = ii['pose'].I
    if from_id != to_id:
        graph.add(gtsam.BetweenFactorPose3(X_from, X_to, gtsam.Pose3(POSE),FRAME_NOISE)) #   X_to = X_from*POSE
        # initial_estimate.insert(X_to, gtsam.Pose3(POSE))



for ii in re_poses:
    from_id = 1
    to_id=ii['to']
    # print('test',to_id)
    X_from = X(int(from_id))
    X_to = X(int(to_id))
    POSE = ii['re_pose'].I
    if from_id != to_id:
        graph.add(gtsam.BetweenFactorPose3(X_from, X_to, gtsam.Pose3(POSE),FRAME_NOISE)) #   X_to = X_from*POSE
        initial_estimate.insert(X_to, gtsam.Pose3(POSE)) 


root_points_path = './points'
points_path = os.walk(root_points_path)

processed_points = []


for i,j,k in points_path: #loop for processing point clouds

    for frame_name in sorted(k):
        frame_path = os.path.join(root_points_path,frame_name)
        ff = open(frame_path)
        line = ff.readline()
        frame_id = int(frame_name.split(".")[0])
        while line:
            id = int(line.split(',')[0])+num_frames+100
            index = int(line.split(',')[1])
            # print('here',index)
            real_index = 4*id+index
            x = float(line.split(',')[2])
            y = float(line.split(',')[3])
            z = float(line.split(',')[4])
            
            # theta = np.arctan(y/x)*180/np.pi
            # phi = np.arctan(z/(np.sqrt(x*x+y*y)))*180/np.pi
            r=np.sqrt(x*x+y*y+z*z)
            L_to = L(int(real_index))
            X_from = X(int(frame_id))

            bearing_measurement = gtsam.Unit3(np.array([x,y,z]))
            graph.add(gtsam.BearingRangeFactor3D(X_from, L_to, bearing_measurement, r, gtsam.noiseModel.Isotropic.Sigma(3, 0.03)))
            marker_noise = 0.01
            if index ==0:

                X_marker = X(int(id))
                mx = float(-0.5*marker_size)
                my = float(-0.5*marker_size)
                mz = float(0)
                mr = np.sqrt(mx * mx + my * my + mz * mz)
                m_bearing_measurement = gtsam.Unit3(np.array([mx, my, mz]))
                graph.add(gtsam.BearingRangeFactor3D(X_marker, L_to, m_bearing_measurement, mr, gtsam.noiseModel.Isotropic.Sigma(3, marker_noise)))


            if index ==1:

                X_marker = X(int(id))
                mx = float(0.5*marker_size)
                my = float(-0.5*marker_size)
                mz = float(0)
                mr = np.sqrt(mx * mx + my * my + mz * mz)
                m_bearing_measurement = gtsam.Unit3(np.array([mx, my, mz]))
                graph.add(gtsam.BearingRangeFactor3D(X_marker, L_to, m_bearing_measurement, mr, gtsam.noiseModel.Isotropic.Sigma(3, marker_noise)))
            if index ==2:

                X_marker = X(int(id))
                mx = float(0.5*marker_size)
                my = float(0.5*marker_size)
                mz = float(0)
                mr = np.sqrt(mx * mx + my * my + mz * mz)
                m_bearing_measurement = gtsam.Unit3(np.array([mx, my, mz]))
                graph.add(gtsam.BearingRangeFactor3D(X_marker, L_to, m_bearing_measurement, mr, gtsam.noiseModel.Isotropic.Sigma(3, marker_noise)))
            if index ==3:

                X_marker = X(int(id))
                mx = float(-0.5*marker_size)
                my = float(0.5*marker_size)
                mz = float(0)
                mr = np.sqrt(mx * mx + my * my + mz * mz)
                m_bearing_measurement = gtsam.Unit3(np.array([mx, my, mz]))
                graph.add(gtsam.BearingRangeFactor3D(X_marker, L_to, m_bearing_measurement, mr, gtsam.noiseModel.Isotropic.Sigma(3, marker_noise)))


            processed =0
            for ii in re_poses:
                if frame_id == ii['to']:
                    POSE = ii['re_pose'].I
                    point = POSE.dot(np.array([x,y,z,1]))
                    point = point[0,0:3].reshape((3,1))

                    if int(real_index) in processed_points:
                        processed = 1
                    if processed ==0:
                        initial_estimate.insert(L_to, point)
                        processed_points.append(int(real_index))


            line = ff.readline()


# print('gtsam graph',graph)
params = gtsam.LevenbergMarquardtParams()
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate,
                                                  params)
result = optimizer.optimize()
#2nd graph ends. result is the optimized graph.
#####################################################################




#visualization
#####################################################################
pc_path = os.walk(root_path)
pcd_all = o3d.geometry.PointCloud()


for i,j,k in pc_path: #loop for processing point clouds
    
    for frame_name in sorted(k):
        frame_path = os.path.join(root_path,frame_name)

        points = load_pcd_to_ndarray(frame_path)
        points = points.copy()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        points_intensity = points[:, 3] # intensity
        points_colors = [colormap[int(points_intensity[i]) % colormap.shape[0]] for i in range(points_intensity.shape[0])]
        pcd.colors = o3d.utility.Vector3dVector(points_colors)

        frame_id = int(frame_name.split(".")[0])
        pose = result.atPose3(X(frame_id))

        R = pose.rotation().matrix()
        t = pose.translation()
        pcd.rotate(R,center=(0,0,0))
        pcd.translate((float(t[0]),float(t[1]),float(t[2])),relative=True)
        pcd_all = pcd_all+pcd

o3d.io.write_point_cloud("out.pcd",pcd_all) #save the result
os.remove("./points.txt")
os.remove("./pose.txt")
os.remove("./this.txt")
os.remove("./this.png")
os.remove("./this.pcd")
coor = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.6, origin=[0, 0, 0])
o3d.visualization.draw_geometries([pcd_all,coor],)




