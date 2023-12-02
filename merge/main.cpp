//standard imports
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <dirent.h>

//point cloud imports
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char* argv[]){
	//dir vars
	//std::cout<<argv[1]<<std::endl;
	const char* fdir = argv[1];
	//const char* fdir = "./ggg4/";
	struct dirent *entry = nullptr;
	DIR *dp = nullptr; //directory pointer
	std::string name;
	
	//pointcloud vars
	std::vector<std::vector<float>> unique_points;
	std::vector<float> unique_point;
	float intensity_max = 0;
	//load the pcd into a pointcloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	
	dp = opendir(fdir);
	if(dp != nullptr){
		//iterate through directory
		while((entry = readdir(dp))){
			name = entry->d_name;
			
			//only get pcd files
			if(name.length() > 5 && name.substr(name.length()-4, name.length()-1) == ".pcd"){
				//set pointer to pointcloud
				std::cout << std::string(fdir).append(name) << std::endl;
				
				if(pcl::io::loadPCDFile<pcl::PointXYZI>(std::string(fdir).append(name), *cloud) == -1){
					PCL_ERROR("Could not read file \n");
					return(-1);
				}
				
				//remove points that are too close to sensor in x
				for(const auto &point1: *cloud){
					//the point is not a duplicate, so add it to unique points
					if(point1.x != 0 && point1.intensity != 0){
						unique_point = {point1.x, point1.y, point1.z, point1.intensity};
						unique_points.push_back(unique_point);
						if (point1.intensity> intensity_max) intensity_max = point1.intensity;

						// std::cout<<"intensity"<<point1.intensity<<std::endl;
					}
				}
			}
		}
	}
	
	//create empty pointcloud
	pcl::PointCloud<pcl::PointXYZI> unique_cloud;
	unique_cloud.width = unique_points.size();
	unique_cloud.height = 1;
	unique_cloud.is_dense = false;
	unique_cloud.points.resize(unique_cloud.width * unique_cloud.height);
	
	
	
	//write the unique points to the new pointcloud
	for(size_t p = 0; p < unique_points.size(); ++p){
		unique_cloud.points[p].x = unique_points[p][0];
		unique_cloud.points[p].y = unique_points[p][1];
		unique_cloud.points[p].z = unique_points[p][2];
		unique_cloud.points[p].intensity = unique_points[p][3]/intensity_max*255;
		
	}
	
	
	
	
	pcl::io::savePCDFileASCII(std::string(fdir).append("/out/out.pcd"), unique_cloud);
		// std::cout<<"intensity"<<intensity_max<<std::endl;
	closedir(dp);
	return 0;
}
