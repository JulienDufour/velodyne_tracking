#ifndef KITTI2PCL_H
#define KITTI2PCL_H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <fstream>

namespace utils{

void kitti2pcl(const std::string name, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
	
	// need empty pointcloud.
	if(cloud->points.size() != 0){
		cloud->clear();
	}

	std::fstream input(name, std::ios::in | std::ios::binary);
	if(!input.good()){
		std::cerr << "Could not read file: " << name << std::endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, std::ios::beg);

	int i;
	for (i=0; input.good() && !input.eof(); i++) {
		pcl::PointXYZI point;
		input.read((char *) &point.x, 3*sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));
		cloud->push_back(point);
	}
	input.close();
};

}
#endif