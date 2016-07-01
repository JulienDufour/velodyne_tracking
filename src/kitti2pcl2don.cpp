//This program filters the map using algorithms in Probablist Robotics chapter 6
#include <ros/ros.h>
#include <ros/package.h>
  // PCL specific includes
#include <pcl_ros/point_cloud.h>


#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
// #include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
#include <utils/clustering.h>
#include <utils/dataio.h>

#include <iostream>
#include <fstream>


typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace pcl;
using namespace std;

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "velodine_tracking");
  ros::NodeHandle nh = ros::NodeHandle("~");

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  // PARAMETERS from kitti2don.launch
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  std::string date;
  int tmpid;
  float scale1, scale2, threshold;

  nh.param<std::string>("date", date, "0");
  nh.param<int>("id", tmpid, 0);
  nh.param<float>("scale1", scale1, 0);
  nh.param<float>("scale2", scale2, 0);
  nh.param<float>("threshold", threshold, 0);

  // scale1 needs to be smaller than scale 1;
  if(scale2 < scale1){
    float tmp = scale2;
    scale2 = scale1;
    scale1 = tmp;
  }

  // Convert integer id to a string of 4 characters 
  std::string id = std::to_string(tmpid);
  if(id.length() > 4){
    std::cout << "dataset ID to long, can only be 4 letters, (0000, 0012, 0153,... etc)." << std::endl;
    return 0;
  } else if(id.length() < 4){
    while(4-id.length() > 0){
      id = "0" + id;
    }
  }

  // Create PATHS and folder for files.
  std::string pkg_path = ros::package::getPath("velodyne_tracking") + "/datasets/" + date + "/" + date + "_drive_" + id + "_sync/";
  std::string load_path = pkg_path + "velodyne_points/data/";
  std::string save_path = load_path + "pcd/don/";
  utils::createDirectories(save_path);

  std::vector<std::string> filenames;
  std::string type = "bin";
  int i = 0;
  std::cout << load_path << std::endl;
  if(utils::getFileNames(load_path, type, filenames)){
    std::cout << "" << std::endl;
    std::cout << "Processing dataset " << date << "-" << id << "." << std::endl;
    ros::Time begin = ros::Time::now();
    for(auto name : filenames){

  		pcl::PointCloud<PointXYZI>::Ptr cloud (new pcl::PointCloud<PointXYZI>);
  		if(utils::kitti2pcl(load_path, name, type, cloud)){
  			pcl::PointCloud<PointNormal>::Ptr doncloud(new pcl::PointCloud<PointNormal>);
  			utils::donFilter(cloud, scale1, scale2, threshold, doncloud);
  			pcl::PCDWriter writer;
  	    // Save as PCL pointcloud.
  	    writer.write<PointNormal>(save_path + name + ".pcd", *doncloud, false);
  		}
      int sec = (ros::Time::now() - begin).toSec() / (i/(float)filenames.size());
      int min = sec / 60;
      int hours = min / 60;

      std::cout << "\rProcessed: ";
      std::cout << setfill(' ') << setw(4) << std::right << std::round(i/(float)(filenames.size())*100.0);
      std::cout << "\%," << setfill(' ') << setw(4) << i << "/" << filenames.size();
      std::cout << ". Estimated time remaining: ";

      if(hours > 999 || hours < 0){
        std::cout << setfill(' ') << setw(10) << std::left << "NaN.";
      } else {
        std::cout << setfill(' ') << setw(3) << int(hours) << ":";
        std::cout << setfill('0') << setw(2) << int(min%60) << ":";
        std::cout << std::left << setfill('0') << setw(2) << int(sec%60);
        std::cout << ".";
      }
      std::cout.flush();
      i = i+1;

      if(!ros::ok()) break;

  	}
  }

	return(0);
}
