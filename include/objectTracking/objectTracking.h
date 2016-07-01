// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// objectTracking.h
// Purpose: Class that handles multiple moving object. Performes data association
// between objects and updates the correct one.

// @author Unnar Þór Axelsson
// @version 1.0 19/01/16
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


#ifndef OBJECTTRACKING_H
#define OBJECTTRACKING_H

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <objectTracking/object.h>
#include <objectEKF/objectEKF.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <math.h>
#include <Eigen/Dense>

// OSM (OpenStreetMap data)
#include <osm/osm.h>

// Label
#include <label/label.h>

using namespace std;
using namespace pcl;

class objectTracking{

	vector<object> objects_;
	ObjectEKF filter_;

	ros::NodeHandle *nh_;
	ros::Publisher pub_;
	bool publish_;
	float h_;
    float max_distance_, path_length_, time_since_measurement_;
    int k_kdTree;
	const float ratio_;

	label lab_;

public:
	objectTracking() : publish_(false), h_(rand()), ratio_(0.618033988749895), lab_(label(ros::package::getPath("velodyne_tracking") + "/datasets/label/")){};
	~objectTracking(){};

	// Used to initialize ros nodehandle to publish on topics, used for vizualisation.
	// This can be skipped if vizualisation is not wanted.
	// Input:
	// 	nh: The ros nodehandle.
	// 	id: Integar id, used when publishing to RVIZ.
	void initializeNH(ros::NodeHandle *nh);

	// Updates the correct set of objects.
	// Inputs:
	// 	time: The time the measurement was performed as a double.
	// 	clouds: vector of PointClouds where each cloud represents a potential object.
	// Output:
	// colored_cloud: Colored version of all the clouds where each color represents a single object. The color
	// 	is consistent over time for each object.
	void update(unsigned long time, Eigen::Matrix4d oxts_first_pose, utils::transformMatrix tf_mat, osm osmClass, vector<PointCloud<PointXYZ>::Ptr> &clouds, PointCloud<PointXYZRGB>::Ptr colored_cloud);


	// Creates a new object
	// Inputs:
	// 	time: The time the measurement was performed as a double.
	// 	position: x,y,z position of the centroid of the object.
	// 	sizex, sizey, sizez: the size of each side of the minimum aligned bounding box
	// 											 of the object.
	// 	filter: ekf filter to estimate position.
  //  objectvec: vector to store the created object.
	void createObject(unsigned long time,
                    utils::OBBcube obb,
                    ObjectEKF &filter,
                    std::vector<object> &objectvec);



	// Returns the total number of detected object.
	int getSize(void){ return objects_.size(); }

	// Return the path the object has moved after.
  	std::vector < object > getObjects(void) {
    	return objects_;
  	}

	// Publishes the paths of all objects that have moved a significant amount.
	void publishAllPaths(void);

	void writePathsToFile(std::vector<double> oxts_origin, std::string save_path, std::string file_name);

private:

	/**
     *  @brief obbIsOnRoad      Determine if an object defined by OBBcube is on road or not.
     *  @param obb              Represents an object in terms of position, orientation and size.                              (input)
     *  @param presentInWays    Contains collection of pair <way id, lane number> where object is present.                    (output)
     *  @param oxts_first_pose  Homogeneous matrix which represents the first state of the car.                               (input)
     *  @param osmClass         Osm class object which permits to manipulate osm data. Ensure that the object is up to date.  (input)
     *  @return                 True if the object is on the osm map. If not return False.
     */
	bool obbIsOnRoad(utils::OBBcube obb, std::vector<std::pair<long int,int>> &presentInWays, Eigen::Matrix4d oxts_first_pose, osm osmClass);

	/**
     *  @brief obbVsRoadType    Type of an object, obtains with SVM, is accorded to type of road ?
     *  @param obb              Represents an object in terms of position, orientation and size.                              (input)
     *  @param tf_mat           Collection of homogeneous matrix which represents all possible transformations at one step.   (input) 
     *  @param presentInWays    Contains collection of pair <way id, lane number> where object is present.                    (input)
     *  @param osmClass         Osm class object which permits to manipulate osm data. Ensure that the object is up to date.  (input)
     *  @return                 True if the object is accorded to the principal road type in presentInWays. If not return False.
     */
	bool obbVsRoadType(utils::OBBcube obb, utils::transformMatrix tf_mat, std::vector<std::pair<long int,int>> presentInWays, osm osmClass);

};

#endif
