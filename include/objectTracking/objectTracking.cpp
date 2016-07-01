#include <objectTracking/objectTracking.h>
#include <utils/clustering.h>
#include <utils/dataio.h>
#include <tf2_kdl/tf2_kdl.h>
#include <iostream>
#include <fstream>



void objectTracking::initializeNH(ros::NodeHandle *nh){
    nh_ = nh;
    pub_ = nh_->advertise<visualization_msgs::Marker>("visualization_msgs", 1);
    publish_ = true;

    // load parameters.
    nh_->param<float>("max_distance", max_distance_, 3.0);
    nh_->param<float>("path_length", path_length_, 10.0);
    nh_->param<float>("time_since_measurement", time_since_measurement_, 1.0);
    nh_->param<int>("k_kdTree", k_kdTree, 10);
}

bool objectTracking::obbIsOnRoad(utils::OBBcube obb, std::vector<std::pair<long int, int>> &presentInWays, Eigen::Matrix4d oxts_first_pose, osm osmClass){

    // Put Center of BBox object in Mercator CS
    Eigen::MatrixXd obbInMerc;

    obbInMerc.resize(4,1);
    obbInMerc(0,0) = static_cast<double>(obb.position(0));
    obbInMerc(1,0) = static_cast<double>(obb.position(1));
    obbInMerc(2,0) = static_cast<double>(obb.position(2));
    obbInMerc(3,0) = 1.0;

    obbInMerc = oxts_first_pose.inverse() * obbInMerc;

    // Use osm and kdtreeNearestSearch to determine if this object is on road
    std::vector<double> pointRadiusSquaredDistance;
    std::vector<std::pair<double,double>> pointsNearestSearch;

    if(!osmClass.kdtreeNearestSearch(obbInMerc(0,0), obbInMerc(1,0),pointRadiusSquaredDistance, pointsNearestSearch, this->k_kdTree)){
        std::cerr<<"No neighbors in nearest search "<<std::endl;
        return false;
    }

    std::vector<std::pair<int, int>> betweenNodes; // not used here! Impossible to pass or set a default value because it a non-const reference
    return osmClass.objectIsOnRoad(pointsNearestSearch, osmium::geom::Coordinates(obbInMerc(0,0), obbInMerc(1,0)), presentInWays, betweenNodes);
}

bool objectTracking::obbVsRoadType(utils::OBBcube obb, utils::transformMatrix tf_mat, std::vector<std::pair<long int,int>> presentInWays, osm osmClass){

    // Get label of object
    // Recall : idLabels = {{"Car", 1}, {"Cyclist", 2}, {"Pedestrian", 3},
    // {"Van", 4}, {"Truck", 5}, {"Tram", 6}, {"Person_sitting", 7}, {"Misc", 8}};
    int label = lab_.classObb(obb, tf_mat);

    // Get the major type in presentInWays
    // Recall : 0 if the way is a Car way, 1 if it is a Bicycle/Pedestrian way, -1 if it isnt a way.
    int nCarRoad = 0;
    int nPedesRoad = 0;
    for(auto way_id : presentInWays){
        if(osmClass._waysDict[way_id.first].type == 0) nCarRoad++;
        else nPedesRoad++;
    }

    if(nCarRoad >= nPedesRoad && (label==1 || label==2 || label==4 || label==5)){
        return true;
    }
    else if(nCarRoad < nPedesRoad && (label==2 || label==3)){
        return true;
    }
    else return false;
}

void objectTracking::update(unsigned long time,
                            Eigen::Matrix4d oxts_first_pose,
                            utils::transformMatrix tf_mat,
                            osm osmClass,
                            std::vector<PointCloud<PointXYZ>::Ptr> &clouds,
                            PointCloud<PointXYZRGB>::Ptr colored_cloud){

	std::vector<utils::OBBcube> obbvec;
    utils::getOBB(clouds, obbvec);
    std::vector< std::pair<long int,int>> presentInWays;

    // Nothing has been seen previously.
    // Initialize objects, load all seen objects.
    if(objects_.size() == 0){
        for(auto obb : obbvec){
            presentInWays.clear();
            if(objectTracking::obbIsOnRoad(obb, presentInWays, oxts_first_pose, osmClass)){
                if(objectTracking::obbVsRoadType(obb, tf_mat, presentInWays, osmClass)){
                    objectTracking::createObject(time, obb, filter_, objects_);
                }
            }
    	}
    	return;
    }

    // Match the detected objects to already seen objects
    // Criteria: distance and shape (Possibly add previous movement).

    // First step is to predict the state of all obects
    for(auto &object : objects_){
        object.predictState(time);
    }

    float r,g,b;
    for(int i = 0; i < clouds.size(); i++){

        // Find the euclidean distance between the centroids of two 3d objects.
        auto func_dist = [] (utils::OBBcube c1, ObjectEKF::Vector ob){
            return sqrt(pow(c1.position[0]-ob(1),2) + pow(c1.position[1]-ob(2),2)); //+ pow(c1.position[2]-ob(3),2) );
        };

        float mindist = 1000000000.0;
        int mindistIdx = -1;
        float dist = 0.0;

        for(auto j = 0; j < objects_.size(); j++){
            if(objects_[j].timeSinceMeasurement() > time_since_measurement_) continue;
            dist = func_dist(obbvec[i], objects_[j].returnState());
            if(dist < mindist){
                // Found a probable match for that object, store it.
                mindist = dist;
                mindistIdx = j;
            }
        }

        if(mindistIdx == -1 || mindist >= max_distance_){

            // No match was found, need to create a new object with the coordinates.
            presentInWays.clear();
            if(objectTracking::obbIsOnRoad(obbvec[i], presentInWays, oxts_first_pose, osmClass)){
                if(objectTracking::obbVsRoadType(obbvec[i], tf_mat, presentInWays, osmClass)){
                    objectTracking::createObject(time, obbvec[i], filter_, objects_);
                }
            }

        } else {
            // Previously seen object seen again, update position and color appropriately.
            objects_[mindistIdx].update(time, obbvec[i]);

            r = objects_[mindistIdx].getRed();
            g = objects_[mindistIdx].getGreen();
            b = objects_[mindistIdx].getBlue();
            PointCloud<PointXYZRGB>::Ptr tmp_cloud (new PointCloud<PointXYZRGB>());
            PointXYZRGB cPoint;
            for(auto point : clouds[i]->points){
                cPoint.x = point.x;
                cPoint.y = point.y;
                cPoint.z = point.z;
                cPoint.r = r*255.0;
                cPoint.g = g*255.0;
                cPoint.b = b*255.0;
                tmp_cloud->points.push_back(cPoint);
            }
            *colored_cloud += *tmp_cloud;
        }
    }
};

void objectTracking::createObject(unsigned long time,
                                utils::OBBcube obb,
                                ObjectEKF &filter,
                                std::vector<object> &objectvec){


	Matrix P0;
	static const double _P0[] = {
		500.0*100.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 500.0*100.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 5.0*10.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 80.0*10.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 80.0*10.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 5.0*10.0
	};
	P0.assign(6,6,_P0);

	Vector x(6);
	x(1) = obb.position[0];
	x(2) = obb.position[1];
	x(3) = obb.position[2];
	x(4) = 0;
	x(5) = 0;
	x(6) = 0;

	filter.initT(time,x,P0);
    object tmpobb(time, filter, obb, path_length_, time_since_measurement_);
    h_ += ratio_;
    h_ = fmod(h_,1);
    tmpobb.setColor(h_);

    if(publish_){
        tmpobb.initializeNH(nh_, objects_.size());
    }
    objectvec.push_back(tmpobb);
}

void objectTracking::publishAllPaths(void){

    // Start by deleting all paths currently visualized in RVIZ.
    // (There is a bug where some paths don't get deleted correctly in RVIZ  when
    // deleted individually at runtime. (Don't know why!!!)

    if(publish_){

        visualization_msgs::Marker line_list;
        line_list.type   = visualization_msgs::Marker::LINE_LIST;
        line_list.action = 3; // 3 means visualization_msgs::Marker::DELETEALL
        line_list.header.frame_id = "odom";
        line_list.ns              = "paths";
        pub_.publish(line_list);

        for(auto &object : objects_){
            if(object.isMoving()){
                object.publishPath();
            }
        }
    }
}

void objectTracking::writePathsToFile(std::vector<double> oxts_origin, std::string save_path, std::string file_name){

    utils::writePathsToFile(oxts_origin, objects_, save_path, file_name);
};
