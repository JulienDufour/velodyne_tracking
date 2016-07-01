#include <ros/ros.h>
#include <ros/package.h>

// Img (OpenCV)
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// OSM (OpenStreetMap data)
#include <osm/osm.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

#include <utils/clustering.h>
#include <utils/dataio.h>
#include <objectTracking/objectTracking.h>
#include <objectTracking/object.h>
#include <oxts/oxts.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <signal.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZRGB PointTC;
typedef pcl::PointCloud<PointTC> PointCloudTC;

using namespace pcl;
using namespace std;
using namespace Eigen;

void computeBB(utils::transformMatrix tf_mat, object::ObjectPath objPath, cv::Mat &img, cv::Scalar color){

    std::vector<Eigen::MatrixXd> corners_BB;
    Eigen::Matrix4d quat;
    Eigen::Matrix3f quat_tmp;

    bool bb_in_fov = true;
    bool bb_in_img = true;

    //quaternion to Mat4d
    quat.setIdentity();
    quat_tmp = objPath.quaternion.toRotationMatrix();

    for (int i_q = 0; i_q < 3; ++i_q)
    {
        for (int j_q = 0; j_q < 3; ++j_q)
        {
            quat(i_q,j_q) = (double)quat_tmp(i_q,j_q);
        }
    }

    quat(0,3) = objPath.position(1);
    quat(1,3) = objPath.position(2);
    quat(2,3) = objPath.position(3);

    //Get 8 corners of BB
    corners_BB.push_back( (Eigen::MatrixXd(4,1) << -objPath.x/2.0, objPath.y/2.0, objPath.z/2.0, 1.0).finished() );
    corners_BB.push_back( (Eigen::MatrixXd(4,1) << objPath.x/2.0, objPath.y/2.0, objPath.z/2.0, 1.0).finished() );
    corners_BB.push_back( (Eigen::MatrixXd(4,1) << objPath.x/2.0, -objPath.y/2.0, objPath.z/2.0, 1.0).finished() );
    corners_BB.push_back( (Eigen::MatrixXd(4,1) << -objPath.x/2.0, -objPath.y/2.0, objPath.z/2.0, 1.0).finished() );
    corners_BB.push_back( (Eigen::MatrixXd(4,1) << -objPath.x/2.0, objPath.y/2.0, -objPath.z/2.0, 1.0).finished() );
    corners_BB.push_back( (Eigen::MatrixXd(4,1) << objPath.x/2.0, objPath.y/2.0, -objPath.z/2.0, 1.0).finished() );
    corners_BB.push_back( (Eigen::MatrixXd(4,1) << objPath.x/2.0, -objPath.y/2.0, -objPath.z/2.0, 1.0).finished() );
    corners_BB.push_back( (Eigen::MatrixXd(4,1) << -objPath.x/2.0, -objPath.y/2.0, -objPath.z/2.0, 1.0).finished() );

    //Set Rotation & translation
    for (int k = 0; k < corners_BB.size(); ++k)
    {
        //GPS global -> Cam
        corners_BB[k] = (
            tf_mat.calib_velo_to_cam
            * tf_mat.calib_imu_to_velo
            * tf_mat.oxts_pose.inverse()
            * quat
            * corners_BB[k]
            );

        // All corners are in FOV of Cam ?
        if (corners_BB[k](2,0) < abs(corners_BB[k](0,0))){
            bb_in_fov = false;
        }
    }

    if (bb_in_fov)
    {
        //Set Rotation & translation
        for (int k = 0; k < corners_BB.size(); ++k)
        {
            //CAM -> image
            //[x y z 1] to [su sv s]
            corners_BB[k] = (
                tf_mat.calib_camRect_to_Image
                * tf_mat.calib_cam_to_camRect
                * corners_BB[k]
                );

            //Image -> u,v
            corners_BB[k](0,0) = float(corners_BB[k](0,0))/float(corners_BB[k](2,0));
            corners_BB[k](1,0) = float(corners_BB[k](1,0))/float(corners_BB[k](2,0));

            //All corners are in IMG (u,v) ?
            if (corners_BB[k](0,0)<0 && corners_BB[k](0,0)>=img.size().width && corners_BB[k](1,0)<0 && corners_BB[k](1,0)>=img.size().height){
                bb_in_img = false;
            }
        }

        // If all corners(u,v) are in img
        if (bb_in_img)
        {        
            // Draw BB lines
            cv::line(img, cv::Point( int(corners_BB[0](0,0)), int(corners_BB[0](1,0)) ), cv::Point( int(corners_BB[1](0,0)), int(corners_BB[1](1,0)) ), color, 3, CV_AA);
            cv::line(img, cv::Point( int(corners_BB[0](0,0)), int(corners_BB[0](1,0)) ), cv::Point( int(corners_BB[3](0,0)), int(corners_BB[3](1,0)) ), color, 3, CV_AA);
            cv::line(img, cv::Point( int(corners_BB[0](0,0)), int(corners_BB[0](1,0)) ), cv::Point( int(corners_BB[4](0,0)), int(corners_BB[4](1,0)) ), color, 3, CV_AA);

            cv::line(img, cv::Point( int(corners_BB[1](0,0)), int(corners_BB[1](1,0)) ), cv::Point( int(corners_BB[2](0,0)), int(corners_BB[2](1,0)) ), color, 3, CV_AA);
            cv::line(img, cv::Point( int(corners_BB[1](0,0)), int(corners_BB[1](1,0)) ), cv::Point( int(corners_BB[5](0,0)), int(corners_BB[5](1,0)) ), color, 3, CV_AA);

            cv::line(img, cv::Point( int(corners_BB[2](0,0)), int(corners_BB[2](1,0)) ), cv::Point( int(corners_BB[3](0,0)), int(corners_BB[3](1,0)) ), color, 3, CV_AA);
            cv::line(img, cv::Point( int(corners_BB[2](0,0)), int(corners_BB[2](1,0)) ), cv::Point( int(corners_BB[6](0,0)), int(corners_BB[6](1,0)) ), color, 3, CV_AA);

            cv::line(img, cv::Point( int(corners_BB[3](0,0)), int(corners_BB[3](1,0)) ), cv::Point( int(corners_BB[7](0,0)), int(corners_BB[7](1,0)) ), color, 3, CV_AA);

            cv::line(img, cv::Point( int(corners_BB[4](0,0)), int(corners_BB[4](1,0)) ), cv::Point( int(corners_BB[5](0,0)), int(corners_BB[5](1,0)) ), color, 3, CV_AA);
            cv::line(img, cv::Point( int(corners_BB[4](0,0)), int(corners_BB[4](1,0)) ), cv::Point( int(corners_BB[7](0,0)), int(corners_BB[7](1,0)) ), color, 3, CV_AA);

            cv::line(img, cv::Point( int(corners_BB[5](0,0)), int(corners_BB[5](1,0)) ), cv::Point( int(corners_BB[6](0,0)), int(corners_BB[6](1,0)) ), color, 3, CV_AA);

            cv::line(img, cv::Point( int(corners_BB[6](0,0)), int(corners_BB[6](1,0)) ), cv::Point( int(corners_BB[7](0,0)), int(corners_BB[7](1,0)) ), color, 3, CV_AA);
        }
    }
}

void addBBintoCvImg(utils::transformMatrix tf_mat, unsigned long time, objectTracking track, cv::Mat &img){
    
    std::vector<object> objects;
    std::vector<object::ObjectPath> objPath;
    
    objects = track.getObjects();

    for(int i = 0; i < objects.size(); i++){
        
        objPath = objects[i].getPath();

        for (int j = 0; j<objPath.size(); j++){

            if(objPath[j].time == time){
                //Compute BBox
                computeBB(tf_mat, objPath[j], img, cv::Scalar(int(255*objects[i].getBlue()),int(255*objects[i].getGreen()),int(255*objects[i].getRed())));
            }
        }
    }
}

void addCloudPointsIntoCvImg(utils::transformMatrix tf_mat, const PointCloud<PointXYZRGB>::Ptr veloPoints, cv::Mat &img){

    Eigen::MatrixXd pt(4,1);

    for(auto veloPoint : veloPoints->points){
        
        pt.resize(4,1);
        
        pt(0,0) = (double)veloPoint.x;
        pt(1,0) = (double)veloPoint.y;
        pt(2,0) = (double)veloPoint.z;
        pt(3,0) = 1.0;

        pt = (
            tf_mat.calib_velo_to_cam
            * tf_mat.calib_imu_to_velo
            * tf_mat.oxts_pose.inverse()
            * pt
            );

        //Cam CS : If the point is on fov of camera (90 degree) <-> z >= abs(x)
        if (pt(2,0) >= abs(pt(0,0)))
        {
            pt = (
            tf_mat.calib_camRect_to_Image
            * tf_mat.calib_cam_to_camRect
            * pt
            );        

            //Image -> u,v
            pt(0,0) = float(pt(0,0))/float(pt(2,0));
            pt(1,0) = float(pt(1,0))/float(pt(2,0));

            // If u, v are in img
            if (pt(0,0)>0 && pt(0,0)<1392 && pt(1,0)>0 && pt(1,0)<512)
            {
                cv::circle(img, cv::Point(int(pt(0,0)),int(pt(1,0))), 3, cv::Scalar(veloPoint.b, veloPoint.g, veloPoint.r));
            } 
        }
    }
}

void computeParallelLineFromDistance(osmium::geom::Coordinates pt1, 
    osmium::geom::Coordinates pt2, osmium::geom::Coordinates &pt3, 
    osmium::geom::Coordinates &pt4, double distance){

    double dx = pt2.x - pt1.x;
    double dy = pt2.y - pt1.y;

    osmium::geom::Coordinates perp = osmium::geom::Coordinates(dy,-dx);

    double len = sqrt(pow(perp.x,2)+pow(perp.y,2));

    perp.x = perp.x/len;
    perp.y = perp.y/len;

    perp.x = perp.x * distance;
    perp.y = perp.y * distance;

    pt3.x = pt1.x + perp.x;
    pt3.y = pt1.y + perp.y;

    pt4.x = pt2.x + perp.x;
    pt4.y = pt2.y + perp.y;

}

void mapFromOSM(osm& osmClass, ros::Publisher pub, utils::transformMatrix tf_mat, const std::vector<double> oxtsPose){

    Eigen::MatrixXd pt(4,1);

    visualization_msgs::MarkerArray osmMap;
    std::vector<visualization_msgs::Marker> osmRoads;

    double wL, wR;

    int idx_id=0;

    for(std::map<long int,oneWay>::iterator it_way = osmClass._waysDict.begin(); it_way!=osmClass._waysDict.end(); ++it_way) {

        if(it_way->second.type!=-1){

            if(!osmClass.widthLandR(it_way->first, wL, wR)){
                std::cerr<<"ERROR : osmClass.widthLandR"<<std::endl;
            }

            osmRoads.clear();

            for (int i = 0; i < 3; ++i)
            {
                visualization_msgs::Marker osmData;
                osmData.id=idx_id;
                idx_id++;
                osmData.type = visualization_msgs::Marker::LINE_LIST;
                osmData.action = visualization_msgs::Marker::ADD;
                
                osmData.scale.x = 0.5;
                osmData.scale.y = 0.5;

                osmData.color.r = 0.5;
                osmData.color.g = 0.5;
                osmData.color.b = 0.5;

                if( it_way->second.type == 0 ){
                    osmData.color.r = 1.0;
                    osmData.color.g = 0.0;
                    osmData.color.b = 0.0;
                } 
                else if( it_way->second.type == 1 ){
                    osmData.color.r = 0.0;
                    osmData.color.g = 0.0;
                    osmData.color.b = 1.0;
                }

                if(i==0){
                    osmData.color.r = 1.0;
                    osmData.color.g = 1.0;
                    osmData.color.b = 1.0;
                }

                osmData.color.a = 0.5;

                osmData.header.frame_id = "odom";
                osmData.ns = "Highways";

                osmRoads.push_back(osmData);

            }

            geometry_msgs::Point p1, p2, p3, p4;

            for(int i=0 ; i < it_way->second.myNodes.size()-1 ; i++){

                pt.resize(4,1);
        
                pt(0,0) = it_way->second.myNodes[i].x;
                pt(1,0) = it_way->second.myNodes[i].y;
                pt(2,0) = oxtsPose[2];
                pt(3,0) = 1.0;

                pt = (
                    tf_mat.oxts_pose
                    * pt
                    );

                p1.x=pt(0,0);
                p1.y=pt(1,0);
                p1.z=pt(2,0);

                pt.resize(4,1);
        
                pt(0,0) = it_way->second.myNodes[i+1].x;
                pt(1,0) = it_way->second.myNodes[i+1].y;
                pt(2,0) = oxtsPose[2];
                pt(3,0) = 1.0;

                pt = (
                    tf_mat.oxts_pose
                    * pt
                    );

                p2.x=pt(0,0);
                p2.y=pt(1,0);
                p2.z=pt(2,0);

                osmRoads[0].points.push_back(p1);
                osmRoads[0].points.push_back(p2);

                osmium::geom::Coordinates pt3 = osmium::geom::Coordinates(0,0);
                osmium::geom::Coordinates pt4 = osmium::geom::Coordinates(0,0);

                computeParallelLineFromDistance(osmium::geom::Coordinates(p1.x,p1.y), osmium::geom::Coordinates(p2.x,p2.y), pt3, pt4, wR);

                p3.x=pt3.x;
                p3.y=pt3.y;
                p3.z=p1.z;
                p4.x=pt4.x;
                p4.y=pt4.y;
                p4.z=p2.z;

                osmRoads[1].points.push_back(p3);
                osmRoads[1].points.push_back(p4);

                computeParallelLineFromDistance(osmium::geom::Coordinates(p1.x,p1.y), osmium::geom::Coordinates(p2.x,p2.y), pt3, pt4, -wL);

                p3.x=pt3.x;
                p3.y=pt3.y;
                p3.z=p1.z;
                p4.x=pt4.x;
                p4.y=pt4.y;
                p4.z=p2.z;

                osmRoads[2].points.push_back(p3);
                osmRoads[2].points.push_back(p4);
            }

            for (int i = 0; i < 3; ++i)
            {
                osmMap.markers.push_back(osmRoads[i]);
            }
        }
    }
    pub.publish(osmMap);
}

int main (int argc, char *argv[])
{
    ros::init (argc, argv, "velodyne_tracking");
    
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // NodeHandle & Publisher
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    ros::NodeHandle nh = ros::NodeHandle("~");
    ros::Publisher pub = nh.advertise<PointCloudTC>("clusters", 1);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("lidar", 1);
    ros::Publisher osmMap_pub = nh.advertise<visualization_msgs::MarkerArray>("OSM",1);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher cam_pub = it.advertise("camera/Cam00", 1);

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // PARAMETERS
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    std::string date;
    float voxel_leaf_size, ec_tolerance, ro_radius;
    int ec_min_size, ec_max_size, ro_min_neighbours;
    bool visualize_in_rviz, img_in_rviz, bb_in_rviz, point_in_rviz;
    int tmpid;
    float osmCutRay;

    nh.param<std::string>("date", date, "0");
    nh.param<int>("id", tmpid, 0);
    nh.param<bool>("visualize_in_rviz", visualize_in_rviz, true);
    nh.param<bool>("img_in_rviz", img_in_rviz, true);
    nh.param<bool>("bb_in_rviz", bb_in_rviz, true);
    nh.param<bool>("point_in_rviz", point_in_rviz, true);
    nh.param<float>("voxel_leaf_size", voxel_leaf_size, 0.1);
    nh.param<float>("ec_tolerance", ec_tolerance, 0.5);
    nh.param<int>("ec_min_size", ec_min_size, 50);
    nh.param<int>("ec_max_size", ec_max_size, 1000000);
    nh.param<float>("ro_radius", ro_radius, 1.0);
    nh.param<int>("ro_min_neighbours", ro_min_neighbours, 8);
    nh.param<float>("osmCutRay", osmCutRay, 1.0);

    std::string id = std::to_string(tmpid);
    if(id.length() > 4){
        std::cout << "dataset ID to long, can only be 4 letters, (0000, 0012, 0153,... etc)." << std::endl;
        return 0;
    } else if(id.length() < 4){
        while(4-id.length() > 0){
            id = "0" + id;
        }
    }

    objectTracking track;
    track.initializeNH(&nh);

    oxts positions;
    utils::transformMatrix trans_mat;

    // osm Class to use OpenStreetMap data
    osm osmClass;

    // Create PATHS to correspondant datasets files.
    std::string my_datasets_path = ros::package::getPath("velodyne_tracking") + "/datasets/" + date + "/" + date + "_drive_" + id + "_sync/";
    std::string cloud_path = my_datasets_path + "velodyne_points/data/pcd/don/";
    std::string oxts_path = my_datasets_path + "oxts/data/";
    std::string timeFromVelo_path = my_datasets_path + "velodyne_points/";
    std::string matrix_path = ros::package::getPath("velodyne_tracking") + "/datasets/calib/";
    
    std::string osmFileName;
    nh.param<std::string>("osmFileName", osmFileName, "baden-wuerttemberg-latest.osm.pbf");
    std::string osmFile_fullPath = ros::package::getPath("velodyne_tracking") + "/datasets/osm/data/" + osmFileName;


    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // Load data from files
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    std::cout<<std::endl;
    std::cout<<"======================================="<<std::endl;
    std::cout<<"Load data from files"<<std::endl;
    std::cout<<"======================================="<<std::endl;
    
    //------------------------------------------------
    // Files Names
    //------------------------------------------------
    std::cout<<"File names : "<<"Loading...";
    std::vector<std::string> cloudnames;
    if(!utils::getFileNames(cloud_path, "pcd", cloudnames)){
        std::cerr << "Couldn't load file names, path might be incorrect." << std::endl;
        std::cerr << "Path: " << my_datasets_path << std::endl;
        return 0;
    }
    std::cout << "done." << std::endl << std::endl;

    //------------------------------------------------
    // GPS Data
    //------------------------------------------------
    std::cout<<"GPS Data : "<<"Loading...";
    std::vector<std::string> gpsnames;
    std::vector<double> vec;
    if(utils::getFileNames(oxts_path, "txt", gpsnames)){
        for(auto name : gpsnames){
            utils::readGPSFile(oxts_path, name, "txt", vec);
            positions.newPose(vec);
        }
    }
    std::cout << "done." << std::endl << std::endl;

    //------------------------------------------------
    // TIMESTAMPS
    //------------------------------------------------
    std::cout<<"Timestamps : "<<"Loading...";
    std::vector<unsigned long> times;
    utils::readTimeStamps(timeFromVelo_path, "timestamps", "txt", times);
    std::cout << "done." << std::endl << std::endl;

    //------------------------------------------------
    // Transformation matrix
    //------------------------------------------------
    std::cout<<"Transformation matrix : "<<std::endl;
    
    std::cout<<"-> calib_imu_to_velo : Loading...";
    if(!utils::readRigidTransformFile(matrix_path, "calib_imu_to_velo", "txt", trans_mat.calib_imu_to_velo)){
        std::cerr << "ERROR : Couldn't load calib_imu_to_velo, path might be incorrect." << std::endl;
        std::cerr << "Path: " << matrix_path << std::endl;
        return -1;
    }
    else std::cout<<"done."<<std::endl;
    //std::cout<<"calib_imu_to_velo\n"<<trans_mat.calib_imu_to_velo<<std::endl;

    std::cout<<"-> calib_velo_to_cam : Loading...";
    if(!utils::readRigidTransformFile(matrix_path, "calib_velo_to_cam", "txt", trans_mat.calib_velo_to_cam)){
        std::cerr << "ERROR : Couldn't load calib_velo_to_cam, path might be incorrect." << std::endl;
        std::cerr << "Path: " << matrix_path << std::endl;
        return -1;
    }
    else std::cout<<"done."<<std::endl;
    //std::cout<<"calib_velo_to_cam\n"<<trans_mat.calib_velo_to_cam<<std::endl;

    std::cout<<"-> calib_cam_to_cam & calib_cam_to_camRect : Loading...";
    if(!utils::readCamTransformFile(matrix_path, "calib_cam_to_cam", "txt", trans_mat.calib_cam_to_camRect, trans_mat.calib_camRect_to_Image)){
        std::cerr << "ERROR : Couldn't load calib_cam_to_cam, path might be incorrect." << std::endl;
        std::cerr << "Path: " << matrix_path << std::endl;
        return -1;
    }
    else std::cout<<"done."<<std::endl;
    //std::cout<<"calib_cam_to_camRect\n"<<trans_mat.calib_cam_to_camRect<<std::endl;
    //std::cout<<"calib_camRect_to_Image\n"<<trans_mat.calib_camRect_to_Image<<std::endl;

    trans_mat.oxts_pose = positions.firstPoseMatrix();
    std::cout<<std::endl;

    //------------------------------------------------
    // OpenStreetMap Data
    //------------------------------------------------
    std::cout<<"OpenStreetMap Data : "<<std::endl;
    // osm Class to use OpenStreetMap data
    osmClass = osm(positions.firstPose()[1], positions.firstPose()[0], osmCutRay);

    osmium::Location min,max;
    osmClass.getMinMaxGPSPoint(osmClass._origin,min,max,osmClass._cutRay);

    std::cout<<"-> Cut map <"<<osmFileName<<"> to increase computation : Progress...";
    osmium::io::Reader osmReader(osmFile_fullPath);
    if(!osmClass.isOnMap(osmClass._origin, osmReader.header())
        || !osmClass.isOnMap(min, osmReader.header())
        || !osmClass.isOnMap(max, osmReader.header())
        ){
        std::cerr<<"ERROR : Current dataset is not in the map."<<std::endl;
        std::cerr << "Path: " << osmFile_fullPath << std::endl;
        return -1;
    }
    osmReader.close();
    osmClass.cutOsmMap(ros::package::getPath("velodyne_tracking"), osmFileName, min, max);
    std::cout<<"done."<<std::endl;
    
    std::cout<<"-> Around map : Loading...";
    osmClass.readOsmFile(ros::package::getPath("velodyne_tracking"));
    std::cout<<"done."<<std::endl;

    std::cout<<"-> KdTree : Initialization...";
    if(!osmClass.setKdtree()) return -1;
    std::cout<<"done."<<std::endl<<std::endl;

    std::cout<<"======================================="<<std::endl;
    std::cout<<"All data are loaded"<<std::endl;
    std::cout<<"======================================="<<std::endl;

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // Start Process
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if(visualize_in_rviz){
        mapFromOSM(osmClass, osmMap_pub, trans_mat, positions.firstPose());
    }

    geometry_msgs::PoseStamped posestamped;
    posestamped.header.frame_id="odom";
    
    posestamped.pose.orientation.x = 0;
    posestamped.pose.orientation.y = -1;
    posestamped.pose.orientation.z = 0;
    posestamped.pose.orientation.w = 1;
    
    posestamped.pose.position.x = 0.0;
    posestamped.pose.position.y = 0.0;
    posestamped.pose.position.z = 0.0;

    pcl::PCDReader reader;
    std::cout << " " << std::endl;
    for(int j = 0; j < cloudnames.size()-1; ++j)
    {
        std::cout << "\rProgress: " << std::round(j/(float)(cloudnames.size()-1)*100.0) << "\%";
        std::cout.flush();

        PointCloud<PointNormal>::Ptr cloud (new PointCloud<PointNormal>());
        PointCloud<PointNormal>::Ptr tmp_cloudA (new PointCloud<PointNormal>());
        PointCloud<PointNormal>::Ptr tmp_cloudB (new PointCloud<PointNormal>());
        PointCloud<PointNormal>::Ptr cloud_load (new PointCloud<PointNormal>());

        // Read PCL pointcloud.
        reader.read<PointNormal>(cloud_path + cloudnames[j] + ".pcd", *cloud_load, false);

        // Voxel grid filter.
        pcl::VoxelGrid<PointNormal> sor;
        sor.setInputCloud (cloud_load);
        sor.setLeafSize (voxel_leaf_size,voxel_leaf_size,voxel_leaf_size);
        sor.filter (*tmp_cloudA);

        // Radius outlier removal.
        utils::roRemoval(tmp_cloudA, ro_radius, ro_min_neighbours);

        // Put VelodynePoints into GPS CS
        Matrix4d transVtoI = trans_mat.calib_imu_to_velo.inverse();
        pcl::transformPointCloud(*tmp_cloudA, *tmp_cloudB, transVtoI);

        // Find transformation based on GPS data and publish laser pose.
        Matrix4d trans = positions.returnPose();
        posestamped.pose.position.x = trans(0,3);
        posestamped.pose.position.y = trans(1,3);
        posestamped.pose.position.z = trans(2,3);

        // Transform cloud based on GPS data.
        pcl::transformPointCloud(*tmp_cloudB, *cloud, trans);

        // Cluster the results from the DoN method using euclidean clustering.
        std::vector<PointCloud<PointXYZ>::Ptr> outvec;
        utils::ecClustering<PointNormal>(cloud, ec_tolerance, ec_min_size, ec_max_size, outvec);

        // Update the list of objects using the resulting clusters.
        PointCloud<PointXYZRGB>::Ptr colored_cloud (new PointCloud<PointXYZRGB>());
        trans_mat.oxts_pose = positions.returnPose(j);
        track.update(times[j], positions.firstPoseMatrix(), trans_mat, osmClass, outvec, colored_cloud);

        if(visualize_in_rviz){

            if(img_in_rviz){
                //Load img
                std::string img_id = std::to_string(j);
                
                while(10-img_id.length() > 0){
                img_id = "0" + img_id;
                }

                std::string img_path = my_datasets_path + "image_00/data/" + img_id + ".png";

                //Img from Cam00 is in GRayscale but load it in colorscale to put into the colored BB
                //CV_LOAD_IMAGE_GRAYSCALE     
                cv::Mat cam_img = cv::imread(img_path, CV_LOAD_IMAGE_COLOR);
                
                if(bb_in_rviz) addBBintoCvImg(trans_mat, times[j], track, cam_img);
                if(point_in_rviz) addCloudPointsIntoCvImg(trans_mat, colored_cloud, cam_img);

                //Convert cv img into sensor_msg
                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cam_img).toImageMsg();
                cam_pub.publish(img_msg);
            }

            // Publish colored cloud to RVIZ.
            colored_cloud->header.frame_id="odom";
            pub.publish(colored_cloud);
            pose_pub.publish(posestamped);
        }
        if(!ros::ok()) break;
    }
    std::cout << " " << std::endl;
    std::cout << "Number of seen objects: " << track.getSize() << std::endl;

    std::cout << "delete all paths and republish actual moving paths." << std::endl;
    track.publishAllPaths();

    // write paths to csv file.
    std::string save_path;
    nh.param<std::string>("save_path", save_path, my_datasets_path+"path");
    utils::createDirectories(save_path);

	std::cout << "write paths to csv file in :" << save_path << std::endl;
    track.writePathsToFile(positions.firstPose(), save_path, date + "-" + id);
    std::cout<<"done."<<std::endl;

    return 0;
}
