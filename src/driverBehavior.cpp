/***************************************************************************
 *  Copyright (C) 2016  by DUFOUR Julien                                   *
 *                                                                         *
 *  This program is free software: you can redistribute it and/or modify   *
 *  it under the terms of the GNU General Public License as published by   *
 *  the Free Software Foundation, either version 3 of the License, or      *
 *  (at your option) any later version.                                    *
 *                                                                         *
 *  This program is distributed in the hope that it will be useful,        *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *  GNU General Public License for more details.                           * 
 *                                                                         *
 *  You should have received a copy of the GNU General Public License      *
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.  *
 ***************************************************************************/

/**
 * @file   driverBehavior.cpp
 * @author DUFOUR Julien
 * @date   June, 2016
 * @brief  File containing driverBehavior main program.
 *
 * From the OSM data and the desired tracked object, this program make a collection of velocity profiles for all next possible paths.
 * Also, timestamp by timestamp, the program plots and updates the desired velocity profile with the current speed of the desired tracked object.
 */

// Ros
#include <ros/ros.h>
#include <ros/package.h>

#include <utils/driverBehavior_utils.h>


int main (int argc, char *argv[])
{

    ros::init (argc, argv, "velodyne_tracking");

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // PARAMETERS
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    /* Path */
    std::string datasetDate;      /*!< Date of the kitti dataset.*/
    std::string datasetId;        /*!< Id of the kitti dataset.*/
    std::string pkg;              /*!< Full path to the package folder.*/
    std::string dataset_path;     /*!< Full path to the dataset folder.*/
    std::string osmFileName;      /*!< Input osm file name.*/
    std::string osmFileFullPath;  /*!< Full path to osm input file.*/

    /* Osm parameterization */
    double osmCutRay;    /*!< Size of map circle diameter. Center is represented by kitti car.*/
    bool cutChoose;      /*!< Recut the input osm file or not.*/
    double minPathSize;  /*!< Minimal desired path size (in meters) for all paths.*/
    double scale;        /*!< Permit to obtain a Mercator CS in meters.*/

    /* Drive Behavior */
    std::vector<velocityProfile> veloProfiles;             /*!< For each possible path, a velocity profile object.*/
    std::vector<std::vector<double>> xPaths;               /*!< For each possible path, list of X coordinates which represent it (in Mercator CS in meters).*/
    std::vector<std::vector<double>> yPaths;               /*!< For each possible path, list of Y coordinates which represent it (in Mercator CS in meters).*/
    std::vector<std::vector<double>> xControlPointsPaths;  /*!< For each possible curved path, list of X coordinates which represent his control points (in Mercator CS in meters).*/
    std::vector<std::vector<double>> yControlPointsPaths;  /*!< For each possible curved path, list of Y coordinates which represent his control points (in Mercator CS in meters).*/
    int refIdObject;                                       /*!< Desired object represented by his id.*/
    int refIdPath;                                         /*!< Desired path represented by an index. For the desired object.*/
    int refIdVeloP;                                        /*!< Desired velocity profile represented by an index. For the desired object and path.*/
    double distanceForObject = 0.0;                        /*!< Current object distance (refIdObject), in meters, from the origin of paths.*/
    int viewForObject = 0;                                 /*!< Used to observe the desired object when his EKF have converged.*/
    bool firstTimeForObject = true;                        /*!< When this bool pass to false, it specifies that the object EKF have converged.*/
    std::pair<double,double> posPrecForObject;             /*!< Used to compute the object deplacement.*/

    /* Velocity profile parameterization */
    bool trafficRulesConstraints;   /*!< Represents if yes or not these constraints must be add during velocity profile creation (user choice).*/
    bool curvatureConstraints;      /*!< Represents if yes or not these constraints must be add during velocity profile creation (user choice).*/
    bool accelerationConstraints;   /*!< Represents if yes or not these constraints must be add during velocity profile creation (user choice).*/
    bool decelerationConstraints;   /*!< Represents if yes or not these constraints must be add during velocity profile creation (user choice).*/
    bool idmConstraints;            /*!< Represents if yes or not these constraints must be add during velocity profile creation (user choice).*/
    double reSamplePathMinDist;     /*!< Minimal distance between two points in the final path (in meters).*/
    double smoothHardTurnMinBound;  /*!< Minimal bound which define one hard turn (in degree).*/
    double smoothHardTurnMaxBound;  /*!< Maximal bound which define one hard turn (in degree).*/
    double smoothSoftTurnMinBound;  /*!< Minimal bound which define one soft turn (in degree).*/
    double smoothSoftTurnMaxBound;  /*!< Maximal bound which define one soft turn (in degree).*/
    int nbSampleCurvProfile;        /*!< Number of desired samples in the curved path. For good result, assure that it is bigger than the size of the path.*/
    double amountOfSmoothing;       /*!< Amount of smoothing for the curvature profile.*/
    double stopDistance;            /*!< Stopping distance before a traffic rule (in meters).*/
    double maxSpeedPercentP1;       /*!< Percentage of law speed for the defensive profile, p1, (in %).*/
    double maxSpeedPercentP2;       /*!< Percentage of law speed for the normal profile, p2, (in %).*/
    double maxSpeedPercentP3;       /*!< Percentage of law speed for the sporty profile, p3, (in %).*/
    double aLatP1;                  /*!< Maximal lateral acceleration for the defensive profile, p1, (in m/s²).*/
    double aLatP2;                  /*!< Maximal lateral acceleration for the normal profile, p2, (in m/s²).*/
    double aLatP3;                  /*!< Maximal lateral acceleration for the sporty profile, p3, (in m/s²).*/
    double aP1;                     /*!< Maximum acceleration parameter for the defensive profile, p1, (in m/s²).*/
    double aP2;                     /*!< Maximum acceleration parameter for the normal profile, p2, (in m/s²).*/
    double aP3;                     /*!< Maximum acceleration parameter for the sporty profile, p3, (in m/s²).*/
    double gP1;                     /*!< Minimal velocity gradient for the defensive profile, p1, (in /s).*/
    double gP2;                     /*!< Minimal velocity gradient for the normal profile, p2, (in /s).*/
    double gP3;                     /*!< Minimal velocity gradient for the sporty profile, p3, (in /s).*/
    double equalDiffPrecent;        /*!< Permit to delete same case in all possible velocity profiles following an acceptable difference (in %).*/
    double aExpo;                   /*!< Acceleration exponent.*/
    double comfDeceleration;        /*!< Comf. deceleration (in m/s²).*/
    double d0;                      /*!< Min. gap to leading vehicule (in m).*/
    double timeGap;                 /*!< Time gap to leading vehicule (in s).*/

    /* Plot parameterization */
    double preDistForObject = 0.0;        /*!< Used to update the velocity profile plot with IDM time by time modification into velocity profiles.*/
    std::map<int,oneColor> colorObjDict;  /*!< Dictionary which links id of object and color.*/
    bool seeAllPaths;                     /*!< Represents if the user wants to see all possible paths for his selected object.*/
    bool seeAllCurvatureProfiles;         /*!< Represents if the user wants to see all possible curvature profiles for his selected object.*/
    bool seeAllVelocityProfiles;          /*!< Represents if the user wants to see all possible velocity profiles for his selected object.*/
    bool pauseBetweenEachTime;            /*!< Represents if the user wants enable a waitKey between each time contained in timestamps.*/
    bool defaultPlotDriver;               /*!< Represents if the user wants to use the default driver for PlPlot (xwin).*/
    int screenHeight;                     /*!< Screen dimension (in pixel) : Height.*/
    int screenWidth;                      /*!< Screen dimension (in pixel) : Width.*/


    /* Process */
    std::vector<unsigned long> times;                                /*!< Timestamps for the objects.*/
    std::vector<double> gpsFirstPose;                                /*!< Contains first GPS pose. Permit to create OSM object and obtain gpsTransformMatrix.*/
    std::vector< std::vector<double> > gpsPoses;                     /*!< Contains all informations of kitti car at each time. From GPS.*/
    std::vector< std::vector<utils::ObjectPath> > myTracks;          /*!< Informations about tracked object obtained from csv file.*/
    Eigen::Matrix4d gpsTransformMatrix;                              /*!< Transformation matrix : Global CS -> Mercator CS.*/
    std::map<unsigned long,std::vector<oneObject>> objDict;          /*!< Dictionary which link a list of objects with time.*/
    std::map<std::pair<unsigned long, int>, oneObject> objectByTime; /*!< Dictionary which link one object with time and his id.*/
    std::vector<int> driverBehaviorEval = std::vector<int>{0,0,0};   /*!< For each profile (defensive, normal and sporty) count, at each step, the most likely profile for the desired object.*/

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // Load data
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    std::cout<<std::endl;
    std::cout<<"======================================="<<std::endl;
    std::cout<<"Load data"<<std::endl;
    std::cout<<"======================================="<<std::endl<<std::endl;

    //------------------------------
    // NodeHandle & Package (ROS)
    //------------------------------
    std::cout<<"NodeHandle & Package (ROS) : "<<std::endl<<"Loading...";
    ros::NodeHandle nh = ros::NodeHandle("~");

    /* Datasets informations */
    nh.param<std::string>("date", datasetDate, "0");
    int datasetIdTmp;
    nh.param<int>("id", datasetIdTmp, 0);

    // Check input param
    if(datasetDate.compare("0") == 0 || datasetIdTmp == 0){
        std::cerr<<std::endl<<"User Error : You must enter the date and the id of the desired dataset"<<std::endl;
        return -1;
    }
    
    datasetId = std::to_string(datasetIdTmp);
    if(datasetId.length() > 4){
        std::cerr << "dataset ID to long, can only be 4 letters, (0000, 0012, 0153,... etc)." << std::endl;
        return -1;
    } else if(datasetId.length() < 4){
        while(4-datasetId.length() > 0){
            datasetId = "0" + datasetId;
        }
    }
    std::cout << "...";

    /* Package & dataset path */
    pkg = ros::package::getPath("velodyne_tracking");
    dataset_path = pkg + "/datasets/" + datasetDate + "/" + datasetDate + "_drive_" + datasetId + "_sync/";
    std::cout << "...";

    /* OSM option */
    nh.param<double>("minPathSize", minPathSize, 140.0);
    nh.param<std::string>("osmFileName", osmFileName, "baden-wuerttemberg-latest.osm.pbf");
    nh.param<double>("osmCutRay", osmCutRay, 1.2);
    osmFileFullPath = pkg + "/datasets/osm/data/" + osmFileName;
    nh.param<bool>("cut", cutChoose, true);
    std::cout << "...";

    /* Driver Behavior */
    nh.param<int>("driverIdObject", refIdObject, 0);
    nh.param<int>("driverIdPath", refIdPath, 0);
    nh.param<int>("driverIdVeloP", refIdVeloP, 0);
    std::cout << "...";

    /* Velocity profiles parameters */
    nh.param<bool>("trafficRulesConstraints", trafficRulesConstraints, true);
    nh.param<bool>("curvatureConstraints", curvatureConstraints, true);
    nh.param<bool>("accelerationConstraints", accelerationConstraints, true);
    nh.param<bool>("decelerationConstraints", decelerationConstraints, true);
    nh.param<bool>("idmConstraints", idmConstraints, true);
    nh.param<double>("reSamplePathMinDist", reSamplePathMinDist, 15.0);
    nh.param<double>("smoothHardTurnMinBound", smoothHardTurnMinBound,  65.0);
    nh.param<double>("smoothHardTurnMaxBound", smoothHardTurnMaxBound, 115.0);
    nh.param<double>("smoothSoftTurnMinBound", smoothSoftTurnMinBound,  20.0);
    nh.param<double>("smoothSoftTurnMaxBound", smoothSoftTurnMaxBound, 160.0);
    nh.param<int>("nbSampleCurvProfile", nbSampleCurvProfile, 600);
    nh.param<double>("amountOfSmoothing", amountOfSmoothing, 0.1);
    nh.param<double>("stopDistance", stopDistance,  3.5);
    nh.param<double>("maxSpeedPercentP1", maxSpeedPercentP1,  80.0);
    nh.param<double>("maxSpeedPercentP2", maxSpeedPercentP2,  90.0);
    nh.param<double>("maxSpeedPercentP3", maxSpeedPercentP3, 100.0);
    nh.param<double>("aLatP1", aLatP1, 2.00);
    nh.param<double>("aLatP2", aLatP2, 2.75);
    nh.param<double>("aLatP3", aLatP3, 3.50);
    nh.param<double>("aP1", aP1, 1.5);
    nh.param<double>("aP2", aP2, 2.0);
    nh.param<double>("aP3", aP3, 2.5);
    nh.param<double>("gP1", gP1, 0.15);
    nh.param<double>("gP2", gP2, 0.20);
    nh.param<double>("gP3", gP3, 0.25);
    nh.param<double>("equalDiffPrecent", equalDiffPrecent, 0.0);
    nh.param<double>("aExpo", aExpo, 4.0);
    nh.param<double>("comfDeceleration", comfDeceleration, 3.0);
    nh.param<double>("d0", d0, 2.0);
    nh.param<double>("timeGap", timeGap, 0.8);
    std::cout << "...";

    /* Plot parameters */
    nh.param<bool>("seeAllPaths", seeAllPaths, false);
    nh.param<bool>("seeAllCurvatureProfiles", seeAllCurvatureProfiles, false);
    nh.param<bool>("seeAllVelocityProfiles", seeAllVelocityProfiles, false);
    nh.param<bool>("pauseBetweenEachTime", pauseBetweenEachTime, false);
    nh.param<bool>("defaultPlotDriver", defaultPlotDriver, true);
    nh.param<int>("screenHeight", screenHeight, 600);
    nh.param<int>("screenWidth", screenWidth, 800);

    std::cout << "done." << std::endl << std::endl;

    //------------------------------------------------
    // TIMESTAMPS
    //------------------------------------------------
    std::cout<<"Timestamps : "<<std::endl<<"Reading ";
    utils::readTimeStamps(dataset_path + "velodyne_points/", "timestamps", "txt", times);
    std::cout << "done." << std::endl << std::endl;

    //------------------------------------------------
    // GPS Data
    //------------------------------------------------
    std::cout<<"GPS Data : "<<"Loading...";
    std::vector<std::string> gpsnames;
    std::vector<double> gpsPose;

    if(utils::getFileNames(dataset_path + "oxts/data/", "txt", gpsnames)){
        for(auto name : gpsnames){
            utils::readGPSFile(dataset_path + "oxts/data/", name, "txt", gpsPose);
            gpsPoses.push_back(gpsPose);
        }
    }
    std::cout << "done." << std::endl << std::endl;

    //------------------------------------------------
    // CSV File
    //------------------------------------------------
    std::cout<<"CSV File : "<<std::endl;
    std::cout<<"-> Read file : Reading...";
    utils::readPathsFromFile((pkg + "/datasets/csv/").c_str(), datasetDate + "-" + datasetId, gpsFirstPose, myTracks );
    std::cout << "done." << std::endl << std::endl;

    // Check if refIdObject exists
    // Possible id are represented by the size of myTracks (from CSV file) + 1 (for the kitti car).
    if(refIdObject < 0 || refIdObject > static_cast<int>(myTracks.size())){
        std::cerr<<"User Error : Desired object id must be included into [0.."<<myTracks.size()<<"]"<<std::endl;
        return -1;
    }

    //------------------------------------------------
    // OSM data
    //------------------------------------------------
    std::cout<<"OSM : "<<std::endl;
    osm osmClass = osm(/*lon*/ gpsFirstPose[1], /*lat*/ gpsFirstPose[0], osmCutRay);

    if(cutChoose){

        osmium::Location min,max;

        std::cout<<"-> Get Coordinates of the bbox (for the purpose of cut) : Processing...";
        osmClass.getMinMaxGPSPoint(osmClass._origin,min,max,osmClass._cutRay);
        std::cout << "done." << std::endl;

        std::cout<<"-> All corners are in <"<<osmFileName<<"> : ";
        osmium::io::Reader reader(osmFileFullPath);
        if(!osmClass.isOnMap(osmClass._origin, reader.header())
            || !osmClass.isOnMap(min, reader.header())
            || !osmClass.isOnMap(max, reader.header())
            ){
            std::cout<<"No"<<std::endl;
            std::cerr<<"User Error : Please change input parameters in <driverBehaviorParameters.yaml>."<<std::endl;
            return -1;
        }
        else std::cout<<"Yes"<<std::endl;
        reader.close();

        std::cout<<"-> Cut map <"<<osmFileName<<"> to increase computation : Progress...";
        osmClass.cutOsmMap(pkg, osmFileName, min, max);
        std::cout << "done." << std::endl;
    }

    std::cout<<"-> Around map : Loading...";
    osmClass.readOsmFile(pkg);
    std::cout<<"done."<<std::endl;

    std::cout<<"-> KdTree : Initialization...";
    osmClass.setKdtree();
    std::cout<<"done."<<std::endl<<std::endl;

    //------------------------------------------------
    // Extra data
    //------------------------------------------------
    std::cout<<"Extra : "<<std::endl;
    // Get Transform matrix Global CS -> Mercator CS
    std::cout<<"-> Transform matrix Global CS -> Mercator CS : Loading...";
    getTrMatrixFromFirstPose(gpsFirstPose,gpsTransformMatrix);
    std::cout << "done." << std::endl;

    // Get Color for each tracked object
    std::cout<<"-> Set one Color/tracked object : Setting...";
    colorObjDict = getRandColorForEachObject(myTracks.size()+1);
    std::cout << "done." << std::endl;

    // Get object list sorted by timestamps
    std::cout<<"-> Sorted objects by timestamps : Sorting...";
    sortCsvDataByTimestamp(gpsPoses, times, myTracks, gpsTransformMatrix, osmClass, objDict, objectByTime);
    std::cout << "done." << std::endl << std::endl;

    std::cout<<"======================================="<<std::endl;
    std::cout<<"All data are loaded"<<std::endl;
    std::cout<<"======================================="<<std::endl;

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // Init PlPlot
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    /* Map */
    plstream *plsMap;
    plsMap = new plstream();
    plsMap->spage(0.0, 0.0, 0.5*screenWidth, 0.5*screenHeight, 0.0, 0.0);
    initPlPlot(plsMap, colorObjDict, defaultPlotDriver);

    /* Curve Profile */
    plstream *plsCurveProfile;
    plsCurveProfile = new plstream();
    plsCurveProfile->spage(0.0, 0.0, 0.5*screenWidth, 0.5*screenHeight, 0.5*screenWidth, 0.0);
    initPlPlot(plsCurveProfile, colorObjDict, defaultPlotDriver);

    /* Velocity Profile */
    plstream *plsVelocityProfile;
    plsVelocityProfile = new plstream();
    plsVelocityProfile->spage(0.0, 0.0, 0.5*screenWidth, 0.5*screenHeight, 0.5*screenWidth, 0.5*screenHeight);
    initPlPlot(plsVelocityProfile, colorObjDict, defaultPlotDriver);

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // Process
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    for (int idxTime = 0; idxTime < times.size(); ++idxTime)
    {
    	// Update Map
    	drawMap(plsMap, gpsPoses[idxTime], osmClass, utils::longTimetoStringTime(times[idxTime]), false);

    	std::vector<int> currentObjectsIds;
    	for(auto obj : objDict[times[idxTime]]){

    		currentObjectsIds.push_back(obj.id);

    		// Update Map
    		addObject(plsMap, obj, osmClass, 4);
    	} 
    	
    	// Works only if refIdObject exists at this time
    	if(std::find(currentObjectsIds.begin(), currentObjectsIds.end(), refIdObject) != currentObjectsIds.end()){

    		oneObject obj = objectByTime[std::make_pair(times[idxTime],refIdObject)];

    		if(firstTimeForObject && viewForObject < 4){
    			viewForObject++;
    		}
    		else if(firstTimeForObject && obj.currentWay != -1){
    			firstTimeForObject = false;

    			// Create all next possible paths
    			std::vector<std::vector<long int>> nextPossiblePaths;
                osmClass.nextPossiblePaths(obj.currentWay, obj.indexNodesInCurrentWay, obj.heading, obj.posMerc, nextPossiblePaths, minPathSize);

                // Check if refIdPath input parameter is ok
                if(refIdPath < 0 || refIdPath >= static_cast<int>(nextPossiblePaths.size())){
                    std::cerr<<"User Error : Desired path idx must be included into [0.."<<nextPossiblePaths.size()-1<<"]"<<std::endl;
                    return -1;
                }

                // For each path create VP
                for (int idxPath = 0; idxPath < nextPossiblePaths.size(); ++idxPath){

                	std::vector<osmium::geom::Coordinates> sortedNodesPath;
                    std::vector<int> speedLimit;
                    osmClass.sortedNodesFromPath(obj.indexNodesInCurrentWay, obj.heading, obj.posMerc, nextPossiblePaths[idxPath], sortedNodesPath, speedLimit);
                    std::vector<std::string> nodesRules ;
                    osmClass.getTrafficRules(sortedNodesPath, nodesRules);

                    velocityProfile vp;

                    std::vector<std::pair<double,double>> path_vp;
                    std::vector<double> speedLimit_vp(speedLimit.begin(),speedLimit.end());
                    for(auto onePointInPath : sortedNodesPath) path_vp.push_back(std::make_pair(onePointInPath.x,onePointInPath.y));

                    vp.init(path_vp, speedLimit_vp, nodesRules);
                    vp.reSamplePath(reSamplePathMinDist);
                    vp.smoothHardSoftTurn(smoothHardTurnMinBound, smoothHardTurnMaxBound, smoothSoftTurnMinBound, smoothSoftTurnMaxBound);

                    // Save current control points
				    xControlPointsPaths.push_back(vp.get_xPath());
				    yControlPointsPaths.push_back(vp.get_yPath());

				    vp.setCurvatureProfile(nbSampleCurvProfile);
                    std::vector<double> curvatureOriginProfile = vp.get_curvatureProfile();
                    vp.smoothCurvatureProfile(amountOfSmoothing);

				    // Save Current Path
				    xPaths.push_back(vp.get_xPath());
				    yPaths.push_back(vp.get_yPath());

                    // Add current path into the map
                    if(seeAllPaths || (!seeAllPaths && idxPath==refIdPath)){
                        std::cout<<"Plot path (idxPath) : "<<idxPath<<std::endl;
                        addCurvePath(plsMap, osmClass, xControlPointsPaths.back(), yControlPointsPaths.back(), xPaths.back(), yPaths.back(), obj.id);
                    }

                    // Draw current curvature profile
                    if( (seeAllCurvatureProfiles && seeAllPaths) || (!seeAllCurvatureProfiles && idxPath==refIdPath)){
                        std::cout<<"Plot curvature profile for path (idxPath) : "<<idxPath<<std::endl;
                        drawCurvatureProfile(plsCurveProfile, curvatureOriginProfile, vp.get_distanceProfile(), vp.get_trafficRules(), 9);
                        addCurvatureProfile(plsCurveProfile, vp.get_curvatureProfile(), vp.get_distanceProfile(), 11+obj.id);
                    }

                    if(seeAllPaths || seeAllCurvatureProfiles)
                        std::getchar(); // WaitKey

                    vp.setVelocityProfiles(maxSpeedPercentP1, maxSpeedPercentP2, maxSpeedPercentP3);

                    if(trafficRulesConstraints)
                        vp.addTrafficRulesConstraints(stopDistance, maxSpeedPercentP1, maxSpeedPercentP2, maxSpeedPercentP3, gP1, gP2, gP3);
                    
                    if(curvatureConstraints)
                        vp.addLatAccelerationConstraints(aLatP1, aLatP2, aLatP3);
                    
                    if(accelerationConstraints)
                        vp.addAccelerationConstraints(aP1, aP2, aP3);
                    
                    if(decelerationConstraints)
                        vp.addDecelerationConstraints(gP1, gP2, gP3);
                    
                    vp.cleanVelocityProfiles(equalDiffPrecent);

                    // Save VP
                    veloProfiles.push_back(vp);

                    if(!ros::ok()) break;

                    // Draw current velocity profile
                    if( (seeAllVelocityProfiles && seeAllPaths) || (seeAllVelocityProfiles && !seeAllPaths && idxPath==refIdPath) ){
                        for(int vpIdx=0; vpIdx < static_cast<int>(vp.get_velocityProfiles().size()); vpIdx++){
                            std::cout<<"Plot velocity profile for path (idxPath) and case (idxCase) : "<<idxPath<<" "<<vpIdx<<std::endl;
                            drawVelocityProfile(plsVelocityProfile, vp.get_velocityProfiles()[vpIdx], vp.get_speedLimits(), vp.get_trafficRules(), vp.get_distanceProfile());
                            // WaitKey
                            std::getchar();
                            if(!ros::ok()) break;
                        }
                    }
                    else if(!seeAllVelocityProfiles && idxPath==refIdPath){
                        std::cout<<"Plot velocity profile for path (idxPath) and case (idxCase) : "<<idxPath<<" "<<refIdVeloP<<std::endl;
                        // Draw refIdVeloP velocity profile
                        drawVelocityProfile(plsVelocityProfile, vp.get_velocityProfiles()[refIdVeloP], vp.get_speedLimits(), vp.get_trafficRules(), vp.get_distanceProfile());
                    }
                }
                if(!ros::ok()) break;
                
                // WaitKey. End of Init part
                std::getchar();
                
                if(seeAllVelocityProfiles)
                    drawVelocityProfile(plsVelocityProfile, veloProfiles[refIdPath].get_velocityProfiles()[refIdVeloP], veloProfiles[refIdPath].get_speedLimits(), veloProfiles[refIdPath].get_trafficRules(), veloProfiles[refIdPath].get_distanceProfile());

                if(seeAllCurvatureProfiles)
                    drawCurvatureProfile(plsCurveProfile, veloProfiles[refIdPath].get_curvatureProfile(), veloProfiles[refIdPath].get_distanceProfile(), veloProfiles[refIdPath].get_trafficRules(), 11+obj.id);

                // Check if refIdVeloP input parameter is ok
                if(refIdVeloP < 0 || refIdVeloP >= static_cast<int>(veloProfiles[refIdPath].get_velocityProfiles().size())){
                    std::cerr<<"User Error : Desired velocity profile idx must be included into [0.."<<veloProfiles[refIdPath].get_velocityProfiles().size()-1<<"]"<<std::endl;
                    return -1;
                }

                // Update ref object pos
                posPrecForObject = std::make_pair(obj.posMerc.first, obj.posMerc.second);

                // Test Preceding Situation
                if(idmConstraints){
                    double distPrecedingSituation;
                    int idPrecedingSituation;
                    if(PrecedingObject(osmClass, objDict[times[idxTime]], obj, distPrecedingSituation, idPrecedingSituation)){
                        
                        std::cout<<"Preceding object : "<<idPrecedingSituation<<" -- Distance : "<<distPrecedingSituation<<" m";
                        std::cout<<" -- Velocity : "<< objectByTime[std::make_pair(times[idxTime],idPrecedingSituation)].velocity<<" m/s"<<std::endl;
                        std::cout<<"Ref Object : "<<obj.id<<" -- Velocity : "<<obj.velocity<<std::endl;

                        veloProfiles[refIdPath].addIdmPrecedingObjectConstraints(distPrecedingSituation, obj.velocity,
                            objectByTime[std::make_pair(times[idxTime],idPrecedingSituation)].velocity, distanceForObject,
                            aP1, aP2, aP3, aExpo, comfDeceleration, d0, timeGap);
                    }
                    else{
                        veloProfiles[refIdPath].addIdmAccelerationConstraints(obj.velocity, distanceForObject, aP1, aP2, aP3, aExpo);
                    }

                    preDistForObject = distanceForObject;
                }
    		}
    		else if(!firstTimeForObject){

    			distanceForObject += sqrt(pow(obj.posMerc.first-posPrecForObject.first,2)+pow(obj.posMerc.second-posPrecForObject.second,2));

    			updatePath(xPaths[refIdPath], yPaths[refIdPath], sqrt(pow(obj.posMerc.first-posPrecForObject.first,2)+pow(obj.posMerc.second-posPrecForObject.second,2)));
    			
    			// Add refIdPath path into the map
    			addCurvePath(plsMap, osmClass, xControlPointsPaths[refIdPath], yControlPointsPaths[refIdPath], xPaths[refIdPath], yPaths[refIdPath], obj.id);
                
                // Add point into VP plot
                addPointVelocityProfile(plsVelocityProfile, obj.velocity, distanceForObject, obj.id, 1);

                // Update ref object pos
                posPrecForObject = obj.posMerc;

                // Test Preceding Situation
                if(idmConstraints){
                    double distPrecedingSituation;
                    int idPrecedingSituation;
                    if(PrecedingObject(osmClass, objDict[times[idxTime]], obj, distPrecedingSituation, idPrecedingSituation)){
                        
                        std::cout<<"Preceding object : "<<idPrecedingSituation<<" -- Distance : "<<distPrecedingSituation<<" m";
                        std::cout<<" -- Velocity : "<< objectByTime[std::make_pair(times[idxTime],idPrecedingSituation)].velocity<<" m/s"<<std::endl;
                        std::cout<<"Ref Object : "<<obj.id<<" -- Velocity : "<<obj.velocity<<std::endl;

                        veloProfiles[refIdPath].addIdmPrecedingObjectConstraints(distPrecedingSituation, obj.velocity,
                            objectByTime[std::make_pair(times[idxTime],idPrecedingSituation)].velocity, distanceForObject,
                            aP1, aP2, aP3, aExpo, comfDeceleration, d0, timeGap);

                    }
                    else{
                        veloProfiles[refIdPath].addIdmAccelerationConstraints(obj.velocity, distanceForObject, aP1, aP2, aP3, aExpo);
                    }
                    addIdmUpdateVelocityProfile(plsVelocityProfile, veloProfiles[refIdPath], std::vector<double>{preDistForObject, distanceForObject}, refIdVeloP);
                    preDistForObject = distanceForObject;
                }

                // Evaluation of tracked driver

                // Permit to avoid "noise" during stop situation
                if(obj.velocity>=1){ 
                    double deltaVeloVp = 1000.0;
                    int idxFinal;
                    double veloVPtmp;

                    for (int idxVeloP = 0; idxVeloP < 3; ++idxVeloP)
                    {
                        double veloVPtmp;
                        veloProfiles[refIdPath].getVelocity(refIdVeloP, idxVeloP, distanceForObject, veloVPtmp);
                        if(fabs(veloVPtmp-obj.velocity) <= deltaVeloVp){
                            deltaVeloVp = fabs(veloVPtmp-obj.velocity);
                            idxFinal = idxVeloP;
                        }
                    }
                    driverBehaviorEval[idxFinal]++;
                }
            }
    	}
        std::cout<<"Progress : ";
        std::cout<< ( static_cast<double>(idxTime)/(times.size()-1) )*100.0 << " %" << std::endl;

        // WaitKey
        if(pauseBetweenEachTime) std::getchar();
        else std::cout<<std::endl;

        if(!ros::ok()) break;
    }

    // Print Eval result
    std::cout<<"======================================="<<std::endl;
    std::cout<<"Driver behavior evaluation"<<std::endl;
    std::cout<<"======================================="<<std::endl;

    std::cout<<"Defensive : ";
    std::cout<<100.0*(static_cast<double>(driverBehaviorEval[0])/(driverBehaviorEval[0]+driverBehaviorEval[1]+driverBehaviorEval[2]));
    std::cout<<" %"<<std::endl;
    std::cout<<"Normal : ";
    std::cout<<100.0*(static_cast<double>(driverBehaviorEval[1])/(driverBehaviorEval[0]+driverBehaviorEval[1]+driverBehaviorEval[2]));
    std::cout<<" %"<<std::endl;
    std::cout<<"Sporty : ";
    std::cout<<100.0*(static_cast<double>(driverBehaviorEval[2])/(driverBehaviorEval[0]+driverBehaviorEval[1]+driverBehaviorEval[2]));
    std::cout<<" %"<<std::endl;

    // WaitKey : Permit to see the result with the plot of the velocity profile
    std::getchar();

    return 0;
}