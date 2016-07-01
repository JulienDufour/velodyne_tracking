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
 * @file   driverBehavior_utils.h
 * @author DUFOUR Julien
 * @date   June, 2016
 * @brief  File containing all useful methods used by driverBehavior main program.
 *
 * This file is organized in two main parts. The first part represents the headers and the second part represents the methods.
 * Each part is composed by two sub-parts. "Process" for compulsory methods and "Plot" for the methods only used during plot.
 */

// Plot
#include <plstream.h>
#include <plplot.h>

// Dataio
#include <utils/dataio.h>

// OSM
#include <osm/osm.h>

// Velo profile
#include <velocityProfile/velocityProfile.h>


/********************************************************************************/
/**********************    Struct & Methods declaration    **********************/
/**************************       (Process)            **************************/
/********************************************************************************/

/**
 * @brief This struct represents all useful accesible informations for one object at one time.
 */
struct oneObject{
    int id;                                    /*!< Id (unique) of the object.*/
    std::pair<double,double> posMerc;          /*!< Object position in Mercator CS ((x,y) in meters).*/
    std::vector<double> posGlob;               /*!< Object position in Global CS ((x,y,z) in meters).*/
    double width;                              /*!< Width of the object (in meters).*/
    double length;                             /*!< Length of the object (in meters).*/
    double height;                             /*!< Height of the object (in meters).*/
    double velocity;                           /*!< Velocity of the object (in m/s²).*/
    double heading;                            /*!< Heading of the object (in degree).*/
    long int currentWay;                       /*!< Id of the way where the objecti is.*/
    int laneInCurrentWay;                      /*!< Number of the lane in current way where the object is.*/
    std::pair<int,int> indexNodesInCurrentWay; /*!< Pair <idx, idx>. It represents the two node index (in currentWay.myNodes) between which the object is.*/
};

/**
 *  @brief getTrMatrixFromFirstPose   From first GPS information (t=0) compute the matrix which permit to link global CS and mercator CS.
 *  @param oxtsPose                   Contains all info about GPS at one time (assume that it is t=0). These info are read from txt file into oxts/data in our dataset.     (input)
 *  @param tfMat                      Transformation matrix which link global CS and Mercator CS.                                                                           (output)
 *  @return                           False if any problem occurs.
 */
bool getTrMatrixFromFirstPose(const std::vector<double> oxtsPose, Eigen::Matrix4d &tfMat);

/**
 *  @brief sortCsvDataByTimestamp   From csv file reading, sort and re-organize data by timestamp.
 *  @param gpsPoses                 All info about GPS at each time. These info are read from txt file into oxts/data in our dataset.               (input)
 *  @param times                    All timestamps (in ns).                                                                                         (input)
 *  @param myTracks                 Informations about tracked object readed into cvs file. First dim means one object, second dim means one time.  (input)
 *  @param transformMP              Transformation matrix which link global CS and Mercator CS.                                                     (input)
 *  @param osmClass                 osm current object, permit to use OSM data.                                                                     (input)
 *  @param objectsListByTime        Dictionary which link a list of objects with time.                                                              (output)
 *  @param objectByTime             Dictionary which link one object with time and his id.                                                          (output)
 *  @return                         False if any problem occurs.
 */
bool sortCsvDataByTimestamp(std::vector<std::vector<double>> gpsPoses, 
                            std::vector<unsigned long> times,
                            std::vector<std::vector<utils::ObjectPath>> myTracks,
                            Eigen::Matrix4d transformMP, osm osmClass,
                            std::map<unsigned long,std::vector<oneObject>> &objectsListByTime,
                            std::map<std::pair<unsigned long, int>, oneObject> &objectByTime);

/**
 *  @brief CoordOrthProjPointToLine  Define the coordinates of an orthogonal projection from a point to a line.
 *                                   This function permit to have the relative position of an object on the line represented by nodes in OSM data.
 *  @param p1                        Fist point which represents the line in Mercator CS (in meters).   (input)
 *  @param p2                        Second point which represents the line in Mercator CS (in meters). (input)
 *  @param pObject                   Coordinates of the object in Mercator CS (in meters).              (input)
 *  @return                          Coordinates of the orthogonal projection of pObject in p1p2 in Mercator CS (in meters). 
 */
osmium::geom::Coordinates CoordOrthProjPointToLine(osmium::geom::Coordinates p1, osmium::geom::Coordinates p2, osmium::geom::Coordinates pObject);

/**
 *  @brief isPrecedingObject    From a current object, determine if an other object is a precedent object or not.
 *  @param osmClass             osm current object, permit to use OSM data.                                                                                  (input)
 *  @param sortedNodesPath      List of sorted nodes according to the direction of the current object.                                                       (input)
 *  @param betweenNodes         Pair <idx, idx>. It represents the two node index between which the possible precedent object is.                            (input)
 *  @param wayId                For the current object (index 0) and for the possible precedent object (index 1), contains the id of their way.              (input)
 *  @param laneInWay            For the current object (index 0) and for the possible precedent object (index 1), contains the lane number in their way.     (input)
 *  @param posObjects           For the current object (index 0) and for the possible precedent object (index 1), contains their position (in Mercator CS).  (input)
 *  @param distBetweenObjects   Distance, according to the way characteristics, between the two objects (in meters).                                         (output)
 *  @return                     True if the tested object is a precedent object. Else, false.
 */
bool isPrecedingObject(osm osmClass, std::vector<osmium::geom::Coordinates> sortedPath,
    std::pair<int,int> betweenNodes, std::vector<long int> wayId, std::vector<int> laneInWay,
    std::vector<osmium::geom::Coordinates> posObjects, double &distBetweenObjects);

/**
 *  @brief PrecedingObject   From a reference object at one time, determine if it have a precedent object or not.
 *  @param osmClass          osm current object, permit to use OSM data.                                                                                       (input)
 *  @param objects           List of objects which are present at the same time that the object of reference.                                                  (input)
 *  @param objectRef         Object of reference (looking for his possible precedent object).                                                                  (input)
 *  @param distance          Distance, according to the way characteristics, between the reference object and his precedent object if it exists. (in meters).  (output)
 *  @param id                Id of finded precedent object of the reference object.                                                                            (output)
 *  @return                  True if the reference object have a precedent object. Else, false.
 */
bool PrecedingObject(osm osmClass, std::vector<oneObject> objects, oneObject objectRef, double &distance, int &id);


/********************************************************************************/
/**********************    Struct & Methods declaration    **********************/
/**************************         (Plot)             **************************/
/********************************************************************************/

/**
 * @brief This struct represents a RGB color.
 */
struct oneColor{
    int r; /*!< Red component (0..255).*/
    int g; /*!< Green component (0..255).*/
    int b; /*!< Blue component (0..255).*/
};

/**
 *  @brief getRandColorForEachObject   Create a random color for each object following their id (0 to totalNumberOfObject-1).
 *  @param numberOfObject              Number of seen object in the current dataset.   (input)
 *  @return                            Dictionary which links id of object and color.
 */
std::map<int,oneColor> getRandColorForEachObject(int numberOfObject);

/**
 *  @brief initPlPlot        Init PlPlot object and set his color table.
 *  @param pls               PlPlot object.                                                                                                 (input)
 *  @param colorObjDict      Dictionary which links id of object and color.                                                                 (input)
 *  @param defaultPlotDriver If true, xwin driver will be use. Else, if it false, a list of different option will be write in the terminal. (input)
 */
void initPlPlot(plstream *pls, std::map<int,oneColor> colorObjDict, bool defaultPlotDriver);

/**
 *  @brief drawMap      Clear and draw OSM into PlPlot object. (0,0) is obtained from origin position of osm object.
 *  @param pls          PlPlot object.                                                                                            (input)
 *  @param gpsPose      Current GPS pose (lat, long in decimal degree). Define the center of the current plot.                    (input)
 *  @param osmClass     osm current object, permit to use OSM data.                                                               (input)
 *  @param timestamp    Timestamp of current kitti position (gpsPose).                                                            (input)
 *  @param wayId        Set as True, add way id into the plotted map.                                                             (input)
 *  @param deltaGraph   Plotted Ray around kitti car (in meters). Default value is 80.0 meters. (Velodyne HDL-64E Range : 120m)   (input)
 */
void drawMap(plstream *pls, std::vector<double> gpsPose, osm osmClass, std::string timestamp, bool wayId, double deltaGraph = 80.0);

/**
 *  @brief addCurvePath      Permit to add a path (curve with his control points) into the map.
 *  @param pls               PlPlot object.                                                          (input)
 *  @param osmClass          osm current object, permit to use OSM data.                             (input)
 *  @param xControlPoints    Coordinates X of control points (meters, Mercator CS).                  (input)
 *  @param yControlPoints    Coordinates Y of control points (meters, Mercator CS).                  (input)
 *  @param xCurve            Coordinates X of the path (meters, Mercator CS).                        (input)
 *  @param yCurve            Coordinates Y of the path (meters, Mercator CS).                        (input)
 *  @param id                Id of concerned object (for the purpose of using correspondant color).  (input)
 */
void addCurvePath(plstream *pls, osm osmClass, std::vector<double> xControlPoints, std::vector<double> yControlPoints,
                std::vector<double> xCurve, std::vector<double> yCurve, int id);

/**
 *  @brief drawCurvatureProfile   Clear and draw curvature profile into a PlPlot object.
 *  @param pls                    PlPlot object.                                                       (input)
 *  @param curvatureProfile       Y axis. Curvature at each sample (1/m).                              (input)
 *  @param times                  X axis. Contains the distance from origin for each sample (meters).  (input)
 *  @param trafficRules           Traffic rules at each sample.                                        (input)
 *  @param color                  Permit to choose printed color. See initPlPlot.                      (input)
 */
void drawCurvatureProfile(plstream *pls, std::vector<double> curvatureProfile, std::vector<double> times, std::vector<std::string> trafficRules, int color);

/**
 *  @brief addCurvatureProfile   Permit to add an additional curvature profile into an existing curvature plot (PlPlot object).
 *  @param pls                   PlPlot object.                                                       (input)
 *  @param curvatureProfile      Y axis. Curvature at each sample (1/m).                              (input)
 *  @param times                 X axis. Contains the distance from origin for each sample (meters).  (input)
 *  @param color                 Permit to choose printed color. See initPlPlot.                      (input)
 */
void addCurvatureProfile(plstream *pls, std::vector<double> curvatureProfile, std::vector<double> times, int color);

/**
 *  @brief drawVelocityProfile   Clear and draw velocity profiles (defensive, normal, sporty) into a PlPlot object.
 *  @param pls                   PlPlot object.                                                       (input)
 *  @param velocityProfiles      Y axis. velocities at each sample for each profile (m/s).            (input)
 *  @param speedLimit            Law Speed limit at each sample.                                      (input)
 *  @param nodesRules            Traffic rules at each sample.                                        (input)
 *  @param times                 X axis. Contains the distance from origin for each sample (meters).  (input)
 */
void drawVelocityProfile(plstream *pls, std::vector<std::vector<double>> velocityProfiles, std::vector<double> speedLimit, std::vector<std::string> nodesRules, std::vector<double> times);

/**
 *  @brief addIdmUpdateVelocityProfile   Add IDM modification into an existing velocity profiles plot (PlPlot object).
 *  @param pls                           PlPlot object.                                                                                                                   (input)
 *  @param vp                            velocityProfile object. Contains all possible velocity profiles (defensive, normal, sporty) in all possibles case for one path.  (input)
 *  @param times                         X axis. Contains the distance from origin for each sample (meters).                                                              (input)
 *  @param desiredCase                   Desired case into all possible case for the velocityProfile object.                                                              (input)
 */
void addIdmUpdateVelocityProfile(plstream *pls, velocityProfile vp, std::vector<double> times, int desiredCase);

/**
 *  @brief addPointVelocityProfile  Permit to add a velocty for one object at one time (distqnce) into an existing velocity profiles plot (PlPlot object).
 *  @param pls                      PlPlot object.                                                                 (input)
 *  @param velocity                 Y axis. velocity of the object (m/s).                                          (input)
 *  @param distance                 X axis. Distance from origin to the object in the velocity profiles (meters).  (input)
 *  @param id                       Id of concerned object (for the purpose of using correspondant color).         (input)
 *  @param type                     Represents the type of plotted point. See PlPlot documentation.                (input)
 */
void addPointVelocityProfile(plstream *pls, double velocity, double distance, int id, int type);

/**
 *  @brief addText  Permit to add some text into an existing plot (PlPlot object).
 *  @param pls      PlPlot object.  (input)
 *  @param posY     Y axis.         (input)
 *  @param posX     X axis.         (input)
 *  @param tex      Text.           (input)
 */
void addText(plstream *pls, double posX, double posY, std::string tex);

/**
 *  @brief addText    Permit to add a tracked object, as a point, into the existing map plot (PlPlot object).
 *  @param pls        PlPlot object.                                                   (input)
 *  @param obj        Object.                                                          (input)
 *  @param osmClass   osm object.                                                      (input)
 *  @param type       Represents the type of plotted point. See PlPlot documentation.  (input)
 */
void addObject(plstream *pls, oneObject obj, osm osmClass, int type);

void updatePath(std::vector<double> &x_pts, std::vector<double> &y_pts, double distance);


/********************************************************************************/
/******************************                    ******************************/
/**************************                            **************************/
/**********************          Methods (Process)         **********************/
/**************************                            **************************/
/******************************                    ******************************/
/********************************************************************************/

bool getTrMatrixFromFirstPose(const std::vector<double> oxtsPose, Eigen::Matrix4d &tfMat){
    
	if(oxtsPose.empty()){
		std::cerr<<"getTrMatrixFromFirstPose cannot be used. Input <oxtsPose> is empty."<<std::endl;
        return false;
	}

    // Permit to exprime Mercator CS in meters (ECCENT from mercatorProjection.h)
    double scale = sqrt( 1 - pow(ECCENT,2) * pow(sin(mp::deg_rad(oxtsPose[0])),2)) / cos(mp::deg_rad(oxtsPose[0]));
    
    // Create the transformation matrix.
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();

    // Fill in translation part.
    double mx = mp::merc_x(oxtsPose[1]);
    double my = mp::merc_y(oxtsPose[0]);
    transform.translation() << mx/scale, my/scale, oxtsPose[2];

    // Add rotation part.
    transform.rotate(Eigen::AngleAxisd(oxtsPose[5], Eigen::Vector3d::UnitZ()));
    transform.rotate(Eigen::AngleAxisd(oxtsPose[4], Eigen::Vector3d::UnitY()));
    transform.rotate(Eigen::AngleAxisd(oxtsPose[3], Eigen::Vector3d::UnitX()));

    tfMat = transform.matrix();

    return true;
}

bool sortCsvDataByTimestamp(std::vector<std::vector<double>> gpsPoses, 
                            std::vector<unsigned long> times,
                            std::vector<std::vector<utils::ObjectPath>> myTracks,
                            Eigen::Matrix4d transformMP,
                            osm osmClass,
                            std::map<unsigned long,std::vector<oneObject>> &objectsListByTime,
                            std::map<std::pair<unsigned long, int>, oneObject> &objectByTime){

    objectsListByTime.clear();
    objectByTime.clear();

    // For each timestamp from Velodyne
    for (int idxTime = 0; idxTime < times.size(); ++idxTime){

        // For each tracked object
        for (int i = 0; i < myTracks.size(); ++i)
        {
            // For each time during the track of object i
            for(utils::ObjectPath obj : myTracks[i]){
                // If timestamp object == timestamps from velo -> Add to dict
                if(obj.time == times[idxTime]){

                    // Create new object
                    oneObject myObject;
                    
                    // Update id
                    myObject.id = i;

                    // Update position in Mercator CS
                    Eigen::MatrixXd position;
                    position.resize(4,1);
                    
                    position(0,0) = obj.position[0];
                    position(1,0) = obj.position[1];
                    position(2,0) = obj.position[2];
                    position(3,0) = 1.0;

                    position = transformMP * position;

                    myObject.posMerc = std::make_pair(position(0,0), position(1,0));

                    // Update position in Global CS (from first GPS pose)
                    myObject.posGlob = std::vector<double>{obj.position[0],obj.position[1],obj.position[2]};

                    // Update speed (in meters per second)
                    myObject.velocity = sqrt(pow(obj.position[3],2) + pow(obj.position[4],2) + pow(obj.position[5],2));

                    // Update size (in meters)
                    myObject.height = obj.z;

                    if(obj.x > obj.y){
                        myObject.length = obj.x;
                        myObject.width = obj.y;
                    }
                    else{
                        myObject.width = obj.x;
                        myObject.length = obj.y;
                    }

                    // Update heading (in degree) from velocity vector
                    Eigen::MatrixXd veloVector;
                    veloVector.resize(4,1);
                    veloVector(0,0) = obj.position[3];
                    veloVector(1,0) = obj.position[4];
                    veloVector(2,0) = obj.position[5];
                    veloVector(3,0) = 0.0;
                    veloVector = transformMP * veloVector;
                    myObject.heading = mp::rad_deg( atan2(veloVector(1,0),veloVector(0,0)) );

                    // Update informations about current way
                    // Use osm and kdtreeNearestSearch to define the road of the object
                    std::vector<double> pointRadiusSquaredDistance;
                    std::vector<std::pair<double,double>> pointsNearestSearch;
                    std::vector<std::pair<long int, int>> presentInWays;
                    std::vector<std::pair<int,int>> betweenNodes;

                    if(osmClass.kdtreeNearestSearch(myObject.posMerc.first, myObject.posMerc.second,pointRadiusSquaredDistance, pointsNearestSearch, 10)){
                        if(osmClass.objectIsOnRoad(pointsNearestSearch, osmium::geom::Coordinates(myObject.posMerc.first, myObject.posMerc.second), presentInWays, betweenNodes)){
                            
                            // Check if the heading of the road is according to the heading of the object (in Mercator CS)
                            std::pair<long int, int> presentInWay;
                            osmClass.wayHeadingVsObjectHeading(presentInWays, betweenNodes, myObject.heading, presentInWay);
                            // Then add the current way (most likely way following heading in presentInWays) as a posible way for the current object
                            myObject.currentWay = presentInWay.first;
                            myObject.laneInCurrentWay = presentInWay.second;

                            int pos = find(presentInWays.begin(), presentInWays.end(), presentInWay) - presentInWays.begin();
                            myObject.indexNodesInCurrentWay = betweenNodes[pos];
                        }
                        else{
                            myObject.currentWay = -1;
                            myObject.laneInCurrentWay = -1;
                            myObject.indexNodesInCurrentWay = std::make_pair(-1,-1);
                        }
                    } 

                    objectByTime[std::make_pair(times[idxTime],myObject.id)] = myObject;

                    // times[idxTime] is an entry of objectsListByTime ?
                    std::map<unsigned long,std::vector<oneObject>>::iterator it_dict = objectsListByTime.find(times[idxTime]);
                    if(it_dict != objectsListByTime.end()){
                        // This entry exists in the map. Add object.
                        it_dict->second.push_back(myObject);
                    }
                    else{
                        // Create vector of object and add this timestamp to the map.
                        objectsListByTime[times[idxTime]] = std::vector<oneObject>{myObject};
                    }
                }
            }
        }

        // Add Kitti Car from gps Data
        oneObject kittiObject;

        // ID
        kittiObject.id = static_cast<int>(myTracks.size());

        // Get current GPS Pose in Mercator CS
        double lon = gpsPoses[idxTime][1];
        double lat = gpsPoses[idxTime][0];

        // Pos in Mercator CS
        kittiObject.posMerc = std::make_pair(mp::merc_x(lon)/osmClass._scale, mp::merc_y(lat)/osmClass._scale);

        // Position in Global CS (from first GPS pose)
        kittiObject.posGlob = std::vector<double>{0.0,0.0,0.0};

        // Speed (in meters per second)
        kittiObject.velocity = sqrt(pow(gpsPoses[idxTime][6],2) + pow(gpsPoses[idxTime][7],2));

        // Size (in meters)
        kittiObject.height = 1.65;
        kittiObject.length = 2.71;
        kittiObject.width = 1.60;

        // Heading
        kittiObject.heading = mp::rad_deg(gpsPoses[idxTime][5]);

         // oneWay
        // Use osm and kdtreeNearestSearch to determine the road of the object
        std::vector<double> pointRadiusSquaredDistance;
        std::vector<std::pair<double,double>> pointsNearestSearch;
        std::vector<std::pair<long int, int>> presentInWays;
        std::vector<std::pair<int,int>> betweenNodes;

        if(osmClass.kdtreeNearestSearch(kittiObject.posMerc.first, kittiObject.posMerc.second,pointRadiusSquaredDistance, pointsNearestSearch, 10)){
            if(osmClass.objectIsOnRoad(pointsNearestSearch, osmium::geom::Coordinates(kittiObject.posMerc.first, kittiObject.posMerc.second), presentInWays, betweenNodes)){
                
                // Check if the heading of the road is according to the heading of the car (in Mercator CS)
                std::pair<long int, int> presentInWay;
                osmClass.wayHeadingVsObjectHeading(presentInWays, betweenNodes, kittiObject.heading, presentInWay);
                // Then add the current way as a posible way for the current object
                kittiObject.currentWay = presentInWay.first;
                kittiObject.laneInCurrentWay = presentInWay.second;

                int pos = find(presentInWays.begin(), presentInWays.end(), presentInWay) - presentInWays.begin();
                kittiObject.indexNodesInCurrentWay = betweenNodes[pos];
            }
            else{
                kittiObject.currentWay = -1;
                kittiObject.laneInCurrentWay = -1;
                kittiObject.indexNodesInCurrentWay = std::make_pair(-1,-1);
            }
        } 

        objectByTime[std::make_pair(times[idxTime],kittiObject.id)] = kittiObject;

        // times[idxTime] is an entry of objectsListByTime ?
        std::map<unsigned long,std::vector<oneObject>>::iterator it_dict = objectsListByTime.find(times[idxTime]);
        if(it_dict != objectsListByTime.end()){
            // This entry exists in the map. Add object.
            it_dict->second.push_back(kittiObject);
        }
        else{
            // Create vector of object and add this object at this timestamp to the map.
            objectsListByTime[times[idxTime]] = std::vector<oneObject>{kittiObject};
        }
    }

    return true;
}

osmium::geom::Coordinates CoordOrthProjPointToLine(osmium::geom::Coordinates p1, osmium::geom::Coordinates p2, osmium::geom::Coordinates pObject){

    // Take care about particular case : p1p2 is a vertical line
    if(p2.x == p1.x){
        return osmium::geom::Coordinates(p1.x,pObject.y);
    }
    // Take care about particular case : p1p2 is a horizontal line
    else if(p2.y == p1.y){
        return osmium::geom::Coordinates(pObject.x,p1.y);
    }
    else{
        // Set "a" and "b" in y=ax+b for the line p1p2 (D1)
        double a = (p2.y - p1.y)/(p2.x - p1.x);
        double b = (p2.y*p1.x - p1.y*p2.x)/(p1.x - p2.x);

        // Set "c" in y = (-1/a)*x + c for the perpendicular line of p1p2 which pass by pObject (D2)
        double c = pObject.y + (1.0/a)*pObject.x;

        // Looking for intersection between D1 and D2
        double x = ( (c-b)*a )/ (a*a + 1);
        double y = a*x+b;
        return osmium::geom::Coordinates(x,y);
    }
}

bool isPrecedingObject(osm osmClass, std::vector<osmium::geom::Coordinates> sortedPath,
    std::pair<int,int> betweenNodes, std::vector<long int> wayId, std::vector<int> laneInWay,
    std::vector<osmium::geom::Coordinates> posObjects, double &distBetweenObjects){

    // First test : The two objects are in the same lane in the same way.
    if(wayId[0] == wayId[1] && laneInWay[0] == laneInWay[1]){

        // Look if the reference nodes, "betweenNodes", for the possible preceding object exist in the path provides by current object
        std::vector<osmium::geom::Coordinates>::iterator it0 = std::find(sortedPath.begin(), sortedPath.end(), osmClass._waysDict[wayId[1]].myNodes[betweenNodes.first]);
        std::vector<osmium::geom::Coordinates>::iterator it1 = std::find(sortedPath.begin(), sortedPath.end(), osmClass._waysDict[wayId[1]].myNodes[betweenNodes.second]);

        if(it0 != sortedPath.end() || it1 != sortedPath.end()){

            // Create a path between two object
            std::vector<osmium::geom::Coordinates> pathBetweenObject(sortedPath.begin(), sortedPath.begin() + std::min(it0-sortedPath.begin(), it1-sortedPath.begin()));

            // Add object coordinates on the path (projection), it represents the end of the path, "pathBetweenObject".
            pathBetweenObject.push_back(CoordOrthProjPointToLine(osmClass._waysDict[wayId[1]].myNodes[betweenNodes.first], osmClass._waysDict[wayId[1]].myNodes[betweenNodes.second], posObjects[1]));

            // Update distBetweenObjects.
            distBetweenObjects = 0.0;
            for (int i = 0; i < static_cast<int>(pathBetweenObject.size())-1; ++i)
            {
                distBetweenObjects += sqrt( pow(pathBetweenObject[i].x - pathBetweenObject[i+1].x,2) + pow(pathBetweenObject[i].y - pathBetweenObject[i+1].y,2) );
            }

            if(pathBetweenObject.size() == 2){
                // Check if the possible preceding object is in front of current object.

                double deltaY = pathBetweenObject[1].y-pathBetweenObject[0].y;
                double deltaX = pathBetweenObject[1].x-pathBetweenObject[0].x;
                double squareDist = sqrt(pow(deltaX,2)+pow(deltaY,2));
                double angleDiffObjectsPath = atan2(deltaY/squareDist, deltaX/squareDist);
                
                deltaY = sortedPath[1].y-sortedPath[0].y;
                deltaX = sortedPath[1].x-sortedPath[0].x;
                squareDist = sqrt(pow(deltaX,2)+pow(deltaY,2));
                angleDiffObjectsPath -= atan2(deltaY/squareDist, deltaX/squareDist);
                
                angleDiffObjectsPath *= (180.0 / M_PI);

                if(angleDiffObjectsPath > -2.0 && angleDiffObjectsPath < 2.0) return true;
            }
            else return true;
        }
    }
    return false;
}

bool PrecedingObject(osm osmClass, std::vector<oneObject> objects, oneObject objectRef, double &distance, int &id){

    bool precedingSituation = false;

    if(objectRef.currentWay != -1){
        distance = 999999999.9;
        double distanceTMP;

        std::vector<osmium::geom::Coordinates> sortedNodesPathRef;
        std::vector<int> speedLimit;
        osmClass.sortedNodesFromPath(objectRef.indexNodesInCurrentWay, objectRef.heading, objectRef.posMerc, std::vector<long int>{objectRef.currentWay}, sortedNodesPathRef, speedLimit);
                        
        for (int idxObject = 0; idxObject < objects.size(); ++idxObject)
        {
            if(objects[idxObject].currentWay != -1){
                if(objects[idxObject].id != objectRef.id){
                    if( isPrecedingObject(osmClass, sortedNodesPathRef, objects[idxObject].indexNodesInCurrentWay,
                        std::vector<long int>{objectRef.currentWay, objects[idxObject].currentWay},
                        std::vector<int>{objectRef.laneInCurrentWay, objects[idxObject].laneInCurrentWay},
                        std::vector<osmium::geom::Coordinates>{osmium::geom::Coordinates(objectRef.posMerc.first,objectRef.posMerc.second),osmium::geom::Coordinates(objects[idxObject].posMerc.first,objects[idxObject].posMerc.second)},
                        distanceTMP)
                        ){

                        if(distanceTMP<distance){
                            distance = distanceTMP;
                            id = objects[idxObject].id;
                            if(!precedingSituation) precedingSituation = true;
                        }

                    }
                }
            }
        }
    }
    return precedingSituation;
}


/********************************************************************************/
/******************************                    ******************************/
/**************************                            **************************/
/**********************           Methods (Plot)           **********************/
/**************************                            **************************/
/******************************                    ******************************/
/********************************************************************************/

std::map<int,oneColor> getRandColorForEachObject(int numberOfObject){

    /* initialize random seed: */
    srand (time(NULL));

    std::map<int,oneColor> colorDict;

    for (int i = 0; i < numberOfObject; ++i)
    {
        oneColor color_tmp;
        color_tmp.r = static_cast<int> (rand() % 256); // rand in the range 0 to 255
        color_tmp.g = static_cast<int> (rand() % 256); // rand in the range 0 to 255
        color_tmp.b = static_cast<int> (rand() % 256); // rand in the range 0 to 255
        colorDict[i] = color_tmp;
    }
    return colorDict;
}

void initPlPlot(plstream *pls, std::map<int,oneColor> colorObjDict, bool defaultPlotDriver){
    
    // Set default driver
    if(defaultPlotDriver) pls->sdev(std::string("xwin").c_str());

    // start plplot object
    pls->init();

    // inhib pause
    pls->spause(false);

    // Upgrade the default (16) max authorized color
    pls->scmap0n(11+colorObjDict.size());

    // Set collection of used colors
    pls->scol0a( 1, 150, 0, 0, 1);     //red for car roads
    pls->scol0a( 2, 180, 180, 255, 1); //pale blue for Pedes/Bicycle ways
    pls->scol0a( 3, 60, 230, 60, 1);   // first pose
    pls->scol0a( 9, 255, 255, 255, 1); // White : Road nodes line
    pls->scol0a( 10, 0, 0, 0, 1);      //black

    for(std::map<int,oneColor>::iterator it_color = colorObjDict.begin(); it_color!=colorObjDict.end(); ++it_color){
        pls->scol0a(11 + it_color->first, it_color->second.r, it_color->second.g, it_color->second.b, 0.5);
    }
}

void drawMap(plstream *pls, std::vector<double> gpsPose, osm osmClass, std::string timestamp, bool wayId, double deltaGraph){

    // Clear precedent plot
    pls->scolbg(0,0,0);
    pls->clear();
    pls->col0(1);

    // Pen size. 0 means minimmum
    pls->width(0);

    // Create PlPlot point vector
    PLFLT x[10000];
    PLFLT y[10000];
    PLFLT xL[10000];
    PLFLT yL[10000];
    PLFLT xR[10000];
    PLFLT yR[10000];

    // Set 0
    osmium::geom::Coordinates coordOrigin = osmium::geom::Coordinates(mp::merc_x(osmClass._origin.lon())/osmClass._scale,mp::merc_y(osmClass._origin.lat())/osmClass._scale);

    // Set current Pose (kitti car) as center
    double lon = gpsPose[1];
    double lat = gpsPose[0];
    osmium::geom::Coordinates coordPose = osmium::geom::Coordinates(mp::merc_x(lon)/osmClass._scale,mp::merc_y(lat)/osmClass._scale);

    // Reset Grid
    PLINT just=0, axis=0;
    double deltaX = coordPose.x - coordOrigin.x;
    double deltaY = coordPose.y - coordOrigin.y;

    PLFLT xmin =deltaX-deltaGraph, ymin=deltaY-deltaGraph, xmax=deltaX+deltaGraph, ymax=deltaY+deltaGraph;
    pls->env(xmin, xmax, ymin, ymax, just, axis );

    // Set title and axis legend
    pls->lab( "GPS/IMU Global CS X (meters)", "GPS/IMU Global CS Y (meters)", ("Kitti Env Road Map -- " + timestamp).c_str());

    // Plot the map from osm
    std::vector<osmium::geom::Coordinates> corners;

    // Add first pose
    x[0]=coordOrigin.x-coordOrigin.x;
    y[0]=coordOrigin.y-coordOrigin.y;
    pls->col0( 3 );
    pls->poin( 1, x, y, 2 );

    // Add current pose in Global CS
    x[0]=coordPose.x - coordOrigin.x;
    y[0]=coordPose.y - coordOrigin.y;

    pls->poin(1, x, y, 1);

    // Add Map
    for(std::map<long int,oneWay>::iterator it_way = osmClass._waysDict.begin(); it_way!=osmClass._waysDict.end(); ++it_way) {

        if(it_way->second.type!=-1){

            int j=0;
            for(int i=0 ; i < static_cast<int>(it_way->second.myNodes.size()) - 1 ; i++){
                x[i]=it_way->second.myNodes[i].x-coordOrigin.x;
                y[i]=it_way->second.myNodes[i].y-coordOrigin.y;
                x[i+1]=it_way->second.myNodes[i+1].x-coordOrigin.x;
                y[i+1]=it_way->second.myNodes[i+1].y-coordOrigin.y;

                corners.clear();
                osmClass.calculBoxCorners(it_way->second.myNodes[i], it_way->second.myNodes[i+1], it_way->first, corners);
                
                xL[i] = corners[0].x-coordOrigin.x;
                yL[i] = corners[0].y-coordOrigin.y;
                xL[i+1] = corners[1].x-coordOrigin.x;
                yL[i+1] = corners[1].y-coordOrigin.y;

                xR[i] = corners[3].x-coordOrigin.x;
                yR[i] = corners[3].y-coordOrigin.y;
                xR[i+1] = corners[2].x-coordOrigin.x;
                yR[i+1] = corners[2].y-coordOrigin.y;
                j++;
            }

            PLINT nbPoint=j+1;
            //Add central line in White
            pls->col0( 9 );
            pls->line(nbPoint, x, y);

            //Add borders
            if( it_way->second.type == 0 ) pls->col0( 1 );
            else if( it_way->second.type == 1 ) pls->col0( 2 );
            else pls->col0( 10 );

            pls->line(nbPoint, xL, yL);
            pls->line(nbPoint, xR, yR);

            //Add way id
            if(wayId){
                pls->schr(0.0,0.3);
                pls->ptex(x[static_cast<int>(ceil(j/2.0))], y[static_cast<int>(ceil(j/2.0))], 0, 0, 0.5, std::to_string(it_way->first).c_str()); 
            }
        }
    }
}

void addCurvePath(plstream *pls, osm osmClass, std::vector<double> xControlPoints, std::vector<double> yControlPoints,
                std::vector<double> xCurve, std::vector<double> yCurve, int id){
    
    // Set 0
    osmium::geom::Coordinates coordOrigin = osmium::geom::Coordinates(mp::merc_x(osmClass._origin.lon())/osmClass._scale,mp::merc_y(osmClass._origin.lat())/osmClass._scale);

    // Create PlPlot point vector.
    PLFLT x[static_cast<int>(xCurve.size())];
    PLFLT y[static_cast<int>(xCurve.size())];

    PLINT nbPoint;
    pls->col0(11+id);

    // Add control point
    int i;
    for (i = 0; i < xControlPoints.size(); ++i)
    {
        x[i]=xControlPoints[i]-coordOrigin.x;
        y[i]=yControlPoints[i]-coordOrigin.y;
    }
    nbPoint=i;
    pls->poin(nbPoint, x, y, 4);

    // Add curve
    for (i = 0; i < xCurve.size(); ++i)
    {
        x[i]=xCurve[i]-coordOrigin.x;
        y[i]=yCurve[i]-coordOrigin.y;
    }
    nbPoint=i;
    pls->line(nbPoint, x, y);
}

void drawCurvatureProfile(plstream *pls, std::vector<double> curvatureProfile, std::vector<double> times, std::vector<std::string> trafficRules, int color){

    // Clear precedent plot
    pls->scolbg(0,0,0);
    pls->clear();
    pls->col0(1);

    // Create PlPlot point vector.
    PLFLT distAxis[static_cast<int>(curvatureProfile.size())];
    PLFLT curvAxis[static_cast<int>(curvatureProfile.size())];
    PLFLT trafficRuleDistAxis[2];
    PLFLT trafficRuleCurvAxis[2];

    // Reset Grid
    PLINT just=0, axis=0;
    double maxValue = *std::max_element(curvatureProfile.begin(),curvatureProfile.end());
    double minValue = *std::min_element(curvatureProfile.begin(),curvatureProfile.end());

    PLFLT xmin =0.0, ymin=minValue, xmax=times.back(), ymax=maxValue;
    pls->env(xmin, xmax, ymin, ymax, just, axis );

    // Set title and axis legend
    pls->lab( "Distance (m)", "Curvature (1/m)", "Curvature Profile");

    // Add Curvature
    int i;
    for (i = 0; i < curvatureProfile.size(); ++i)
    {
        // Add Curvature
        distAxis[i] = times[i];
        curvAxis[i] = curvatureProfile[i];
        
        if(trafficRules[i].compare("no") != 0){

            // Add special node
            trafficRuleDistAxis[0] = times[i];
            trafficRuleDistAxis[1] = times[i];
            trafficRuleCurvAxis[0] = ymin;
            trafficRuleCurvAxis[1] = ymax;

            pls->col0(2);
            pls->line(2, trafficRuleDistAxis, trafficRuleCurvAxis);
        }
    }
    PLINT nbPoint=i;
    pls->col0(color);
    pls->line(nbPoint, distAxis, curvAxis);
}

void addCurvatureProfile(plstream *pls, std::vector<double> curvatureProfile, std::vector<double> times, int color){

    // Create PlPlot point vector.
    PLFLT distAxis[static_cast<int>(curvatureProfile.size())];
    PLFLT curvAxis[static_cast<int>(curvatureProfile.size())];

    // Add Curvature
    int i;
    for (i = 0; i < curvatureProfile.size(); ++i)
    {
        // Add Curvature
        distAxis[i] = times[i];
        curvAxis[i] = curvatureProfile[i];
    }
    PLINT nbPoint=i;
    pls->col0(color);
    pls->line(nbPoint, distAxis, curvAxis);
}

void drawVelocityProfile(plstream *pls, std::vector<std::vector<double>> velocityProfiles, std::vector<double> speedLimit, std::vector<std::string> nodesRules, std::vector<double> times){

    // Clear precedent plot
    pls->scolbg(0,0,0);
    pls->clear();
    pls->col0(1);

    // Create PlPlot point vector.
    PLFLT distAxis[static_cast<int>(velocityProfiles[0].size())];
    PLFLT veloP1Axis[static_cast<int>(velocityProfiles[0].size())];
    PLFLT veloP2Axis[static_cast<int>(velocityProfiles[1].size())];
    PLFLT veloP3Axis[static_cast<int>(velocityProfiles[2].size())];

    PLFLT trafficRuleDistAxis[2];
    PLFLT trafficRuleVeloAxis[2];

    // Reset Grid
    PLINT just=0, axis=0;
    double maxValue = *std::max_element(speedLimit.begin(),speedLimit.end());
    //maxValue /= 3.6; //Km/h -> m/s

    PLFLT xmin =0.0, ymin=-1.5, xmax=times.back(), ymax=maxValue+2.5;
    pls->env(xmin, xmax, ymin, ymax, just, axis );

    // Set title and axis legend
    pls->lab( "Distance (m)", "Velocity (m/s)", "Velocity Profile");


    // Add law max Velocity
    int i;
    for (i = 0; i < speedLimit.size(); ++i)
    {

        distAxis[i] = times[i];
        veloP1Axis[i] = speedLimit[i];

        if(nodesRules[i].compare("no") != 0){

            // Add special node
            trafficRuleDistAxis[0] = times[i];
            trafficRuleDistAxis[1] = times[i];
            trafficRuleVeloAxis[0] = ymin;
            trafficRuleVeloAxis[1] = ymax;

            pls->col0(2);
            pls->line(2, trafficRuleDistAxis, trafficRuleVeloAxis);
        }

    }
    PLINT nbPoint=i;
    pls->col0(9);
    pls->line(nbPoint, distAxis, veloP1Axis);

    for (i = 0; i < velocityProfiles[0].size(); ++i)
    {
        distAxis[i] = times[i];
        veloP1Axis[i] = velocityProfiles[0][i];
        veloP2Axis[i] = velocityProfiles[1][i];
        veloP3Axis[i] = velocityProfiles[2][i];
    }

    nbPoint = i;
    pls->col0(2);
    pls->line(nbPoint, distAxis, veloP1Axis);
    pls->col0(3);
    pls->line(nbPoint, distAxis, veloP2Axis);
    pls->col0(1);
    pls->line(nbPoint, distAxis, veloP3Axis);

}

void addIdmUpdateVelocityProfile(plstream *pls, velocityProfile vp, std::vector<double> times, int desiredCase){

    // Create PlPlot point vector.
    PLFLT distAxis[static_cast<int>(times.size())];
    PLFLT veloP1Axis[static_cast<int>(times.size())];
    PLFLT veloP2Axis[static_cast<int>(times.size())];
    PLFLT veloP3Axis[static_cast<int>(times.size())];

    int i = 0;

    for (i = 0; i < times.size(); ++i)
    {

        distAxis[i] = times[i];

        vp.getVelocity(desiredCase, 0, times[i], veloP1Axis[i]);
        vp.getVelocity(desiredCase, 1, times[i], veloP2Axis[i]);
        vp.getVelocity(desiredCase, 2, times[i], veloP3Axis[i]);

    }

    PLINT nbPoint = i;
    pls->col0(2);
    pls->line(nbPoint, distAxis, veloP1Axis);
    pls->col0(3);
    pls->line(nbPoint, distAxis, veloP2Axis);
    pls->col0(1);
    pls->line(nbPoint, distAxis, veloP3Axis);

}

void addPointVelocityProfile(plstream *pls, double velocity, double distance, int id, int type){
    
    // Create PlPlot point vector.
    PLFLT x[1];
    PLFLT y[1];

    pls->col0(11+id);

    x[0]= distance;
    y[0]= velocity;

    pls->poin(1, x, y, type);
}

void addText(plstream *pls, double posX, double posY, std::string tex){
    //pls->width(0); 
    pls->schr(0.0,0.3);
    pls->ptex(posX,posY,0,0,0.5,tex.c_str());
}

void addObject(plstream *pls, oneObject obj, osm osmClass, int type){
    
    // Create PlPlot point vector.
    PLFLT x[1];
    PLFLT y[1];

    osmium::geom::Coordinates coordOrigin = osmium::geom::Coordinates(mp::merc_x(osmClass._origin.lon())/osmClass._scale,mp::merc_y(osmClass._origin.lat())/osmClass._scale);

    pls->col0(11+obj.id);

    x[0]=obj.posMerc.first - coordOrigin.x;
    y[0]=obj.posMerc.second - coordOrigin.y;

    pls->poin(1, x, y, type);

    addText(pls, x[0], y[0], std::to_string(obj.id));
}

void updatePath(std::vector<double> &x_pts, std::vector<double> &y_pts, double distance){
    int i = 0;
    double distanceOnPath = 0.0;
    
    while (distance > distanceOnPath && i < x_pts.size()-1){
        distanceOnPath += sqrt( pow(x_pts[i+1]-x_pts[i],2) + pow(y_pts[i+1]-y_pts[i],2) );
        i++;
    }

    double dX = x_pts[i] - x_pts[i+1];
    double dY = y_pts[i] - y_pts[i+1];
    double dxdy = sqrt(pow(dX,2)+pow(dY,2));
    dX = dX/dxdy;
    dY = dY/dxdy;

    std::vector<double> x_tmp = std::vector<double>{x_pts[i] + dX*fabs(distanceOnPath-distance)};
    std::vector<double> y_tmp = std::vector<double>{y_pts[i] + dY*fabs(distanceOnPath-distance)};   

    for (int j = i; j < x_pts.size(); ++j)
    {
        x_tmp.push_back(x_pts[j]);
        y_tmp.push_back(y_pts[j]);
    }
    x_pts = x_tmp;
    y_pts = y_tmp;
}