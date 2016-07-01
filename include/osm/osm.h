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
 * @file   osm.h
 * @author DUFOUR Julien
 * @date   May, 2016
 * @brief  File containing useful headers to manipulate and use the Open Street Map (OSM) data.
 *
 * These methods permit to obtain a lot of informations about surrounding roads. Informations as :
 * _ Type
 * _ Speed limit
 * _ Traffic rules
 * _ Geographical positionning
 * _ Number of lane
 * _ Direction of each lane
 * _ Dimension
 *
 * Also, thanks to a kdTree, it is possible to find the current position in term of road and lane of an object.
 * The kdTree methods are provided by nanoflann library.
 *
 * @see include/libosmium-2.6.1
 * @see http://docs.osmcode.org/libosmium/master/
 * @see https://github.com/osmcode/libosmium
 * @see include/nanoflann
 * @see https://github.com/jlblancoc/nanoflann
 */

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <map>
#include <math.h> //cos

// Mercator projection
#include <mercatorProjection/mercatorProjection.h>

// Libosmium
#include <osmium/osm.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/handler.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/visitor.hpp>
#include <osmium/index/map/sparse_mem_array.hpp>

// Nanoflann
#include <nanoflann/include/nanoflann.hpp>

#ifndef OSM_H
#define OSM_H

#define earth_circumference 40075.16 //in Km

using index_t = osmium::index::map::SparseMemArray<osmium::unsigned_object_id_type, osmium::Location>;
using cache_t = osmium::handler::NodeLocationsForWays<index_t>;

/**
 * @brief This struct represent oneWay.
 */
struct oneWay{
    std::vector<osmium::geom::Coordinates> myNodes;       /*!< List of nodes whiches represent one way.*/
    std::string placement;                                /*!< Placement of the line formed by the node in ground truth.*/
    int type;                                             /*!< 0 if the way is a Car way, 1 if it is a Bicycle/Pedestrian way, -1 if it isnt a way.*/
    bool oneway;                                          /*!< True for an one-way road. Else, False. */
    int lanes;                                            /*!< The number of lane in the way, -1 if there is a problem with tags or if it isnt a way.*/
    std::vector<std::vector<std::string>> lanesDirection; /*!< All possible directions for each lane on one way.*/
    double width;                                         /*!< The total width of the way in meters, -1 if it isnt a way.*/
    int maxspeed;                                         /*!< Speed limit of way (in Km/h). -1 if is not mentioned in OSM file */
};

/**
 *  @brief Class which regroup all usefull function to manipulate OpenStreetMap Data.
 */
class osm{

    /*-----------------------------------------------------------------------------
     *  Attributes
     *-----------------------------------------------------------------------------
     */

public:
    double _cutRay;                                                      /*!< Diameter of perception circle around the car (in Kilometers).*/
    osmium::Location _origin;                                            /*!< Longitude/Latitude of initial position of the car (in decimal degrees).*/
    double _scale;                                                       /*!< This scale permit to have a mercator projection in meters. For further informatiom, see : void way(osmium::Way& way)*/    
    std::map<long int,oneWay> _waysDict;                                 /*!< Dictionnary which permit to link information about one way with his id.*/
    std::map<std::pair<double,double>,std::vector<long int>> _nodesDict; /*!< Dictionnary which permit to link (x,y), in Mercator CS (in meters), of one node to one or more way thanks to their IDs.*/
    std::map<std::pair<double,double>,std::string> _trafficRules;        /*!< Dictionnary which permit to link (x,y), in Mercator CS (in meters), of one node to one specific traffic rule.*/

private:
    /**
     * @brief This struct represent the only way to scan OSM files.
     * MyWayHandler permit to catch way data in any OSM file. 
     */
    struct MyWayHandler : public osmium::handler::Handler {
        
        double _scale;                                                       /*!< This scale permit to have a mercator projection in meters.*/
        std::map<long int,oneWay> _waysDict;                                 /*!< Dictionnary which permit to link information about one way with his id*/
        std::map<std::pair<double,double>,std::vector<long int>> _nodesDict; /*!< Dictionnary which permit to link (x,y) in Mercator CS (in meters) of one node to one or more way thanks to their IDs*/
        
        /**
         *  @brief way catch information about one way.
         *  It is call for each way in OSM file.
         *  For more information see Libosmium documentation.
         */
        void way(osmium::Way& way);
    };

    /**
     * @brief This struct represent the only way to scan OSM files.
     * MyNodeHandler permit to catch node data in any OSM file.
     * Here, it is necessary to use an other struct because :
     * . I want to work with nodes after the ways : It is more efficient. I work only with the catched ways.
     * . In OSM files the nodes are after the ways and osmium library explore OSM file from the beginning to the end. 
     */
    struct MyNodeHandler : public osmium::handler::Handler {
        
        double _scale;                                                /*!< This scale permit to have a mercator projection in meters.*/
        std::vector<std::pair<double,double>> _nodes;                 /*!< Contains the list of node which are present in catched ways.*/
        std::map<std::pair<double,double>,std::string> _trafficRules; /*!< Dictionnary which permit to link (x,y), in Mercator CS (in meters), of one node to one specific traffic rule.*/

        /**
         *  @brief node catch information about one node.
         *  It is call for each node in OSM file.
         *  For more information see Libosmium documentation.
         */
        void node(osmium::Node& node);
    };

    /**
     * @brief This struct represent the only way to use kdtree from nanoflann lib.
     * PointCloud represent a structure of point (x,y) associate with some useful function to use kdtree from nanoflann.
     * For further information see : include/nanoflann/examples. 
     */
    struct PointCloud
    {
        struct Point
        {
            double  x,y;
        };

        std::vector<Point> pts;

        // Must return the number of data points
        inline size_t kdtree_get_point_count() const { return pts.size(); }

        // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
        inline double kdtree_distance(const double *p1, const size_t idx_p2,size_t /*size*/) const
        {
            const double d0=p1[0]-pts[idx_p2].x;
            const double d1=p1[1]-pts[idx_p2].y;
            return d0*d0+d1*d1;
        }

        // Returns the dim'th component of the idx'th point in the class:
        // Since this is inlined and the "dim" argument is typically an immediate value, the
        // "if/else's" are actually solved at compile time.
        inline double kdtree_get_pt(const size_t idx, int dim) const
        {
            if (dim==0) return pts[idx].x;
            else if (dim==1) return pts[idx].y;
            else return -1;
        }

        // Optional bounding-box computation: return false to default to a standard bbox computation loop.
        // Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
        // Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
        template <class BBOX>
        bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
    };

    nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud>, PointCloud, 2 /* dim */>* _kdtree; /*!< Permit to implement K nearest neighbor search from nodes list, here _nodesDict.*/
    osm::PointCloud _nodesCloud; /*!< Used by kdtree functions to return (x,y) cordinates of Neighbors.*/


    /*-----------------------------------------------------------------------------
     *  Methods
     *-----------------------------------------------------------------------------
     */

public:
    osm(){};

    /**
     *  @brief Constructor of osm class.
     *  @param originLon   Longitude of initial position (in decimal degrees)                            (input)
     *  @param originLat   Latitude of initial position (in decimal degrees)                             (input)
     *  @param cutRay      Represent the diameter of perception circle area around the car (in meters)   (input)
     */
    osm(double originLon, double originLat, double cutRay);

    /**
     *  @brief readOsmFile Read the OSM file with preset handler (MyWayHandler and MyNodeHandler).
     *                     This function catch important informations about the ways and store them into std::map.
     *
     *  @param pkg_path    Path of ROS package "velodyne_tracking"                                (input)
     */
    void readOsmFile(std::string pkg_path);

    /**
     *  @brief getMinMaxGPSPoint From GPS coordinates, compute 2 points to make a BBox around these coordinates
     *  @param origin            Center of the BBox in GPS Coordinates                             (input)
     *  @param min               Minimum GPS Coordinates for the BBox                              (output)
     *  @param max               Maximum GPS Coordinates for the BBox                              (output)
     *  @param around_distance   Represent the diameter of circle area around the car (in meters)  (output)
     */
    void getMinMaxGPSPoint(osmium::Location origin, osmium::Location &min, osmium::Location &max, double around_distance);

    /**
     *  @brief isOnMap  Determine if the Kitti car is on the osm given map.
     *  @param origin   Position of the car in GPS Coordinates.            (input)
     *  @param header   Contains all metadatas of the osm file.            (input)
     *  @return         True if the car is on the osm map. If not return False.
     */
    bool isOnMap(osmium::Location origin, osmium::io::Header header);

    /**
     *  @brief cutOsmMap      Split osm map from BBox
     *  @param pkg_path       Path of velodyne_tracking package.                                              (input)
     *  @param osm_file_name  Name of the source OSM Map (with extension).                                    (input)
     *  @param min            Minimum GPS Coordinates for the BBox. Assume that they are into the input Map.  (input)
     *  @param max            Maximum GPS Coordinates for the BBox. Assume that they are into the input Map.  (input)
     *  @return               True if the Map had been splitted. If not return False.
     */
    bool cutOsmMap(std::string pkg_path, std::string osm_file_name, osmium::Location min, osmium::Location max);

    /**
     *  @brief widthLandR  Determine the width of each side of one way according to direction.
     *  @param wayId       Id of the way.                                   (input)
     *  @param wL          Width (in meters) of the left side of the way.   (output)
     *  @param wR          Width (in meters) of the right side of the way.  (output)
     *  @return            False if any problem occurs.
     */
     bool widthLandR(long int wayId, double& wL, double& wR);

    /**
     *  @brief setKdtree  Set input of our Kdtree from the nodes list _nodesDict.
     *  @return           False if any problem occurs.
     */
    bool setKdtree();

    /**
     *  @brief kdtreeNearestSearch          Determine a list of k node coordinates whiches contained around (x,y).
     *  @param x                            X coordinate of search point in Mercator CS (in meters).                                                  (input)
     *  @param y                            Y coordinate of search point in Mercator CS (in meters).                                                  (input)
     *  @param pointNearestSquaredDistance  Distance between searched point and finded point in square meter for each points into pointsRadiusSearch. (output)
     *  @param pointsNearestSearch          Coordinates of finded points in Mercator CS.                                                              (output)
     *  @param k                            Number of neighbor.                                                                                       (input)
     *  @return                             False if any problem occurs.
     */  
    bool kdtreeNearestSearch(double x, double y, std::vector<double> &pointNearestSquaredDistance, std::vector<std::pair<double,double>> &pointsNearestSearch, int k);

    /**
     *  @brief calculBoxCorners  Calculate the coordinates of four corners of the Bbox represented by the line (ptNode1,ptNode2) and the caracteristics of the way.
     *  @param ptNode1           Coordinates of the first node which represent the road in Mercator CS (in meters).                           (input)
     *  @param ptNode2           Coordinates of the second node which represent the road in Mercator CS (in meters).                          (input)  
     *  @param wayId             Id of the way.                                                                                               (input)
     *  @param corners           Coordinates of the four points whiches represent the box in Mercator CS (in meters). Clockwise from ptNode1. (output)
     *  @return                  False if any problem occurs.
     */
    bool calculBoxCorners(osmium::geom::Coordinates ptNode1, osmium::geom::Coordinates ptNode2, long int wayId, std::vector<osmium::geom::Coordinates> &corners);

    /**
     *  @brief objectIsOnRoad   Determine if an object is on one or more roads whiches represented by a list of nearest nodes.
     *  @param nearestNodes     Coordinates of the nearest nodes whiches represent the roads in Mercator CS (in meters).                                   (input)
     *  @param objectPosition   Coordinates of the object in Mercator CS (in meters).                                                                      (input)
     *  @param presentInWays    Contains collection of pair <way id, lane number> where object is present.                                                 (output)
     *  @param betweenNodes     Contains collection of pair <idx, idx>. It represents the two node index (in oneWay.myNodes) between which the object is.  (output)
     *  @return                 True if the object position is on one road represented by one or more node.
     */ 
    bool objectIsOnRoad(std::vector<std::pair<double,double>> nearestNodes, osmium::geom::Coordinates objectPosition, std::vector<std::pair<long int,int>> &presentInWays, std::vector<std::pair<int,int>> &betweenNodes);

    /**
     *  @brief wayHeadingVsObjectHeading   Find the smallest difference between road and object heading to find the most likely way.
     *  @param presentInWays               Contains collection of pair <way id, lane number> where object is present.                                                 (input)
     *  @param betweenNodes                Contains collection of pair <idx, idx>. It represents the two node index (in oneWay.myNodes) between which the object is.  (input)
     *  @param objectHeading               Heading of the object in Mercator CS (in degree).                                                                             (input)
     *  @param mostLikelyWay               Pair <way id, lane number> where object is most likely present.                                                            (output)
     *  @return                            True if no problem occurs.
     */ 
    bool wayHeadingVsObjectHeading(const std::vector<std::pair<long int,int>> &presentInWays, const std::vector<std::pair<int,int>> &betweenNodes, double objectHeading, std::pair<long int,int> &mostLikelyWay);

    /**
     *  @brief nextPossiblePaths  From one way define the next possible paths (list of Way) according to the direction of the object.
     *  @param fromWayId          Id which represents the way of reference (beginning way).                                                            (input)
     *  @param betweenNodes       Pair <idx, idx>. It represents the two node index (in oneWay.myNodes from "fromWayId") between which the object is.  (input)
     *  @param objectHeading      Heading of the object in Mercator CS (in degree).                                                                    (input)
     *  @param objectPos          Position of the object in Mercator CS (in meters).                                                                   (input)
     *  @param nextPossiblePaths  List of possible path, formed by a list of WayId, accessible from the way "fromWayId".                               (output)
     *  @param minimalPathSize    Minimal desired path size (in meters) for all founded list of way.                                                   (input)
     *  @return                   True if all possible path (>minimalPathSize) are found. Else or if any problem occurs, false.
     */
    bool nextPossiblePaths(long int fromWayId, std::pair<int,int> betweenNodes, double objectHeading, std::pair<double,double> objectPos, std::vector<std::vector<long int>> &nextPossiblePaths, double minimalPathSize);

    /**
     *  @brief sortedNodesFromPath  From one list of way (one path) return a list of nodes according to the direction of the object.
     *  @param betweenNodes         Pair <idx, idx>. It represents the two node index (in oneWay.myNodes from "path[0]") between which the object is.  (input)
     *  @param objectHeading        Heading of the object in Mercator CS (in degree).                                                                  (input)
     *  @param objectPos            Position of the object in Mercator CS (in meters).                                                                 (input)
     *  @param path                 List of way id which represents one accessible path from the object position.                                      (input)
     *  @param sortedNodesPath      List of sorted nodes according to the direction of the car.                                                        (output)
     *  @param speedLimit           Speed limit at each output node.                                                                                   (output)
     *  @return                     False if any problem occurs. Else, true.
     */
    bool sortedNodesFromPath(std::pair<int,int> betweenNodes, double objectHeading, std::pair<double,double> objectPos, std::vector<long int> path, std::vector<osmium::geom::Coordinates> &sortedNodesPath, std::vector<int> &speedLimit);

    /**
     *  @brief getTrafficRules  From a list of nodes, return one traffic rule for each node.
     *  @param nodes            List of nodes.                                              (input)
     *  @param trafficRules     Traffic rule for each input node.                           (output)
     *  @return                 False if any problem occurs. Else, true.
     */
    bool getTrafficRules(std::vector<osmium::geom::Coordinates> nodes, std::vector<std::string> &trafficRules);

private:
    /**
     *  @brief typeOfRoad  Define the type of one way.
     *  @param wayTags     Element which contains all tags of one way.             (input)
     *  @return            0 if the way is a Car way, 1 if it is a Bicycle/Pedestrian way, -1 if it isnt a way.
     */
    static int typeOfWay(const osmium::TagList& wayTags);

    /**
     *  @brief directionOfWay  Define the direction of one way.
     *  @param wayTags         Element which contains all tags of one way.         (input)
     *  @return                True if the way is an one-direction road, False if is not.
     */
    static bool directionOfWay(const osmium::TagList& wayTags);

    /**
     *  @brief lanesOfWay  Define the number of lane in one way.
     *  @param wayTags     Element which contains all tags of one way.             (input)
     *  @return            The number of lane in one way, -1 if there is a problem with tags.
     */
    static int lanesOfWay(const osmium::TagList& wayTags);

    /**
     *  @brief directionOfLanes  Define all possible directions of each lane in one way.
     *  @param wayTags           Element which contains all tags of one way.       (input)
     *  @param nbLanes           Number of lane in one way.                        (input)
     *  @return                  The number of lane in one way, -1 if there is a problem with tags.
     */
    static std::vector<std::vector<std::string>> directionOfLanes(const osmium::TagList& wayTags, int nbLanes);

    /**
     *  @brief widthOfRoad  Define the total width of one way.
     *  @param wayTags      Element which contains all tags of one way.        (input)
     *  @return             The total width of one way in meters, -1 if it isnt a way.
     */
    static double widthOfRoad(const osmium::TagList& wayTags);

    /**
     *  @brief distPointToLine  Define the perpendicular distance from a point to a line.
     *  @param p1               Fist point which represents the line in Mercator CS (in meters).   (input)
     *  @param p2               Second point which represents the line in Mercator CS (in meters). (input)
     *  @param pObject          Coordinates of the object in Mercator CS (in meters).              (input)
     *  @return                 Shortest distance from pObject to a line (P1P2) in Euclidean geometry (in meters). 
     */
    static double distPointToLine(osmium::geom::Coordinates p1, osmium::geom::Coordinates p2, osmium::geom::Coordinates pObject);

    /**
     *  @brief CoordOrthProjPointToLine  Define the coordinates of an orthogonal projection from a point to a line.
     *                                   This function permit to have the relative position of an object on the line represented by nodes in OSM data.
     *  @param p1                        Fist point which represents the line in Mercator CS (in meters).   (input)
     *  @param p2                        Second point which represents the line in Mercator CS (in meters). (input)
     *  @param pObject                   Coordinates of the object in Mercator CS (in meters).              (input)
     *  @return                          Coordinates of the orthogonal projection of pObject in p1p2 in Mercator CS (in meters). 
     */
    static osmium::geom::Coordinates CoordOrthProjPointToLine(osmium::geom::Coordinates p1, osmium::geom::Coordinates p2, osmium::geom::Coordinates pObject);

    /**
     *  @brief objectIsOnBox    Determine if an object represented by his position is on the box represented by 4 corners.
     *                          Point Inclusion in Polygon Test by W. Randolph Franklin (WRF).
     *  @param corners          Coordinates of the four points which represent the box in Mercator CS (in meters).  (input)
     *  @param objectPosition   Coordinates of the object in Mercator CS (in meters).                               (input)
     *  @return                 True if the object position is on the box represented by the corners.
     */
    bool objectIsOnBox(std::vector<osmium::geom::Coordinates> corners, osmium::geom::Coordinates objectPosition);

    /**
     *  @brief nextPossibleWays  From one way define the next possible ways according to the direction on the object or referenceNode.
     *                           To use this function, assume that _waysDict and _nodesDict were initialized (osm::readOsmFile).
     *  @param fromWayId         Id which represents the way of reference.                                                                                        (input)
     *  @param betweenNodes      Pair <idx, idx>. It represents the two node index (in oneWay.myNodes) between which the object is.                               (input)
     *  @param objectHeading     Heading of the car in Mercator CS (in degree).                                                                                   (input)
     *  @param referenceNode     Represents the ending node of the current way defined by fromWayId. For the first pass, assume that it initialized as <nan,nan>. (input/output)
     *  @return                  List of way IDs which represent the next possible accessible ways. 
     */
    std::vector<long int> nextPossibleWays(long int fromWayId, std::pair<int,int> betweenNodes, double objectHeading, std::pair<double,double> &referenceNode);

};

#endif