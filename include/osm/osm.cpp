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
 * @file   osm.cpp
 * @author DUFOUR Julien
 * @date   May, 2016
 * @brief  File containing useful methods to manipulate and use the Open Street Map (OSM) data.
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

#include <osm/osm.h>

osm::osm(double originLon, double originLat, double cutRay){
	
	_cutRay = cutRay;
	_origin = osmium::Location(originLon,originLat);
    _scale = sqrt( 1 - pow(ECCENT,2) * pow(sin(mp::deg_rad(originLat)),2)) / cos(mp::deg_rad(originLat));

}

void osm::MyWayHandler::way(osmium::Way& way){

	oneWay way_t;
    std::map<std::pair<double,double>,std::vector<long int>>::iterator it_node;

    way_t.type = osm::typeOfWay(way.tags());
    
    if(way_t.type!=-1){
        
        way_t.oneway = osm::directionOfWay(way.tags());
        way_t.width = osm::widthOfRoad(way.tags());
        way_t.lanes = osm::lanesOfWay(way.tags());
        way_t.lanesDirection = osm::directionOfLanes(way.tags(), way_t.lanes);
        way_t.placement = std::string(way.tags().get_value_by_key("placement","center"));
        way_t.maxspeed = atoi(way.tags().get_value_by_key("maxspeed","-1"));
    
        for (const osmium::NodeRef& nr : way.nodes()) {
            osmium::geom::Coordinates coord = osmium::geom::Coordinates(mp::merc_x(nr.location().lon())/_scale,mp::merc_y(nr.location().lat())/_scale);

            // Add the node into vector in oneWay.
            way_t.myNodes.push_back(coord);

            // Update _nodesDict
            it_node = _nodesDict.find(std::make_pair(coord.x,coord.y));
            if(it_node != _nodesDict.end()){
                // This node exists in the map. Add Way ID.
                it_node->second.push_back(way.id());
            }
            else{
                // Create vector of way_id and add this node to the map.
                _nodesDict[std::make_pair(coord.x,coord.y)] = std::vector<long int>{way.id()};
            }
        }
        // Update _waysDict
        _waysDict[way.id()] = way_t;
    }
    else{
    	way_t.width = -1;
    	way_t.lanes = -1;
    }    
}

void osm::MyNodeHandler::node(osmium::Node& node){

    // Get node coordinates in Mercator CS
    osmium::geom::Coordinates coord = osmium::geom::Coordinates(mp::merc_x(node.location().lon())/_scale,mp::merc_y(node.location().lat())/_scale);
    std::pair<double,double> nodeCoord = std::make_pair(coord.x,coord.y);
    // If this node exists in research list, look their tags.
    if(std::find(_nodes.begin(), _nodes.end(), nodeCoord) != _nodes.end()){

        const char* highway = node.get_value_by_key("highway","NA");

        if(
        std::string(highway).compare("give_way")==0
        || std::string(highway).compare("traffic_signals")==0
        || std::string(highway).compare("stop")==0
        || std::string(highway).compare("crossing")==0
        ){
            // Add it into output dictionary
            _trafficRules[nodeCoord] = std::string(highway);
        }
    }
}

void osm::readOsmFile(std::string pkg_path){

    index_t index;
    cache_t cache{index};

	// Set osm data location
    std::string osm_data_path = pkg_path + "/datasets/osm/data";

    //------------------------------------------------
    // First pass : Ways.
    //------------------------------------------------
    // Create osmium Reader & way handler object
    osmium::io::Reader waysReader((osm_data_path + "/osm_bbox.osm").c_str());
    osm::MyWayHandler wayHandler;

    // Set variables used by the handler
    wayHandler._scale = _scale;

    // Read file
    osmium::apply(waysReader, cache, wayHandler);

    // Get data from handler
    _waysDict = wayHandler._waysDict;
    _nodesDict = wayHandler._nodesDict;

    waysReader.close();

    //------------------------------------------------
    // Second pass : Nodes <=> Get traffic rules.
    //------------------------------------------------
    // Create Reader (just for nodes) and set node handler object
    osmium::io::Reader nodesReader((osm_data_path + "/osm_bbox.osm").c_str(), osmium::osm_entity_bits::node);
    osm::MyNodeHandler nodeHandler;

    // Set variables used by the handler
    nodeHandler._scale = _scale;
    for(auto const& element : _nodesDict) nodeHandler._nodes.push_back(element.first);

    // Read file
    osmium::apply(nodesReader, nodeHandler);

    // Get data from handler
    _trafficRules = nodeHandler._trafficRules;

    nodesReader.close();
}

void osm::getMinMaxGPSPoint(osmium::Location origin, osmium::Location &min, osmium::Location &max, double around_distance){

    // Compute delta value on latitude and longitude
    double d_lat = (360.0 / earth_circumference) * (around_distance/2.0);
    double d_lon = (360.0 / (cos(mp::deg_rad(origin.lat()))*earth_circumference)) * (around_distance/2.0);

    // Set the min coordinates
    min.set_lon(origin.lon() - d_lon);
    min.set_lat(origin.lat() - d_lat);

    // Set the max coordinates
    max.set_lon(origin.lon() + d_lon);
    max.set_lat(origin.lat() + d_lat);
}

bool osm::isOnMap(osmium::Location origin, osmium::io::Header header){
    bool result = false;

    for (auto& bbox : header.boxes()){
        if(bbox.contains(origin)){
            result = true;
        }
    }
    return result;
}

bool osm::cutOsmMap(std::string pkg_path, std::string osm_file_name, osmium::Location min, osmium::Location max){

    // Set osm-history-splitter-master path from pkg_path
    std::string osm_splitter_path = pkg_path + "/include/osm/osm-history-splitter-master";

    // Set osm data location
    std::string osm_data_path = pkg_path + "/datasets/osm/data";

    // Delete existing cutted osm file
    // Because if already exist there is an error with osm-history-splitter-master
    // static_cast<void> <=> Suppressing return value warnings 
    static_cast<void>(std::system(("rm " + osm_data_path + "/osm_bbox.osm").c_str()));

    // Create .config file for osm-history-splitter-master
    std::ofstream myConfigFile;
    myConfigFile.open(osm_splitter_path + "/bbox_cut.config", std::ofstream::out | std::ofstream::trunc);

    // Set precision in myConfigFile
    myConfigFile << std::setprecision(std::numeric_limits<double>::digits10 +1);

    // Write data
    // format : save_path_file OPTION BBox_cordinates_min_max
    myConfigFile << (osm_data_path + "/osm_bbox.osm" + " BBOX ");
    myConfigFile <<  min.lon() << "," << min.lat() << "," << max.lon() << "," << max.lat();

    // Close file
    myConfigFile.close();

    // Launch osm-history-splitter-master to cut the map
    // Usage: [OPTIONS] OSMFILE CONFIGFILE
    // Set executable name
    std::string osm_filter_command = osm_splitter_path + "/build/osm-history-splitter";
    // Set OSMFILE
    osm_filter_command += " " + osm_data_path + "/" + osm_file_name;
    // Set CONFIGFILE
    osm_filter_command += " " + osm_splitter_path + "/bbox_cut.config";
    // Launch command
    if(std::system(osm_filter_command.c_str()) !=0) return false;

    return true;
}

int osm::typeOfWay(const osmium::TagList& wayTags){

    const char* highway = wayTags.get_value_by_key("highway","NA");

    // Take care about special case, highway tag equals to "construction".
    // This case means that the type of the road is contained on "construction" tag.
    if(std::string(highway).compare("construction")==0) highway = wayTags.get_value_by_key("construction","NA");

    if(
        std::string(highway).compare("motorway")==0
        || std::string(highway).compare("motorway_link")==0
        || std::string(highway).compare("trunk")==0
        || std::string(highway).compare("trunk_link")==0
        || std::string(highway).compare("primary")==0
        || std::string(highway).compare("primary_link")==0
        || std::string(highway).compare("secondary")==0
        || std::string(highway).compare("secondary_link")==0
        || std::string(highway).compare("tertiary")==0
        || std::string(highway).compare("tertiary_link")==0
        || std::string(highway).compare("unclassified")==0
        || std::string(highway).compare("residential")==0
        || std::string(highway).compare("service")==0
        ) return 0; // Car way
    else if ( std::string(highway).compare("NA")!=0) return 1; // Bicycle and pedestrian way
    else return -1; // Not a way
}

bool osm::directionOfWay(const osmium::TagList& wayTags){

    // If the tag "oneway" is not contained in the tags, the default case is no.
    const char* oneway = wayTags.get_value_by_key("oneway","no");

    if(std::string(oneway).compare("yes")==0) return true;
    else return false;
}

int osm::lanesOfWay(const osmium::TagList& wayTags){

    const char* lanes_tag = wayTags.get_value_by_key("lanes","NA");

    // If the tag lanes already exist, return the value in int.
    // Else return a default value according to the type of road.
    if(std::string(lanes_tag).compare("NA")!=0){
        return atoi(lanes_tag);
    }
    else{
        const char* highway = wayTags.get_value_by_key("highway","NA");
        const char* oneway = wayTags.get_value_by_key("oneway","");

        if(
            std::string(highway).compare("motorway")==0
            || std::string(highway).compare("trunk")==0
            )
        {
            if(std::string(oneway).compare("yes")==0) return 2;
            else return -1; // Error : These should usually be mapped as two separate ways
        }
        else if(
            std::string(highway).compare("primary")==0
            || std::string(highway).compare("secondary")==0
            || std::string(highway).compare("tertiary")==0
            || std::string(highway).compare("residential")==0
            )
        {
            if(std::string(oneway).compare("yes")==0) return 1;
            else return 2;
        } 
        else if(std::string(highway).compare("NA")!=0) return 1;
        else return -1;
    }

}

std::vector<std::vector<std::string>> osm::directionOfLanes(const osmium::TagList& wayTags, int nbLanes){

    std::vector<std::vector<std::string>> dirOfLanes; // Return

    const char* turn_lanes_tag = wayTags.get_value_by_key("turn:lanes","NA");
    // Example of result : left|through|through;right
    // "|" represents a lane delimiter and ";" represents a possible direction delimiter

    // If the tag turn:lanes does not exist return a default value ("through") according to the number of lanes.
    if(std::string(turn_lanes_tag).compare("NA")==0){
        for (int i = 0; i < nbLanes; ++i)
        {
            dirOfLanes.push_back(std::vector<std::string>{"through"});
        }
    }
    else{
        
        // For each lane delimited by "|"
        std::istringstream iss(turn_lanes_tag);
        std::string sub_str;
        while(std::getline(iss, sub_str, '|')){
            
            // Take care about particulary case
            // The tag "none" means that is not classified way. Generally represents "through"
            // ";" in a sub-string means that there are more than one possible direction for the lane

            if(sub_str.find("none") == std::string::npos){

                if(sub_str.find(";") != std::string::npos){

                    // Create an ISS to go through the sub-string and useful variables
                    std::istringstream iss_sub_str(sub_str);
                    std::string sub_sub_string;
                    std::vector<std::string> dirOfLane;

                    // Go through the sub_str and update output vector
                    while(std::getline(iss_sub_str, sub_sub_string, ';')){
                        dirOfLane.push_back(sub_sub_string);
                    }
                    dirOfLanes.push_back(dirOfLane);
                }
                else dirOfLanes.push_back(std::vector<std::string>{sub_str});
            }
            else dirOfLanes.push_back(std::vector<std::string>{"through"});
        }
    }
    return dirOfLanes;
}

double osm::widthOfRoad(const osmium::TagList& wayTags){

    const char* width_tag = wayTags.get_value_by_key("width","NA");

    // If the tag width already exist, return the value in double.
    // Else return a default value according to the type of road.
    if(std::string(width_tag).compare("NA")!=0){
        return atof(width_tag);
    }
    else{
        const char* highway = wayTags.get_value_by_key("highway","NA");
        int lanes = osm::lanesOfWay(wayTags);
        if(lanes == -1) return -1;

        if(
            std::string(highway).compare("motorway")==0
            || std::string(highway).compare("motorway_link")==0
            || std::string(highway).compare("trunk")==0
            || std::string(highway).compare("trunk_link")==0
            ) return 3.75*lanes;

        else if(
            std::string(highway).compare("primary")==0
            || std::string(highway).compare("primary_link")==0
            ) return 3.5*lanes;

        else if(
            std::string(highway).compare("secondary")==0
            || std::string(highway).compare("secondary_link")==0
            ) return 3.25*lanes;

        else if(
            std::string(highway).compare("tertiary")==0
            || std::string(highway).compare("tertiary_link")==0
            ) return 3.0*lanes;

        else if(
            std::string(highway).compare("unclassified")==0
            || std::string(highway).compare("residential")==0
            || std::string(highway).compare("service")==0
            ) return 2.75*lanes;

        else if(
            std::string(highway).compare("NA")!=0
            ) return 1.8*lanes;
        else return -1;
    }
}

bool osm::widthLandR(long int wayId, double& wL, double& wR){

    if(_waysDict.empty()){
        std::cerr<<"Ways are not initialized -- Please run readOsmFile"<<std::endl;
        return false;
    }

    if(_waysDict[wayId].placement.compare("center")==0){
        wL = _waysDict[wayId].width/2.0;
        wR = _waysDict[wayId].width/2.0;
        return true;
    }
    else{
        std::string globalPos = _waysDict[wayId].placement.substr(0,_waysDict[wayId].placement.find(":"));
        int inLaneN = std::stoi(_waysDict[wayId].placement.substr(_waysDict[wayId].placement.find(":")+std::string(":").length(),_waysDict[wayId].placement.length()));

        if(globalPos.compare("right_of")==0){
            
            // Particular case : gps line _waysDict[wayId] is on the right way border.
            if(inLaneN==_waysDict[wayId].lanes){
                wL=_waysDict[wayId].width;
                wR=0.0;
            }
            else{
                wL = (inLaneN/static_cast<double>(_waysDict[wayId].lanes)) * _waysDict[wayId].width;
                wR = ((_waysDict[wayId].lanes-inLaneN)/static_cast<double>(_waysDict[wayId].lanes)) * _waysDict[wayId].width;
            }
            return true;
        }
        else if(globalPos.compare("left_of")==0){

            // Particular case : gps line _waysDict[wayId] is on the left way border.
            if(inLaneN==1){
                wL=0.0;
                wR=_waysDict[wayId].width;
            }
            else{
                wR = (inLaneN/static_cast<double>(_waysDict[wayId].lanes)) * _waysDict[wayId].width;
                wL = ((_waysDict[wayId].lanes-inLaneN)/static_cast<double>(_waysDict[wayId].lanes)) * _waysDict[wayId].width;
            }
            return true;
        }
        else if(globalPos.compare("middle_of")==0){
            wL = ((inLaneN/static_cast<double>(_waysDict[wayId].lanes))-0.5) * _waysDict[wayId].width;
            wR = (((_waysDict[wayId].lanes-inLaneN)/static_cast<double>(_waysDict[wayId].lanes))+0.5) * _waysDict[wayId].width;
            return true;
        }
        else return false;
    }
}

double osm::distPointToLine(osmium::geom::Coordinates p1, osmium::geom::Coordinates p2, osmium::geom::Coordinates pObject){
    double deltaY = p2.y - p1.y;
    double deltaX = p2.x - p1.x;

    double dist = abs( deltaY*pObject.x - deltaX*pObject.y + p2.x*p1.y - p2.y*p1.x );
    dist /= sqrt(pow(deltaY,2)+pow(deltaX,2));
    return dist;
}

osmium::geom::Coordinates osm::CoordOrthProjPointToLine(osmium::geom::Coordinates p1, osmium::geom::Coordinates p2, osmium::geom::Coordinates pObject){

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

bool osm::setKdtree(){

    if(_nodesDict.empty()){
        std::cerr<<"Kdtree cannot be initialized. Nodes must be initialized. Please run readOsmFile before."<<std::endl;
        return false;
    }

    // Generate PointCloud for nanoflann from _nodesDict 
    _nodesCloud.pts.resize(std::distance(_nodesDict.begin(), _nodesDict.end()));
    for(std::map<std::pair<double,double>,std::vector<long int>>::iterator it_node = _nodesDict.begin(); it_node!=_nodesDict.end(); ++it_node){
        _nodesCloud.pts[std::distance(_nodesDict.begin(), it_node)].x = it_node->first.first;
        _nodesCloud.pts[std::distance(_nodesDict.begin(), it_node)].y = it_node->first.second;
    }

    // construct a kd-tree index:
    _kdtree = new nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud>, PointCloud, 2 /* dim */>(2 /*dim*/, _nodesCloud, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
    _kdtree->buildIndex();

    return true;
}

bool osm::kdtreeNearestSearch(double x, double y, std::vector<double> &pointNearestSquaredDistance, std::vector<std::pair<double,double>> &pointsNearestSearch, int k){

    if(_nodesCloud.pts.empty()){
        std::cerr<<"Kdtree cannot be used. It might be not initialized. Please run setKdtree before."<<std::endl;
        return false;
    }

    // Transform x,y
    const double query_pt[2] = {x, y};

    // Assure that output vector are empty
    pointsNearestSearch.clear();

    // K nearest neighbor search. Perform a search for the k closest points.
    std::vector<size_t> pointIdxKNNSearch(k);
    pointNearestSquaredDistance = std::vector<double>(k);
    _kdtree->knnSearch(&query_pt[0], k, &pointIdxKNNSearch[0], &pointNearestSquaredDistance[0]);

    if(static_cast<int>(pointIdxKNNSearch.size()) > 0){
        for(auto ptIdx : pointIdxKNNSearch){
            pointsNearestSearch.push_back(std::make_pair(_nodesCloud.pts[ptIdx].x,_nodesCloud.pts[ptIdx].y));
        }
        return true;
    }
    else return false;
}

bool osm::calculBoxCorners(osmium::geom::Coordinates ptNode1, osmium::geom::Coordinates ptNode2, long int wayId, std::vector<osmium::geom::Coordinates> &corners){

    // Assure that output vect is empty.
    corners.clear();

    // Create one perpendicular vector to the line formed by ptNode1 and ptNode2.
    double dx = ptNode2.x - ptNode1.x;
    double dy = ptNode2.y - ptNode1.y;

    osmium::geom::Coordinates perp = osmium::geom::Coordinates(dy,-dx);

    double len = sqrt(pow(perp.x,2)+pow(perp.y,2));

    perp.x = perp.x/len;
    perp.y = perp.y/len;

    double wayWidthL, wayWidthR;
    if(!osm::widthLandR(wayId, wayWidthL, wayWidthR)) return false;
    
    // Compute the points of the search box
    corners.push_back(osmium::geom::Coordinates(ptNode1.x+perp.x*-wayWidthL, ptNode1.y+perp.y*-wayWidthL));
    corners.push_back(osmium::geom::Coordinates(ptNode2.x+perp.x*-wayWidthL, ptNode2.y+perp.y*-wayWidthL));
    corners.push_back(osmium::geom::Coordinates(ptNode2.x+perp.x*wayWidthR, ptNode2.y+perp.y*wayWidthR));
    corners.push_back(osmium::geom::Coordinates(ptNode1.x+perp.x*wayWidthR, ptNode1.y+perp.y*wayWidthR));

    return true;
}

bool osm::objectIsOnBox(std::vector<osmium::geom::Coordinates> corners, osmium::geom::Coordinates objectPosition){
    bool isOnBox = false;

    if(static_cast<int>(corners.size()) < 4){
        std::cerr<<"objectIsOnBox cannot be used. It needed 4 points, "<<corners.size()<<" points have been given in std::vector<osmium::geom::Coordinates> corners."<<std::endl;
        return false;
    }

    // Point Inclusion in Polygon Test by W. Randolph Franklin (WRF).
    int i,j;
    for(i=0, j=static_cast<int>(corners.size())-1; i < static_cast<int>(corners.size()); j=i++){
        if( (((corners[i].y <= objectPosition.y) && (objectPosition.y < corners[j].y)) ||
            ((corners[j].y <= objectPosition.y) && (objectPosition.y < corners[i].y))) &&
            (objectPosition.x < (corners[j].x - corners[i].x) * (objectPosition.y - corners[i].y) / (corners[j].y - corners[i].y) + corners[i].x) ){
            isOnBox=!isOnBox;
        }
    }
    return isOnBox;
}

bool osm::objectIsOnRoad(std::vector<std::pair<double,double>> nearestNodes, osmium::geom::Coordinates objectPosition, std::vector<std::pair<long int,int>> &presentInWays, std::vector<std::pair<int,int>> &betweenNodes){
    bool isOnRoad = false;

    if(_nodesDict.empty()){
        std::cerr<<"objectIsOnRoad cannot be used. Nodes must be initialized. Please run readOsmFile before."<<std::endl;
        return false;
    }

    // Assure that output vector is empty
    presentInWays.clear();

    std::vector<long int> analysedWays;

    for(auto node : nearestNodes){
        // From node get the correspondant way ids
        std::vector<long int> wayIds = _nodesDict[node];
        for(auto wayId : wayIds){
            // If the way has not been analysed
            if(std::find(analysedWays.begin(), analysedWays.end(), wayId) == analysedWays.end()){
                analysedWays.push_back(wayId);
                osmium::geom::Coordinates ptNode = osmium::geom::Coordinates(node.first,node.second);

                // From node and the current way id get oneWay
                oneWay myWay = _waysDict[wayId];

                // From the way, get next and precedent node
                // Following 3 cases : At the end of the way, At the start of the way, On the middle of the way.

                // Initialize next and precedent node
                osmium::geom::Coordinates ptNode1 = osmium::geom::Coordinates(0.0,0.0); // Precedent
                osmium::geom::Coordinates ptNode2 = osmium::geom::Coordinates(0.0,0.0); // Next

                int idx = std::distance(myWay.myNodes.begin(), std::find(myWay.myNodes.begin(), myWay.myNodes.end(), ptNode));

                if(static_cast<uint>(idx) == myWay.myNodes.size()){
                    std::cerr<<"An error occurs in osm::objectIsOnRoad : node is not present in finded way"<<std::endl;
                    return false;
                }
                // End of the way
                else if(static_cast<uint>(idx) == myWay.myNodes.size()-1){
                    ptNode2 = ptNode;
                    ptNode1 = myWay.myNodes[idx-1];

                    std::vector<osmium::geom::Coordinates> searchBox;
                    osm::calculBoxCorners(ptNode1, ptNode2, wayId, searchBox);
                    if(osm::objectIsOnBox(searchBox, objectPosition)){
                        int laneNumber = ceil( osm::distPointToLine(searchBox[0], searchBox[1], objectPosition) / (myWay.width/myWay.lanes) );
                        presentInWays.push_back(std::make_pair(wayId,laneNumber));
                        betweenNodes.push_back(std::make_pair(idx-1,idx));
                        isOnRoad = true;
                    }
                }
                // Beginning of the way
                else if(static_cast<uint>(idx) == 0){
                    ptNode2 = myWay.myNodes[idx+1];
                    ptNode1 = ptNode;

                    std::vector<osmium::geom::Coordinates> searchBox;
                    osm::calculBoxCorners(ptNode1, ptNode2, wayId, searchBox);
                    if(osm::objectIsOnBox(searchBox, objectPosition)){
                        int laneNumber = ceil( osm::distPointToLine(searchBox[0], searchBox[1], objectPosition) / (myWay.width/myWay.lanes) );
                        presentInWays.push_back(std::make_pair(wayId,laneNumber));
                        betweenNodes.push_back(std::make_pair(idx,idx+1));
                        isOnRoad = true;
                    }
                }
                // Middle of the way
                else{
                    ptNode2 = myWay.myNodes[idx+1];
                    ptNode1 = myWay.myNodes[idx-1];

                    std::vector<osmium::geom::Coordinates> searchBox1, searchBox2;
                    osm::calculBoxCorners(ptNode1, ptNode, wayId, searchBox1);
                    osm::calculBoxCorners(ptNode, ptNode2, wayId, searchBox2);
                    std::vector<osmium::geom::Coordinates> searchBox3{searchBox2[0],searchBox1[1],searchBox2[3],searchBox1[2]};

                    if(osm::objectIsOnBox(searchBox1, objectPosition)){
                        int laneNumber = ceil( osm::distPointToLine(searchBox1[0], searchBox1[1], objectPosition) / (myWay.width/myWay.lanes) );
                        presentInWays.push_back(std::make_pair(wayId,laneNumber));
                        betweenNodes.push_back(std::make_pair(idx-1,idx));
                        isOnRoad = true;
                    }
                    else if(osm::objectIsOnBox(searchBox2, objectPosition)){
                        int laneNumber = ceil( osm::distPointToLine(searchBox2[0], searchBox2[1], objectPosition) / (myWay.width/myWay.lanes) );
                        presentInWays.push_back(std::make_pair(wayId,laneNumber));
                        betweenNodes.push_back(std::make_pair(idx,idx+1));
                        isOnRoad = true;
                    }
                    else if(osm::objectIsOnBox(searchBox3, objectPosition)){
                        int laneNumber = ceil( osm::distPointToLine(searchBox3[0], searchBox3[1], objectPosition) / (myWay.width/myWay.lanes) );
                        presentInWays.push_back(std::make_pair(wayId,laneNumber));
                        betweenNodes.push_back(std::make_pair(idx-1,idx));
                        isOnRoad = true;
                    }
                }
            }
        }
    }
    return isOnRoad;
}

bool osm::wayHeadingVsObjectHeading(const std::vector<std::pair<long int,int>> &presentInWays, const std::vector<std::pair<int,int>> &betweenNodes, double objectHeading, std::pair<long int,int> &mostLikelyWay){

    if(presentInWays.empty()){
        std::cerr<<"wayHeadingVsObjectHeading cannot be used. presentInWays must be initialized. Please run objectIsOnRoad before."<<std::endl;
        return false;
    }

    // The aim is to find the smallest difference between road and car heading to find the most likely way
    double delta = 1000.0; // Higher value to initialize the research of the smallest

    for(int idxPreInWays = 0; idxPreInWays < static_cast<int>(presentInWays.size()); ++idxPreInWays){

        // Calculate the heading of the way
        double dx = _waysDict[presentInWays[idxPreInWays].first].myNodes[betweenNodes[idxPreInWays].second].x - _waysDict[presentInWays[idxPreInWays].first].myNodes[betweenNodes[idxPreInWays].first].x;
        double dy = _waysDict[presentInWays[idxPreInWays].first].myNodes[betweenNodes[idxPreInWays].second].y - _waysDict[presentInWays[idxPreInWays].first].myNodes[betweenNodes[idxPreInWays].first].y;
        double wayHeading = mp::rad_deg(atan2(dy,dx));

        // If the difference between the objectHeading and the wayHeading is around 180.0 degree
        // AND the way represented by the node is not an one-way road THEN it is important to "invert" the roadHeading
        if(!_waysDict[presentInWays[idxPreInWays].first].oneway && abs(wayHeading-objectHeading) > 165.0 && abs(wayHeading-objectHeading) < 195.0){
            dx = _waysDict[presentInWays[idxPreInWays].first].myNodes[betweenNodes[idxPreInWays].first].x - _waysDict[presentInWays[idxPreInWays].first].myNodes[betweenNodes[idxPreInWays].second].x;
            dy = _waysDict[presentInWays[idxPreInWays].first].myNodes[betweenNodes[idxPreInWays].first].y - _waysDict[presentInWays[idxPreInWays].first].myNodes[betweenNodes[idxPreInWays].second].y;
            wayHeading = mp::rad_deg(atan2(dy,dx));
        }

        if(abs(wayHeading-objectHeading) <= delta){
            delta = abs(wayHeading-objectHeading);
            mostLikelyWay = presentInWays[idxPreInWays];
        }
    }
    return true;
}

std::vector<long int> osm::nextPossibleWays(long int fromWayId, std::pair<int,int> betweenNodes, double objectHeading, std::pair<double,double> &referenceNode){

    //------------------------------------------------
    // First step : Get the node which represent the end of the current way, fromWayId.
    // If "referenceNode" is not already initialized from "objectHeading" and "betweenNodes".
    // Else from "referenceNode" passed as parameter
    // The aim is to use _nodesDict from this node to get all next possible way.
    //------------------------------------------------

    if(std::isnan(referenceNode.first) || std::isnan(referenceNode.second)){
        // Take care about particular case
        // The way, fromWayId, is not an one-way road and the nodes are not stored according to the direction of the car
        if(!_waysDict[fromWayId].oneway){

            // Calculate the heading of the way
            double dx = _waysDict[fromWayId].myNodes[betweenNodes.second].x - _waysDict[fromWayId].myNodes[betweenNodes.first].x;
            double dy = _waysDict[fromWayId].myNodes[betweenNodes.second].y - _waysDict[fromWayId].myNodes[betweenNodes.first].y;
            double wayHeading = mp::rad_deg(atan2(dy,dx));

            // From heading of the car and the way, check if the car move in nodes list direction
            if(abs(wayHeading-objectHeading) > 165.0 && abs(wayHeading-objectHeading) < 195.0){
                // End node is represented by the first node
                referenceNode = std::make_pair(_waysDict[fromWayId].myNodes[0].x, _waysDict[fromWayId].myNodes[0].y);
            }
            else referenceNode = std::make_pair(_waysDict[fromWayId].myNodes.back().x, _waysDict[fromWayId].myNodes.back().y);
        }
        else referenceNode = std::make_pair(_waysDict[fromWayId].myNodes.back().x, _waysDict[fromWayId].myNodes.back().y);
    }
    else{

        if(!_waysDict[fromWayId].oneway){
            if(referenceNode.first == _waysDict[fromWayId].myNodes.back().x && referenceNode.second == _waysDict[fromWayId].myNodes.back().y){
                referenceNode = std::make_pair(_waysDict[fromWayId].myNodes[0].x, _waysDict[fromWayId].myNodes[0].y);
            }
            else referenceNode = std::make_pair(_waysDict[fromWayId].myNodes.back().x, _waysDict[fromWayId].myNodes.back().y);
        }
        else referenceNode = std::make_pair(_waysDict[fromWayId].myNodes.back().x, _waysDict[fromWayId].myNodes.back().y);
    }
    //------------------------------------------------
    // Second step : From the node get all next possible ways and keep way which is according to the entrance direction
    //------------------------------------------------
    std::vector<long int> nextPossibleWays_tmp = _nodesDict[referenceNode];
    std::vector<long int> nextPossibleWays;

    for(auto nextWayId : nextPossibleWays_tmp){
        // The current way, fromWayId, must not be take into account
        if(nextWayId != fromWayId){
            // If the next way is not an one-way road, add it
            if(!_waysDict[nextWayId].oneway){
                nextPossibleWays.push_back(nextWayId);
            }
            // Else, check if the first node of the next way and ending node of current way are equal.
            else if(_waysDict[nextWayId].myNodes[0].x == referenceNode.first && _waysDict[nextWayId].myNodes[0].y == referenceNode.second){
                nextPossibleWays.push_back(nextWayId);
            }
        }
    }

    return nextPossibleWays;
}

bool osm::nextPossiblePaths(long int fromWayId, std::pair<int,int> betweenNodes, double objectHeading, std::pair<double,double> objectPos, std::vector<std::vector<long int>> &nextPossiblePaths, double minimalPathSize){

    //------------------------------------------------
    // By recursivity, find all next accessible ways from the heading and the position in OSM point of view (betweenNodes) of an object.
    // Stop criteria for one path : size of road in meters is bigger than minimalPathSize (in meters).
    //------------------------------------------------

    if(_waysDict.empty()){
        std::cerr<<"nextPossiblePaths cannot be used. Ways must be initialized. Please run readOsmFile before."<<std::endl;
        return false;
    }

    //--------------------
    // For stop criteria : Compute distance between object and the end of the way
    //--------------------
    // Calculate the heading of the way
    bool sameDirection;
    double dx = _waysDict[fromWayId].myNodes[betweenNodes.second].x - _waysDict[fromWayId].myNodes[betweenNodes.first].x;
    double dy = _waysDict[fromWayId].myNodes[betweenNodes.second].y - _waysDict[fromWayId].myNodes[betweenNodes.first].y;
    double wayHeading = mp::rad_deg(atan2(dy,dx));

    // From heading of the car and the way, check if the car moves in nodes list direction
    if(abs(wayHeading-objectHeading) > 165.0 && abs(wayHeading-objectHeading) < 195.0){
        // End node is represented by the first node
        sameDirection = false;
    }
    else sameDirection = true;

    osmium::geom::Coordinates objectCoord = osm::CoordOrthProjPointToLine(_waysDict[fromWayId].myNodes[betweenNodes.first], _waysDict[fromWayId].myNodes[betweenNodes.second], osmium::geom::Coordinates(objectPos.first,objectPos.second));
    double Firstlength = 0.0;
    // object to betweenNodes.second to end
    if(sameDirection){
        Firstlength += sqrt(pow(_waysDict[fromWayId].myNodes[betweenNodes.second].y-objectCoord.y,2)+pow(_waysDict[fromWayId].myNodes[betweenNodes.second].x-objectCoord.x,2));
        for (int i = betweenNodes.second; i < static_cast<int>(_waysDict[fromWayId].myNodes.size())-1; ++i)
        {
            Firstlength += sqrt(pow(_waysDict[fromWayId].myNodes[i+1].y-_waysDict[fromWayId].myNodes[i].y,2)+pow(_waysDict[fromWayId].myNodes[i+1].x-_waysDict[fromWayId].myNodes[i].x,2));
        }
    }
    // object to betweenNodes.first to 0
    else{
        Firstlength += sqrt(pow(objectCoord.y-_waysDict[fromWayId].myNodes[betweenNodes.first].y,2)+pow(objectCoord.x-_waysDict[fromWayId].myNodes[betweenNodes.first].x,2));
        for (int i = betweenNodes.first; i > 0; --i)
        {
            Firstlength += sqrt(pow(_waysDict[fromWayId].myNodes[i].y-_waysDict[fromWayId].myNodes[i-1].y,2)+pow(_waysDict[fromWayId].myNodes[i].x-_waysDict[fromWayId].myNodes[i-1].x,2));
        }
    }

    //----------------
    // Initialization
    //----------------
    nextPossiblePaths.clear(); // Assure that output vector is empty
    
    std::vector<std::vector<long int>> nextPossiblePaths_search; // Collection of path which represents the begining of new possible paths
    nextPossiblePaths_search.push_back(std::vector<long int>{fromWayId}); // Initialize the first possible path <=> current way.
    
    std::vector<std::pair<double,double>> referenceNodes;
    // To initialize the recursive process, referenceNode must be set as <nan,nan>.
    referenceNodes.push_back(std::make_pair(std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN()));

    //----------------
    // Process
    //----------------
    std::vector<std::vector<long int>> nextPossiblePaths_tmp;
    std::vector<std::pair<double,double>> referenceNodes_tmp;
    std::pair<double,double> referenceNode;

    std::vector<long int> oneNewFinalPath;
    std::vector<long int> nextPossibleWays;
    bool pathTooSmall = false;

    while(!nextPossiblePaths_search.empty())
    {
        for(std::vector<long int> oneFinalPath : nextPossiblePaths_search){

            referenceNode = referenceNodes[ std::find(nextPossiblePaths_search.begin(),nextPossiblePaths_search.end(),oneFinalPath) - nextPossiblePaths_search.begin() ];            
            nextPossibleWays = osm::nextPossibleWays(oneFinalPath.back(), betweenNodes, objectHeading, referenceNode);
            
            bool allNextWayHaveDifferentType = true;
            for(long int wayId : nextPossibleWays){

                // If the type of way changes OR If the new possible way is already in the current path, ignore it
                if(std::find(oneFinalPath.begin(), oneFinalPath.end(), wayId) == oneFinalPath.end() && _waysDict[fromWayId].type == _waysDict[wayId].type){
                  
                    if(allNextWayHaveDifferentType) allNextWayHaveDifferentType = false;

                    oneNewFinalPath = oneFinalPath;
                    oneNewFinalPath.push_back(wayId);

                    // If this path, oneNewFinalPath, is bigger than minimal size, export (store it into output vector)
                    double pathLength = Firstlength;
                    for (int i = 1; i < static_cast<int>(oneNewFinalPath.size()); ++i)
                    {
                        for (int j = 0; j < static_cast<int>(_waysDict[oneNewFinalPath[i]].myNodes.size())-1; ++j)
                        {
                            pathLength += sqrt(pow(_waysDict[oneNewFinalPath[i]].myNodes[j+1].y-_waysDict[oneNewFinalPath[i]].myNodes[j].y,2)+pow(_waysDict[oneNewFinalPath[i]].myNodes[j+1].x-_waysDict[oneNewFinalPath[i]].myNodes[j].x,2));
                        }
                    }

                    if(pathLength >=  minimalPathSize) nextPossiblePaths.push_back(oneNewFinalPath);
                    // Else add it into nextPossiblePaths_tmp for the next search
                    else{
                        nextPossiblePaths_tmp.push_back(oneNewFinalPath);
                        referenceNodes_tmp.push_back(referenceNode);
                    }
                }
            }

            if(nextPossibleWays.empty() || allNextWayHaveDifferentType){
                // Add current possible path (oneFinalPath) into output vector and notify that there is/are incomplete path(s) in the output
                nextPossiblePaths.push_back(oneFinalPath);
                pathTooSmall = true;
            }
        }
        
        // If nextPossiblePaths_tmp is empty and the output vector is empty, export current possible paths (nextPossiblePaths_search) and notify that they are incomplete
        if(nextPossiblePaths_tmp.empty() && nextPossiblePaths.empty()){
            nextPossiblePaths = nextPossiblePaths_search;
            std::cerr<<"nextPossiblePaths : All paths are incomplete (<";
            std::cerr<<minimalPathSize;
            std::cerr<<"m), not enough data in OSM map !"<<std::endl;
            return false;
        }

        nextPossiblePaths_search = nextPossiblePaths_tmp;
        nextPossiblePaths_tmp.clear();

        referenceNodes = referenceNodes_tmp;
        referenceNodes_tmp.clear();
    }

    if(nextPossiblePaths.empty()){
        std::cerr<<"nextPossiblePaths : Output is empty, not enough data in OSM map !"<<std::endl;
        return false;
    }

    // If there are too small paths, notify that and return false
    if(pathTooSmall){
        std::cerr<<"nextPossiblePaths : Some paths are incomplete (<";
        std::cerr<<minimalPathSize;
        std::cerr<<"m), not enough data in OSM map !"<<std::endl;
        return false;
    }

    // All possible paths (>= minimalPathSize) are found.
    return true;
}


bool osm::sortedNodesFromPath(std::pair<int,int> betweenNodes, double objectHeading, std::pair<double,double> objectPos, std::vector<long int> path, std::vector<osmium::geom::Coordinates> &sortedNodesPath, std::vector<int> &speedLimit){

    if(_waysDict.empty()){
        std::cerr<<"sortedNodesFromPath cannot be used. Ways must be initialized. Please run readOsmFile before."<<std::endl;
        return false;
    }

    if(path.empty()){
        std::cerr<<"sortedNodesFromPath cannot be used. Paths must be initialized. Please use nextPossiblePaths before."<<std::endl;
        return false;
    }

    // Assure that output are empty
    sortedNodesPath.clear();
    speedLimit.clear();

    //--------------------
    // First step : Compute first node from object position and push_back the next parts of the way
    //--------------------
    sortedNodesPath.push_back(osm::CoordOrthProjPointToLine(_waysDict[path[0]].myNodes[betweenNodes.first], _waysDict[path[0]].myNodes[betweenNodes.second], osmium::geom::Coordinates(objectPos.first,objectPos.second)));
    speedLimit.push_back(_waysDict[path[0]].maxspeed);

    // Calculate the heading of the way
    bool sameDirection;
    double dx = _waysDict[path[0]].myNodes[betweenNodes.second].x - _waysDict[path[0]].myNodes[betweenNodes.first].x;
    double dy = _waysDict[path[0]].myNodes[betweenNodes.second].y - _waysDict[path[0]].myNodes[betweenNodes.first].y;
    double wayHeading = mp::rad_deg(atan2(dy,dx));

    // From heading of the car and the way, check if the car moves in nodes list direction
    if(abs(wayHeading-objectHeading) > 165.0 && abs(wayHeading-objectHeading) < 195.0){
        // End node is represented by the first node
        sameDirection = false;
    }
    else sameDirection = true;

    // object to betweenNodes.second to end
    if(sameDirection){
        for (int i = betweenNodes.second; i < static_cast<int>(_waysDict[path[0]].myNodes.size()); ++i)
        {
            sortedNodesPath.push_back(_waysDict[path[0]].myNodes[i]);
            speedLimit.push_back(_waysDict[path[0]].maxspeed);
        }
    }
    // object to betweenNodes.first to 0
    else{
        for (int i = betweenNodes.first; i > 0; --i)
        {
            sortedNodesPath.push_back(_waysDict[path[0]].myNodes[i]);
            speedLimit.push_back(_waysDict[path[0]].maxspeed);
        }
    }

    //--------------------
    // Second step : Add the next nodes in correct direction
    //--------------------
    for (int i = 1; i < static_cast<int>(path.size()); ++i)
    {
        if(sortedNodesPath.back().x == _waysDict[path[i]].myNodes[0].x && sortedNodesPath.back().y == _waysDict[path[i]].myNodes[0].y){
            for (int idxNode = 0; idxNode < static_cast<int>(_waysDict[path[i]].myNodes.size()); ++idxNode)
            {
                sortedNodesPath.push_back(_waysDict[path[i]].myNodes[idxNode]);
                speedLimit.push_back(_waysDict[path[i]].maxspeed);
            }
        }
        else{
            for (int idxNode = _waysDict[path[i]].myNodes.size()-1; idxNode >= 0; --idxNode)
            {
                sortedNodesPath.push_back(_waysDict[path[i]].myNodes[idxNode]);
                speedLimit.push_back(_waysDict[path[i]].maxspeed);
            }
        }
    }
    return true;
}

bool osm::getTrafficRules(std::vector<osmium::geom::Coordinates> nodes, std::vector<std::string> &trafficRules){

    if(_trafficRules.empty()){
        std::cerr<<"getTrafficRules cannot be used. Traffic rules must be initialized. Please run readOsmFile before."<<std::endl;
        return false;
    }

    if(nodes.empty()){
        std::cerr<<"getTrafficRules cannot be used. Input path is empty."<<std::endl;
        return false;
    }

    trafficRules.clear();
    std::map<std::pair<double,double>,std::string>::iterator it_node;

    for(auto node : nodes){
        it_node = _trafficRules.find(std::make_pair(node.x, node.y));
        if(it_node != _trafficRules.end()){
            trafficRules.push_back(it_node->second);
        } else trafficRules.push_back("no");
    }
    
    return true;
}