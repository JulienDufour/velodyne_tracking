// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// dataio.h
// Purpose: Utility namespace grouping together methods that deal with reading the KITTI
// dataset files and get them on a easily used form.

// @author Unnar Þór Axelsson | Dufour Julien
// @version 1.0 18/01/16
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


#ifndef DATAIO_H
#define DATAIO_H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <fstream>
#include <cstddef>
#include <time.h>
#include <boost/filesystem.hpp>
#include <stdlib.h>
#include <objectTracking/object.h>
#include <limits>

typedef std::numeric_limits< double > dbl;
namespace fs = boost::filesystem;

namespace utils{

/**
 * @brief This struct contains all possible transformations in the system (All CS are in meters).
 */
struct transformMatrix{
    Eigen::Matrix4d oxts_pose;              /*!< Mercator CS to IMU/GPS kitti car CS.*/
    Eigen::Matrix4d calib_imu_to_velo;      /*!< On kitti car : IMU/GPS CS to Velodyne laserscanner CS.*/
    Eigen::Matrix4d calib_velo_to_cam;      /*!< On kitti car : Velodyne laserscanner CS to Cam0(gray) CS.*/
    Eigen::Matrix4d calib_cam_to_camRect;   /*!< On kitti car : Cam0(gray) CS to Cam0 rectified CS.*/
    Eigen::MatrixXd calib_camRect_to_Image; /*!< On kitti car : Cam0 rectified CS to Image CS <=> Projection matrix.*/
};

/**
 * @brief This struct represents one data line for extracted paths contained in csv file.
 *        One line is represented by : timestamp1, x, y, z, vx, vy, vz, qx, qy, qz, qw, sx, sy, sz.
 *        For more information see : /README.md.
 */
struct ObjectPath{
	unsigned long time;
	Eigen::VectorXd position;
	Eigen::Quaternionf quaternion;
	float x;
	float y;
	float z;
};

// Gets the filenames of all files of a specific type within a folder.
// Inputs:
//  path, the path to the folder.
//  type, the file type that we are searching for.
// Output:
//  filenames, vector containing all the filenames.
bool getFileNames(const std::string path, const std::string type, std::vector<std::string> &filenames){
	fs::path p(path);
    if(!exists(p) || !is_directory(p)) {
        std::cout << p << " is not a path\n";
        return false;
    }
    fs::recursive_directory_iterator begin(p), end;
    std::vector<fs::directory_entry> v(begin, end);
    std::size_t dot;
    std::string file;
    for(auto& f: v){
    	file = f.path().string();
    	dot = file.find_last_of(".\\");
    	if(file.substr(dot+1) == type){ // Correct file type
	    	filenames.push_back(file.substr(0,dot).substr(file.substr(0,dot).find_last_of("/\\")+1));
    	}
    }
    std::sort(filenames.begin(), filenames.end());
    std::cout << "There are " << filenames.size() << " files of type " << type << std::endl;
    return true;
};

// Reads in binary PointCloud files from the KITTI dataset and creates PCL::PointClouds.
// Input:
//  name, the name of the file to be read with full path.
// Output:
//  cloud, PCL PointCloud.
bool kitti2pcl(const std::string name, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){

	// need empty pointcloud.
	if(cloud->points.size() != 0){
		cloud->clear();
	}

	std::fstream input(name, std::ios::in | std::ios::binary);
	if(!input.good()){
		std::cerr << "Could not read file: " << name << std::endl;
		return false;
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
	return true;
};

// Reads in binary PointCloud files from the KITTI dataset and creates PCL::PointClouds.
// Input:
//  path, the full path to the file.
//  name, the name of the file to be read.
//  type, the filetype of the file.
// Output:
//  cloud, PCL PointCloud.
bool kitti2pcl(const std::string path, const std::string name, const std::string type, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
	return kitti2pcl(path + name + "." + type, cloud);
};

/**
 *  @brief readRigidTransformFile  Read a txt file containing rigid transformation and stores it in Matrix4d.
 *  @param path                    Represents the full path to the file.                              (input)
 *  @param name                    Represents the name of the file to be read.                        (input)
 *  @param type                    Represents the filetype of the file.                               (input)
 *  @param mat                     Matrix4d class object which represents the rigid transformation.   (output)
 *  @return                        True if no problem occurs.
 */
bool readRigidTransformFile(const std::string path, const std::string name, const std::string type, Eigen::Matrix4d &mat){
	std::ifstream myfile(path + name + "." + type);
    std::string line;
    std::string val;
    std::stringstream ss;

    mat.setIdentity();
    
    if(myfile.is_open()){
        
        // Line 0 : (example) calib_time: 25-May-2012 16:47:16
        std::getline(myfile,line);
        
        // Line 1 : R (example) R: 9.999976e-01 7.553071e-04 -2.035826e-03 -7.854027e-04 9.998898e-01 -1.482298e-02 2.024406e-03 1.482454e-02 9.998881e-01
        std::getline(myfile,line);
        // Build an istream that holds the input string
        ss.clear(); // Clear ss
        ss << line;
        ss >> val;
        for (int i = 0; i < 3; ++i)
        {
        	for (int j = 0; j < 3; ++j)
        	{
        		ss >> val;
        		mat(i,j) = (double)atof(val.c_str());
        	}
        }

        // Line 2 : T (example) T: -8.086759e-01 3.195559e-01 -7.997231e-01
        std::getline(myfile,line);
        // Build an istream that holds the input string
		ss.clear(); // Clear ss        
		ss << line;
        ss >> val;
        for (int i = 0; i < 3; ++i)
        {
        	ss >> val;
        	mat(i,3) = (double)atof(val.c_str());
        }

        myfile.close();
        return true;
    }
    return false;
}

/**
 *  @brief readCamTransformFile  Read a txt file containing cam transformation and stores it in Matrix4d and MatrixXd.
 *  @param path                  Represents the full path to the file.                                       (input)
 *  @param name                  Represents the name of the file to be read.                                 (input)
 *  @param type                  Represents the filetype of the file.                                        (input)
 *  @param mat                   Matrix4d class object which represents the cam_to_camRect transformation.   (output)
 *  @param proj                  MatrixXd class object which represents the projection matrix.               (output)
 *  @return                      True if no problem occurs.
 */
bool readCamTransformFile(const std::string path, const std::string name, const std::string type, Eigen::Matrix4d &mat, Eigen::MatrixXd &proj){
	std::ifstream myfile(path + name + "." + type);
    std::string line;
    std::string val;
    std::stringstream ss;

    mat.setIdentity();

    if(myfile.is_open()){
        
        // Looking for Line 6
        for (int i = 0; i < 5; ++i)
        {
        	std::getline(myfile,line);
        }
        
        // Line 6 : R (example) R: 9.999976e-01 7.553071e-04 -2.035826e-03 -7.854027e-04 9.998898e-01 -1.482298e-02 2.024406e-03 1.482454e-02 9.998881e-01
        std::getline(myfile,line);
        // Build an istream that holds the input string
        ss.clear(); // Clear ss
        ss << line;
        ss >> val;
        for (int i = 0; i < 3; ++i)
        {
        	for (int j = 0; j < 3; ++j)
        	{
        		ss >> val;
        		mat(i,j) = (double)atof(val.c_str());
        	}
        }

        // Line 7 : T (example) T: -8.086759e-01 3.195559e-01 -7.997231e-01
        std::getline(myfile,line);
        Eigen::Vector3d T;
        // Build an istream that holds the input string
        ss.clear(); // Clear ss
        ss << line;
        ss >> val;
        for (int i = 0; i < 3; ++i)
        {
        	ss >> val;
        	mat(i,3) = (double)atof(val.c_str());
        }

		// Line 9 -> R_Rect
		std::getline(myfile,line);
		std::getline(myfile,line);
		Eigen::Matrix4d tmpMat;
		tmpMat.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1

		// Build an istream that holds the input string
        ss.clear(); // Clear ss
        ss << line;
        ss >> val;
        for (int i = 0; i < 3; ++i)
        {
        	for (int j = 0; j < 3; ++j)
        	{
        		ss >> val;
        		tmpMat(i,j) = (double)atof(val.c_str());
        	}
        }

		//Set calib_cam_to_camRect
		mat = tmpMat*mat;

		// Line 10 -> Projection Matrix
		std::getline(myfile,line);
		// Build an istream that holds the input string
        ss.clear(); // Clear ss
        ss << line;
        ss >> val;
        proj.resize(3,4);
        for (int i = 0; i < 3; ++i)
        {
        	for (int j = 0; j < 4; ++j)
        	{
        		ss >> val;
        		proj(i,j) = (double)atof(val.c_str());
        	}
        }

        myfile.close();
        return true;
    }
    return false;

}

// Reads a txt file containing gps information and stores it in a vector. each txt file only containes one location.
// Input:
//  path, the full path to the file.
//  name, the name of the file to be read.
//  type, the filetype of the file.
// Output:
//  vec, vector containing the gps information.
bool readGPSFile(const std::string path, const std::string name, const std::string type, std::vector<double> &vec){

	if(vec.size() != 0){
		vec.clear();
	}

	std::ifstream myfile(path + name + "." + type);
    std::string line;
    if(myfile.is_open()){
        while(std::getline(myfile,line)){

            // Build an istream that holds the input string
            std::istringstream iss(line);

            // Iterate over the istream, using >> to grab floats
            // and push_back to store them in the vector
            std::copy(std::istream_iterator<double>(iss),
                std::istream_iterator<double>(),
                std::back_inserter(vec));
        }
        // std::cout << "veccc size: " << vec.size() << std::endl;
        // Verify that vec contains good values with good precision
        /* 
        std::cout << "GPS" << std::setprecision(std::numeric_limits<double>::digits10 +1) << std::endl;
        for (unsigned i=0; i < vec.size(); i++) {
   			std::cout<<vec[i]<<" - ";
		}
		std::cout<<std::endl;
		*/
        myfile.close();
        return true;
    }
    return false;
};

// Converts string of the form year-month-date hour:minutes:seconds.milliseconds
// into unsigned long (number of digits is 19).
bool timestampToUnsignedLong(std::string timestamp, unsigned long &time){
	struct tm tm;
	if ( strptime(timestamp.c_str(), "%Y-%m-%d %H:%M:%S", &tm) != NULL ){
		unsigned long sec = (unsigned long)(mktime(&tm)*1000000000);
		unsigned long milli = (unsigned long)(std::atof(("0." +
					timestamp.substr(timestamp.find_last_of(".\\")+1)).c_str())*1000000000);
		time = sec+milli;
		return true;
	}
	return false;
}

// Converts unsigned long (number of digits is 19) into string on they
// form year-month-date hour:minutes:seconds.milliseconds.
std::string longTimetoStringTime(unsigned long time){

    time_t tt = std::floor(time/1000000000.0);
    struct tm * tm;
    tm = localtime(&tt);

    std::stringstream ss;
    ss << 1900+tm->tm_year << "-" << std::setfill('0') << std::setw(2) << 1+tm->tm_mon << "-" << tm->tm_mday << " ";
    ss << tm->tm_hour << ":" << std::setfill('0') << std::setw(2) << tm->tm_min << ":";
    ss << std::setfill('0') << std::setw(2) << tm->tm_sec << ".";
    ss << std::to_string(time).substr(10,9);
    return ss.str();
};

// Reads in measurement timestamps from a txt file,
// Input:
//  path, the full path to the file.
//  name, the name of the file to be read.
//  type, the filetype of the file.
// Output:
//  times, vector containing all the timetamps, stored as a double.
void readTimeStamps(const std::string path,
					const std::string name,
					const std::string type,
					std::vector<unsigned long> &times){

    if(times.size() != 0){
        times.clear();
    }

    std::cout << path + name + "." + type << std::endl;

    std::ifstream myfile(path + name + "." + type);
    std::string line;
	unsigned long time;
    if(myfile.is_open()){
        while(std::getline(myfile,line)){
			if(utils::timestampToUnsignedLong(line, time)){
				times.push_back( time);
			}
        }
    }
}

// Takes in a list of objects and writes there respective paths to a csv file
// See README for file structure.
// The method first checks each object for the length of it's path and only writes
// the path if it exceeds a fixed limit.
void writePathsToFile(std::vector<double> oxts_origin, std::vector<object> objects, std::string save_path, std::string file_name){

	ofstream myfile;
	myfile.open(save_path + "/" + file_name + ".csv", std::ofstream::out | std::ofstream::trunc);
	int idx = 0;

	// Set precision in myFile
	myfile << std::setprecision(std::numeric_limits<double>::digits10 +1);

    myfile << "# oxts origin\n";
    for (size_t i = 0; i < oxts_origin.size(); i++) {
        myfile << oxts_origin[i];
        if(i != oxts_origin.size()-1){
            myfile << ",";
        } else {
            myfile << "\n";
        }
    }

	for(auto object : objects){
		if(object.isMoving()){
			myfile << " \n";
			myfile << "# Track: " << ++idx << "\n";
			for(auto pos : object.getPath()){
				myfile << utils::longTimetoStringTime(pos.time) << ",";
                myfile << pos.position(1) << "," << pos.position(2) << "," << pos.position(3) << ",";
                myfile << pos.position(4) << "," << pos.position(5) << "," << pos.position(6) << ",";
                myfile << pos.quaternion.x() << "," << pos.quaternion.y() << ",";
                myfile << pos.quaternion.z() << "," << pos.quaternion.w() << ",";
                myfile << pos.x << "," << pos.y << "," << pos.z << "\n";
			}
		}
	}
    myfile.close();
}

// Reads in paths from a csv file and stores them in a vector. where each position
// in the path is stored as a object::ObjectPath.
// For the csv file structure se README.
bool readPathsFromFile(std::string file_path, std::string name, std::vector<std::vector<object::ObjectPath> > &paths ){

	const int NUM_FIELDS = 14;

	if(paths.size() > 0) paths.clear();


	ifstream myfile (file_path + name + ".csv");
	std::string line;
	std::string field;
	std::vector<object::ObjectPath> path;

	if (myfile.is_open()){
		while ( std::getline (myfile,line) ){

			if(line == "# oxts origin"){
				std::cout << "FOUND OXTS DATA" << std::endl;
				std::getline (myfile,line);
				// std::getline (myfile,line);
			}

			else if(line.substr(0,8) == "# Track:"){
				std::cout << "FOUND " << line << std::endl;
				if(path.size() != 0){
					paths.push_back(path);
					path.clear();
				}
			}

			else if(line == "" || line == " ") continue;

			else{
				stringstream linestream(line);
				object::ObjectPath pos;
				ObjectEKF::Vector tmppos(6);
				for (size_t i = 0; i < NUM_FIELDS; i++) {
					std::getline(linestream, field, ',');
					switch(i){
						case 0: utils::timestampToUnsignedLong(field, pos.time);break;
						case 1: tmppos(1) = std::atof(field.c_str()); break;
						case 2: tmppos(2) = std::atof(field.c_str()); break;
						case 3: tmppos(3) = std::atof(field.c_str()); break;
						case 4: tmppos(4) = std::atof(field.c_str()); break;
						case 5: tmppos(5) = std::atof(field.c_str()); break;
						case 6: tmppos(6) = std::atof(field.c_str()); break;
						case 7: pos.quaternion.x() = std::atof(field.c_str()); break;
						case 8: pos.quaternion.y() = std::atof(field.c_str()); break;
						case 9: pos.quaternion.z() = std::atof(field.c_str()); break;
						case 10: pos.quaternion.w() = std::atof(field.c_str()); break;
						case 11: pos.x = std::atof(field.c_str()); break;
						case 12: pos.y = std::atof(field.c_str()); break;
						case 13: pos.z = std::atof(field.c_str()); break;
					}
				}
				pos.position = tmppos;
				path.push_back(pos);
			}
		}
        paths.push_back(path);
		myfile.close();
		return true;
	}
	return false;
};

// Reads in paths from a csv file and stores them in a vector. where each position
// in the path is stored as a utils::ObjectPath.
// For the csv file structure se README.
bool readPathsFromFile(std::string file_path, std::string name, std::vector<double> &gpsFirstPose, std::vector<std::vector<utils::ObjectPath> > &paths ){

	const int NUM_FIELDS = 14;

	if(paths.size() > 0) paths.clear();
    gpsFirstPose.clear();

	ifstream myfile (file_path + name + ".csv");
	std::string line;
	std::string field;
	std::vector<utils::ObjectPath> path;

	if (myfile.is_open()){
		while ( std::getline (myfile,line) ){

			if(line == "# oxts origin"){
				//std::cout << "FOUND OXTS DATA" << std::endl;
				std::getline (myfile,line);
				
                std::istringstream iss(line);
                while(std::getline(iss, field, ',')){
                    gpsFirstPose.push_back( std::stof(field) );
                }
			}

			else if(line.substr(0,8) == "# Track:"){
				//std::cout << "FOUND " << line << std::endl;
				if(path.size() != 0){
					paths.push_back(path);
					path.clear();
				}
			}

			else if(line == "" || line == " ") continue;

			else{
				stringstream linestream(line);
				utils::ObjectPath pos;
				Eigen::VectorXd tmppos(6);
				for (size_t i = 0; i < NUM_FIELDS; i++) {
					std::getline(linestream, field, ',');
					switch(i){
						case 0: utils::timestampToUnsignedLong(field, pos.time);break;
						case 1: tmppos[0] = std::atof(field.c_str()); break;
						case 2: tmppos[1] = std::atof(field.c_str()); break;
						case 3: tmppos[2] = std::atof(field.c_str()); break;
						case 4: tmppos[3] = std::atof(field.c_str()); break;
						case 5: tmppos[4] = std::atof(field.c_str()); break;
						case 6: tmppos[5] = std::atof(field.c_str()); break;
						case 7: pos.quaternion.x() = std::atof(field.c_str()); break;
						case 8: pos.quaternion.y() = std::atof(field.c_str()); break;
						case 9: pos.quaternion.z() = std::atof(field.c_str()); break;
						case 10: pos.quaternion.w() = std::atof(field.c_str()); break;
						case 11: pos.x = std::atof(field.c_str()); break;
						case 12: pos.y = std::atof(field.c_str()); break;
						case 13: pos.z = std::atof(field.c_str()); break;
					}
				}
				pos.position = tmppos;
				path.push_back(pos);
			}
		}
        paths.push_back(path);
		myfile.close();
		return true;
	}
	return false;
};

// Method that takes in a path and creates the directories if they do not exist.
void createDirectories(std::string path){
	fs::path dir(path);
    if(fs::create_directories(dir)){
        std::cerr << "Directory Created: " << path << std::endl;
    }
};

}
#endif
