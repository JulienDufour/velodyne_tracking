// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// oxts.h
// Purpose: This class takes in oxts measurements and converts them
// 	to local 3D poses where the first measurement is positioned at (0,0,0).
// 	The poses are in the form of a transformation matrix R.
// 	The code is converted from matlab code that can be found in the
// 	raw data development kit for the KITTI dataset and can be found at.
// 	http://www.cvlibs.net/datasets/kitti/raw_data.php.
//
// @author Unnar Þór Axelsson
// @version 1.0 19/01/16
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


#ifndef OXTS_H
#define OXTS_H

#include <pcl/point_cloud.h>
#include <Eigen/Dense>

#include <mercatorProjection/mercatorProjection.h>

using namespace std;
using namespace pcl;
using namespace Eigen;

class oxts{

	int returnIdx;
	bool initialized = false;
	double _scale = 0;
	Matrix4d Tr_0_inv;
	vector<Matrix4d> poses;
    vector<double> first_pose;

public:
	oxts() : returnIdx(0) {};
	~oxts(){};

	// Inputs a new pose from a gps measurement, calculates a 4x4 transformation
	// matrix and stores in a vector.
	void newPose(const vector<double> oxtsPose);

	// Returns a 4x4 transformation matrix according to an internal index.
	// This index is incremented each time this method is called until all
	// transformations have been returned.
	Matrix4d returnPose(void);

	// Returns a 4x4 transformation matrix with index idx.
	Matrix4d returnPose(const int idx);

	// Prints all calculated transformation matrices.
	void printAllPoses(void);

	std::vector<double> firstPose(){
		return first_pose;
	}

	Matrix4d firstPoseMatrix(){
		return Tr_0_inv;
	}

	int size(void);

};


#endif
