#include <oxts/oxts.h>

void oxts::newPose(const vector<double> oxtsPose){

	if(oxtsPose.size() != 30){
	// Bad measurement, wrong size.
		return;
	}

	if(!initialized){
        first_pose = oxtsPose;

	    // ECCENT from mercatorProjection.h
	    double e = ECCENT;
    	_scale = sqrt( 1 - pow(e,2) * pow(sin(mp::deg_rad(oxtsPose[0])),2)) / cos(mp::deg_rad(oxtsPose[0]));
	}

	// Create the transformation matrix.
	// Matrix4f transform = Matrix4f::Identity();
	Eigen::Affine3d transform = Eigen::Affine3d::Identity();

	// Fill in translation part.
	double mx = mp::merc_x(oxtsPose[1]);
	double my = mp::merc_y(oxtsPose[0]);
	transform.translation() << mx/_scale, my/_scale, oxtsPose[2];

	// Add rotation part.
	transform.rotate(Eigen::AngleAxisd(oxtsPose[5], Eigen::Vector3d::UnitZ()));
	transform.rotate(Eigen::AngleAxisd(oxtsPose[4], Eigen::Vector3d::UnitY()));
	transform.rotate(Eigen::AngleAxisd(oxtsPose[3], Eigen::Vector3d::UnitX()));

	if(!initialized){
	// normalize translation and rotation (start at 0/0/0)
		Tr_0_inv = transform.matrix().inverse();
		initialized = true;
	}

	// Add new pose.
	poses.push_back(Tr_0_inv*transform.matrix());

};


Matrix4d oxts::returnPose(void){
	if(returnIdx < poses.size()){
		returnIdx++;
		return poses[returnIdx];
	}else{
		std::cerr << "All poses have been returned. Return Identity matrix I." << std::endl;
		return Matrix4d::Identity();
	}
};

Matrix4d oxts::returnPose(const int idx){
	if(idx < poses.size()){
		return poses[idx];
	}else{
		std::cout << "Index out of range. Return Identity matrix I." << std::endl;
		return Matrix4d::Identity();
	}
};


void oxts::printAllPoses(void){
	for(int i = 0; i < poses.size(); ++i){
		std::cout << poses[i] << std::endl;
	}
};

int oxts::size(void){
	return poses.size();
}

