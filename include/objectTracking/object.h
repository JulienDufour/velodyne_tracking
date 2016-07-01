// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// object.h
// Purpose: Class that handles a single moving object. Keeps track of the
// object size and uses Extended Kalman Filter for position estimation.

// @author Unnar Þór Axelsson
// @version 1.0 18/01/16
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


#ifndef OBJECT_H
#define OBJECT_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <objectEKF/objectEKF.h>
#include <utils/clustering.h>

class object {

public:
  struct ObjectPath{
    unsigned long time;
    ObjectEKF::Vector position;
    Eigen::Quaternionf quaternion;
    float x;
    float y;
    float z;
  };

private:

  struct obbSize {
    float x;
    float y;
    float z;
  };

  struct obbColor {
    float R;
    float G;
    float B;
  };

  ros::NodeHandle *nh_;
  obbSize size_;
  obbColor    color_;
  std::vector < ObjectPath > path_;
  ros::Publisher pub_;

  int visualID_;
  bool   publish_;
  unsigned long time_;
  unsigned long measuretime_;
  ObjectEKF filter_;
  bool isActive_;
  bool isMoving_;
  float path_length_, time_since_measurement_;

public:

  // Constructor
  // Inputs:
  //    time: The time of the measurement in the form of a double.
  //    position: Estimated x/y/z position.
  //    sx, sy, sz: estimated size of the sides of the smalles oriented bounding
  // box.
  object(unsigned long time, ObjectEKF& filter, utils::OBBcube obb, float path_length, float time_since_measurement);
  ~object() {};


  // Used to initialize ros nodehandle to publish on topics, used for
  // vizualisation.
  // This can be skipped if vizualisation is not wanted.
  // Input:
  //    nh: The ros nodehandle.
  //    id: Integar id, used when publishing to RVIZ.
  void initializeNH(ros::NodeHandle *nh,
                    const int        id);

  // Update size and position of object.
  // Inputs:
  //    time: The time of the measurement in the form of a double.
  //    position: Estimated x/y/z position.
  //    sx, sy, sz: estimated size of the sides of the smalles oriented bounding
  // box.
  void update(unsigned long time, utils::OBBcube obb);


  void predictState(unsigned long time);

  ObjectEKF::Vector returnState() {
    return filter_.getX();
  }

  // return colors.
  float getRed(void) {
    return color_.R;
  }


  float getGreen(void) {
    return color_.G;
  }

  float getBlue(void) {
    return color_.B;
  }

  // Return the path the object has moved after.
  std::vector < ObjectPath > getPath(void) {
    return path_;
  }

  bool isActive(void){
    return isActive_;
  }

  bool isMoving(void){
    if(isMoving_ == true){
      if(pathLength() < path_length_){
        isMoving_ = false;
      }
    }
    return isMoving_;
  }

  int getID(void){
    return visualID_;
  }

  double timeSinceMeasurement(void);
  void setColor(float h);

  void printIfNotIsMoving(void){
    if(isMoving_){
      std::cout << visualID_ << std::endl;
    }
  }

  void publishPath(void);

  double pathLength(void);

private:

  // Publishes the path to a topic that can be viewed in RVIZ.
  // Removes the path from RVIZ if detected as not an object.
  void removePath(void);
};
#endif /* ifndef OBJECT_H */
