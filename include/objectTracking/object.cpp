#include <objectTracking/object.h>
#include <math.h>
#include <RGBConverter/RGBConverter.h>

object::object(unsigned long time, ObjectEKF& filter, utils::OBBcube obb, float path_length, float time_since_measurement) {
  filter_ = filter;

  ObjectPath tmpPos;
  tmpPos.time = time;
  tmpPos.position = filter_.getX();
  tmpPos.quaternion = obb.quat;
  tmpPos.x    = obb.x;
  tmpPos.y    = obb.y;
  tmpPos.z    = obb.z;

  publish_ = false;
  isActive_ = true;
  isMoving_ = true;
  path_length_ = path_length;
  time_since_measurement_ = time_since_measurement;

}

void object::initializeNH(ros::NodeHandle *nh, const int id) {
  nh_  = nh;
  pub_ = nh_->advertise<visualization_msgs::Marker>("visualization_msgs", 1);
  visualID_ = id;
  publish_  = true;
}

void object::update(unsigned long time, utils::OBBcube obb) {

  size_.x    = obb.x;
  size_.y    = obb.y;
  size_.z    = obb.z;

  Vector measurement(3);
  measurement(1) = obb.position[0];
  measurement(2) = obb.position[1];
  measurement(3) = obb.position[2];
  filter_.measureUpdateStepT(time,measurement);
  ObjectPath tmpPos;
  tmpPos.time = time;
  tmpPos.position = filter_.getX();
  tmpPos.quaternion = obb.quat;
  tmpPos.x    = obb.x;
  tmpPos.y    = obb.y;
  tmpPos.z    = obb.z;
  path_.push_back(tmpPos);
  object::publishPath();
  if(!isActive_){
    isActive_ = true;
  }
}

void object::predictState(unsigned long time) {
  if(isActive_){
    Vector u(0);
    filter_.timeUpdateStepT(time, u);

    if(filter_.timeSinceMeasurement() > time_since_measurement_){
      isActive_ = false;
      if( !object::isMoving() ){
        object::removePath();
        isMoving_ = false;
      }
    }
  }
}


double object::pathLength(void){
  if(path_.size() > 1){
    return std::sqrt(
      std::pow(path_[path_.size()-1].position(1) - path_[0].position(1), 2) +
      std::pow(path_[path_.size()-1].position(2) - path_[0].position(2), 2));
  } else {
    return 0;
  }
}

void object::publishPath(void) {

  if (publish_ && (path_.size() > 1)) {
      visualization_msgs::Marker line_list;
      line_list.type            = visualization_msgs::Marker::LINE_LIST;
      line_list.action = visualization_msgs::Marker::ADD;
      line_list.scale.x         = 0.2;
      line_list.color.r         = color_.R;
      line_list.color.g         = color_.G;
      line_list.color.b         = color_.B;
      line_list.color.a         = 1.0;
      line_list.header.frame_id = "odom";
      line_list.ns              = "paths";
      line_list.id              = visualID_;

      geometry_msgs::Point p;

      for (int i = 0; i < path_.size() - 1; ++i) {
        p.x = path_[i].position(1);
        p.y = path_[i].position(2);
        p.z = path_[i].position(3);
        line_list.points.push_back(p);
        p.x = path_[i + 1].position(1);
        p.y = path_[i + 1].position(2);
        p.z = path_[i + 1].position(3);
        line_list.points.push_back(p);
      }
      pub_.publish(line_list);
  }
}


void object::removePath(void){
  visualization_msgs::Marker line_list;
  line_list.type            = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x         = 1.0;
  line_list.color.r         = 1.0;
  line_list.color.g         = 1.0;
  line_list.color.b         = 1.0;
  line_list.color.a         = 1.0;
  line_list.header.frame_id = "odom";
  line_list.ns              = "paths";
  line_list.id              = (int)visualID_;
  line_list.action = visualization_msgs::Marker::DELETE;
  pub_.publish(line_list);
}


double object::timeSinceMeasurement(void){
  return filter_.timeSinceMeasurement();
}


void object::setColor(float h){
  RGBConverter::hsvToRgb(h,0.7,0.95, &color_.R, &color_.G, &color_.B);
}
