#include <ros/ros.h>
#include "geometry_msgs/WrenchStamped.h"
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cnoid/EigenTypes>
#include <cnoid/YAMLReader>

namespace tactile_sensor_wrench_converter {
  class tactile_sensor_wrench_converter
  {
  public:
    tactile_sensor_wrench_converter() {
      ros::NodeHandle nh, pnh("~");
      tactileSensorSub_ = pnh.subscribe("input", 1, &tactile_sensor_wrench_converter::tactileSensorCallback, this);
      static tf2_ros::StaticTransformBroadcaster static_broadcaster;
      // load tactile_sensor_file
      {
	std::string fileName;
	pnh.getParam("filename", fileName);
	std::cerr <<fileName << std::endl;
	cnoid::YAMLReader reader;
	cnoid::MappingPtr node;
	try {
	  node = reader.loadDocument(fileName)->toMapping();
	} catch(const cnoid::ValueNode::Exception& ex) {
	  ROS_ERROR_STREAM(ex.message());
	}
	// load
	auto& tactileSensorList = *node->findListing("tactile_sensor");
	if (!tactileSensorList.isValid()) {
	  ROS_ERROR_STREAM("cannot load config file");
	  return;
	} else {
	  this->tactileSensorPub_.resize(tactileSensorList.size());
	  for (int i=0; i< tactileSensorList.size(); i++) {
	    cnoid::Mapping* info = tactileSensorList[i].toMapping();
	    TactileSensor sensor;
	    // link
	    std::string linkName;
	    info->extract("link", linkName);
	    sensor.linkName = linkName;
	    // translation
	    auto translation_ = info->extract("translation");
	    auto& translationTmp = *translation_->toListing();
	    cnoid::Vector3 translation = cnoid::Vector3(translationTmp[0].toDouble(), translationTmp[1].toDouble(), translationTmp[2].toDouble());
	    sensor.translation = translation;
	    // rotation
	    auto rotation_ = info->extract("rotation");
	    auto& rotationTmp = *rotation_->toListing();
	    cnoid::Matrix3d rotation;
	    rotation << rotationTmp[0].toDouble(), rotationTmp[1].toDouble(), rotationTmp[2].toDouble(),
	      rotationTmp[3].toDouble(), rotationTmp[4].toDouble(), rotationTmp[5].toDouble(),
	      rotationTmp[6].toDouble(), rotationTmp[7].toDouble(), rotationTmp[8].toDouble();
	    Eigen::Quaterniond quat(rotation);
	    this->tactileSensorList_.push_back(sensor);
	    geometry_msgs::TransformStamped static_transformStamped;
	    static_transformStamped.header.frame_id = linkName;
	    static_transformStamped.child_frame_id = "tactile_sensor" + std::to_string(i);
	    static_transformStamped.transform.translation.x = translation[0];
	    static_transformStamped.transform.translation.y = translation[1];
	    static_transformStamped.transform.translation.z = translation[2];
	    static_transformStamped.transform.rotation.x = quat.x();
	    static_transformStamped.transform.rotation.y = quat.y();
	    static_transformStamped.transform.rotation.z = quat.z();
	    static_transformStamped.transform.rotation.w = quat.w();
	    static_broadcaster.sendTransform(static_transformStamped);
	    this->tactileSensorPub_[i] = pnh.advertise<geometry_msgs::WrenchStamped>("tactile_sensor" + std::to_string(i), 10);
	  }
	}
      }
    }
    void tactileSensorCallback(const std_msgs::Float32MultiArray& msg)
    {
      if(msg.data.size() != this->tactileSensorList_.size()*3) {
	ROS_ERROR_STREAM("data length is different. port data length : " << msg.data.size() << " config file sensor length : " << this->tactileSensorList_.size()*3);
	return;
      }
      for (int i=0; i < this->tactileSensorList_.size(); i++) {
	geometry_msgs::WrenchStamped sensor;
	sensor.header.stamp = ros::Time::now();
	sensor.header.frame_id = "tactile_sensor" + std::to_string(i);
	sensor.wrench.force.x = msg.data[3*i + 0];
	sensor.wrench.force.y = msg.data[3*i + 1];
	sensor.wrench.force.z = msg.data[3*i + 2];
	sensor.wrench.torque.x = 0.0;
	sensor.wrench.torque.y = 0.0;
	sensor.wrench.torque.z = 0.0;
	this->tactileSensorPub_[i].publish(sensor);
      }
      msg.data.size();
    }
  protected:
    class TactileSensor
    {
    public:
      std::string linkName;
      cnoid::Vector3 translation; // リンク座標系でどこに取り付けられているか
    };
    ros::Subscriber tactileSensorSub_;
    std::vector<ros::Publisher> tactileSensorPub_;
    std::vector<TactileSensor> tactileSensorList_;
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tactile_sensor_wrench_converter");
  ros::NodeHandle nh;
  tactile_sensor_wrench_converter::tactile_sensor_wrench_converter t;
  ros::spin();
}
