#include "TactileSensorROSBridge.h"
#include <cnoid/YAMLReader>
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

TactileSensorROSBridge::TactileSensorROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_tactileSensorArrayIn_("tactileSensorArrayIn", m_tactileSensorArray_)
{
}

RTC::ReturnCode_t TactileSensorROSBridge::onInitialize(){
  addInPort("tactileSensorArrayIn", this->m_tactileSensorArrayIn_);
  ros::NodeHandle pnh("~");

  this->tactile_sensor_array_pub = nh.advertise<visualization_msgs::MarkerArray>("tactile_sensor", 1);
  this->arrow.x = 0.006;
  this->arrow.y = 0.01;
  this->arrow.z = 0.01;

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  // load tactile_sensor_file
  {
    std::string fileName;
    if(this->getProperties().hasKey("tactile_sensor_file")) fileName = std::string(this->getProperties()["tactile_sensor_file"]);
    else fileName = std::string(this->m_pManager->getConfig()["tactile_sensor_file"]); // 引数 -o で与えたプロパティを捕捉
    std::cerr << "[" << this->m_profile.instance_name << "] tactile_sensor_file: " << fileName <<std::endl;
    cnoid::YAMLReader reader;
    cnoid::MappingPtr node;
    try {
      node = reader.loadDocument(fileName)->toMapping();
    } catch(const cnoid::ValueNode::Exception& ex) {
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << ex.message() << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
    // load
    auto& tactileSensorList = *node->findListing("tactile_sensor");
    if (!tactileSensorList.isValid()) {
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "cannot load config file" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    } else {
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
        this->tactileSensorList.push_back(sensor);
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
      }
    }
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t TactileSensorROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  if(this->m_tactileSensorArrayIn_.isNew()){
    this->m_tactileSensorArrayIn_.read();
    if(this->m_tactileSensorArray_.data.length() != this->tactileSensorList.size()*3) {
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "data length is different. port data length : " << this->m_tactileSensorArray_.data.length() << " config file sensor length : " << this->tactileSensorList.size()*3 << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(this->tactileSensorList.size());
    for (int i=0; i < this->tactileSensorList.size(); i++) {
      //marker array arrow
      marker_array.markers[i].header.frame_id = "/map";//this->tactileSensorList[i].linkName;
      marker_array.markers[i].header.stamp = ros::Time::now();
      marker_array.markers[i].ns = "tactile_sensor_arrow";
      marker_array.markers[i].id = i;
      marker_array.markers[i].lifetime = ros::Duration();
      marker_array.markers[i].type = visualization_msgs::Marker::ARROW;
      marker_array.markers[i].action = visualization_msgs::Marker::ADD;
      marker_array.markers[i].scale = this->arrow;

      marker_array.markers[i].points.resize(2);
      // start
      geometry_msgs::Point v_start;
      v_start.x = this->tactileSensorList[i].translation[0];
      v_start.y = this->tactileSensorList[i].translation[1];
      v_start.z = this->tactileSensorList[i].translation[2];
      // end
      geometry_msgs::Point v_end;
      v_end.x = v_start.x + this->m_tactileSensorArray_.data[3*i + 0] * 100;
      v_end.y = v_start.y + this->m_tactileSensorArray_.data[3*i + 1] * 100;
      v_end.z = v_start.z + this->m_tactileSensorArray_.data[3*i + 2] * 100;
      marker_array.markers[i].points[0] = v_start;
      marker_array.markers[i].points[1] = v_end;
      //color
      marker_array.markers[i].color.r = 1.0f;
      marker_array.markers[i].color.g = 0.0f;
      marker_array.markers[i].color.b = 0.0f;
      marker_array.markers[i].color.a = 1.0f;
    }
    this->tactile_sensor_array_pub.publish(marker_array);
  }
  return RTC::RTC_OK;
}


static const char* TactileSensorROSBridge_spec[] = {
  "implementation_id", "TactileSensorROSBridge",
  "type_name",         "TactileSensorROSBridge",
  "description",       "TactileSensorROSBridge component",
  "version",           "0.0",
  "vendor",            "Takuma-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void TactileSensorROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(TactileSensorROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<TactileSensorROSBridge>, RTC::Delete<TactileSensorROSBridge>);
    }
};
