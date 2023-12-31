#include "TactileSensorROSBridge.h"
#include <cnoid/YAMLReader>
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
      this->tactile_sensor_pub.resize(tactileSensorList.size());
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
        this->tactile_sensor_pub[i] = nh.advertise<geometry_msgs::WrenchStamped>("tactile_sensor" + std::to_string(i), 10);
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
    for (int i=0; i < this->tactileSensorList.size(); i++) {
      geometry_msgs::WrenchStamped sensor;
      sensor.header.stamp = ros::Time::now();
      sensor.header.frame_id = "tactile_sensor" + std::to_string(i);
      sensor.wrench.force.x = this->m_tactileSensorArray_.data[3*i + 0];
      sensor.wrench.force.y = this->m_tactileSensorArray_.data[3*i + 1];
      sensor.wrench.force.z = this->m_tactileSensorArray_.data[3*i + 2];
      sensor.wrench.torque.x = 0.0;
      sensor.wrench.torque.y = 0.0;
      sensor.wrench.torque.z = 0.0;
      this->tactile_sensor_pub[i].publish(sensor);
    }
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
