#include "TactileSensorROSBridge.h"
#include <cnoid/YAMLReader>

TactileSensorROSBridge::TactileSensorROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_tactileSensorArrayIn_("tactileSensorArrayIn", m_tactileSensorArray_)
{
}

RTC::ReturnCode_t TactileSensorROSBridge::onInitialize(){
  addInPort("tactileSensorArrayIn", this->m_tactileSensorArrayIn_);
  ros::NodeHandle pnh("~");

  this->tactile_sensor_pub_ = pnh.advertise<std_msgs::Float32MultiArray>("output", 1);

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
    cnoid::ListingPtr tactileSensorList = node->findListing("tactile_sensor");
    if (!tactileSensorList->isValid()) {
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "cannot load config file" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }

    std::vector<geometry_msgs::TransformStamped> transforms;
    for (int i=0; i< tactileSensorList->size(); i++) {
      cnoid::Mapping* info = tactileSensorList->at(i)->toMapping();
      TactileSensor sensor;
      // name
      info->extract("name", sensor.name);
      // link
      info->extract("link", sensor.linkName);
      // translation
      cnoid::ValueNodePtr translation_ = info->extract("translation");
      if(translation_){
        cnoid::ListingPtr translationTmp = translation_->toListing();
        if(translationTmp->size()==3){
          sensor.translation = cnoid::Vector3(translationTmp->at(0)->toDouble(), translationTmp->at(1)->toDouble(), translationTmp->at(2)->toDouble());
        }
      }
      // rotation
      cnoid::ValueNodePtr rotation_ = info->extract("rotation");
      if(rotation_){
        cnoid::ListingPtr rotationTmp = rotation_->toListing();
        if(rotationTmp->size() == 4){
          sensor.rotation = cnoid::AngleAxisd(rotationTmp->at(3)->toDouble(),
                                              cnoid::Vector3{rotationTmp->at(0)->toDouble(), rotationTmp->at(1)->toDouble(), rotationTmp->at(2)->toDouble()}).toRotationMatrix();
        }
      }

      this->tactileSensorList_.push_back(sensor);

      Eigen::Quaterniond quat(sensor.rotation);
      geometry_msgs::TransformStamped static_transformStamped;
      static_transformStamped.header.frame_id = sensor.linkName;
      static_transformStamped.child_frame_id = sensor.name;
      static_transformStamped.transform.translation.x = sensor.translation[0];
      static_transformStamped.transform.translation.y = sensor.translation[1];
      static_transformStamped.transform.translation.z = sensor.translation[2];
      static_transformStamped.transform.rotation.x = quat.x();
      static_transformStamped.transform.rotation.y = quat.y();
      static_transformStamped.transform.rotation.z = quat.z();
      static_transformStamped.transform.rotation.w = quat.w();
      transforms.push_back(static_transformStamped);

    }
    // sendTransform(const geometry_msgs::TransformStamped &transform)を一つひとつ送ると時間がかかるので、sendTransform(const std::vector< geometry_msgs::TransformStamped > &transforms)でまとめて送る.
    this->static_broadcaster_.sendTransform(transforms);
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t TactileSensorROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  if(this->m_tactileSensorArrayIn_.isNew()){
    this->m_tactileSensorArrayIn_.read();
    std_msgs::Float32MultiArray tactileSensorArray;
    tactileSensorArray.data.resize(this->m_tactileSensorArray_.data.length());
    for (int i=0; i<tactileSensorArray.data.size(); i++) {
      tactileSensorArray.data[i] = this->m_tactileSensorArray_.data[i];
    }
    this->tactile_sensor_pub_.publish(tactileSensorArray);

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
