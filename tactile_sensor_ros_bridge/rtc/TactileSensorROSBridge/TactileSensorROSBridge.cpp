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
  this->marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("marker", 1);

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

    this->marker_.markers.resize(tactileSensorList->size());
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
      this->static_broadcaster_.sendTransform(static_transformStamped);

      this->marker_.markers[i].header.frame_id = sensor.name;
      this->marker_.markers[i].header.stamp = ros::Time::now();
      if(ros::this_node::getNamespace() == ""){
        this->marker_.markers[i].ns = ros::this_node::getName();
      }else{
        this->marker_.markers[i].ns = ros::this_node::getNamespace() + "/" + ros::this_node::getName();
      }
      this->marker_.markers[i].id = i;
      this->marker_.markers[i].lifetime = ros::Duration();
      this->marker_.markers[i].type = visualization_msgs::Marker::ARROW;
      this->marker_.markers[i].action = visualization_msgs::Marker::ADD;
      this->marker_.markers[i].pose.orientation.w = 1.0; // rvizにUninitialized quaternion, assuming identity ワーニングが出る
      this->marker_.markers[i].scale.x = 0.002; //  shaft diameter, and scale.y is the head diameter.
      this->marker_.markers[i].scale.y = 0.004; // head diameter
      this->marker_.markers[i].scale.z = 0.0; // If scale.z is not zero, it specifies the head length.
      //start end
      this->marker_.markers[i].points.resize(2);
      this->marker_.markers[i].points[0].x = 0;
      this->marker_.markers[i].points[0].y = 0;
      this->marker_.markers[i].points[0].z = 0;
      this->marker_.markers[i].points[1].x = 0;
      this->marker_.markers[i].points[1].y = 0;
      this->marker_.markers[i].points[1].z = 0;
      //color
      this->marker_.markers[i].color.r = 0.0f;
      this->marker_.markers[i].color.g = 1.0f;
      this->marker_.markers[i].color.b = 0.0f;
      this->marker_.markers[i].color.a = 1.0f;
    }
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

    if(this->m_tactileSensorArray_.data.length() == this->tactileSensorList_.size() * 3){
      for (int i=0; i<this->tactileSensorList_.size(); i++) {
        this->marker_.markers[i].header.stamp = ros::Time(0);
        this->marker_.markers[i].points[1].x = this->m_tactileSensorArray_.data[i*3+0] * 0.002;
        this->marker_.markers[i].points[1].y = this->m_tactileSensorArray_.data[i*3+1] * 0.002;
        this->marker_.markers[i].points[1].z = this->m_tactileSensorArray_.data[i*3+2] * 0.002;
      }
      this->marker_pub_.publish(this->marker_);
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
