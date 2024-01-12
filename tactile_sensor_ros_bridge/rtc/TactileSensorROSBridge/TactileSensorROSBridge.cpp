#include "TactileSensorROSBridge.h"

TactileSensorROSBridge::TactileSensorROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_tactileSensorArrayIn_("tactileSensorArrayIn", m_tactileSensorArray_)
{
}

RTC::ReturnCode_t TactileSensorROSBridge::onInitialize(){
  addInPort("tactileSensorArrayIn", this->m_tactileSensorArrayIn_);
  ros::NodeHandle pnh("~");

  this->tactile_sensor_pub = pnh.advertise<std_msgs::Float32MultiArray>("output", 1);

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
    this->tactile_sensor_pub.publish(tactileSensorArray); 
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
