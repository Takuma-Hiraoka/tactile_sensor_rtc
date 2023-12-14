#include "TactileSensor.h"

TactileSensor::TactileSensor(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_tactileSensorArrayOut_("tactileSensorArrayOut", m_tactileSensorArray_)
{
}

RTC::ReturnCode_t TactileSensor::onInitialize(){
  addOutPort("estWrenchesOut", this->m_tactileSensorArrayOut_);
  return RTC::RTC_OK;
}

RTC::ReturnCode_t TactileSensor::onExecute(RTC::UniqueId ec_id){
  return RTC::RTC_OK;
}

static const char* TactileSensor_spec[] = {
  "implementation_id", "TactileSensor",
  "type_name",         "TactileSensor",
  "description",       "TactileSensor component",
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
  void TactileSensorInit(RTC::Manager* manager) {
    RTC::Properties profile(TactileSensor_spec);
    manager->registerFactory(profile, RTC::Create<TactileSensor>, RTC::Delete<TactileSensor>);
  }
};
