#include "TactileSensor.h"
#include <cnoid/YAMLReader>

TactileSensor::TactileSensor(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_tactileSensorArrayOut_("tactileSensorArrayOut", m_tactileSensorArray_)
{
}

RTC::ReturnCode_t TactileSensor::onInitialize(){
  addOutPort("estWrenchesOut", this->m_tactileSensorArrayOut_);

  int shm_key = 6555;
  this->t_shm = (struct tactile_shm *) set_shared_memory(shm_key, sizeof(struct tactile_shm));
  if (t_shm == NULL) {
    std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "set_shared_memory failed" << "\x1b[39m" << std::endl;
  }

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
      this->num_sensor = tactileSensorList.size();
    }
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t TactileSensor::onExecute(RTC::UniqueId ec_id){
  // write port
  RTC::Time tm;
  tm.sec = 0;
  tm.nsec = 0;
  m_tactileSensorArray_.tm = tm;
  m_tactileSensorArray_.data.length(this->num_sensor*3); // xyz
  for (int i=0; i < this->num_sensor*3; i++) {
    m_tactileSensorArray_.data[i] = t_shm->contact_force[i/3][i%3];
  }
  this->m_tactileSensorArrayOut_.write();
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
