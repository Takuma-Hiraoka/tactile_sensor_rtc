#ifndef TactileSensor_H
#define TactileSensor_H

#include <memory>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataType.hh>

#include "tactile_shm.h"

class TactileSensor : public RTC::DataFlowComponentBase{
protected:
  RTC::TimedDoubleSeq m_tactileSensorArray_;
  RTC::OutPort<RTC::TimedDoubleSeq> m_tactileSensorArrayOut_;
public:
  TactileSensor(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

private:
  struct tactile_shm *t_shm;
  int num_sensor = 0;
};

extern "C"
{
  void TactileSensorInit(RTC::Manager* manager);
}

#endif // TactileSensor_H
