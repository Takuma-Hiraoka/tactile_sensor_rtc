#ifndef TactileSensor_H
#define TactileSensor_H

#include <memory>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

class TactileSensor : public RTC::DataFlowComponentBase{
protected:

public:
  TactileSensor(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

private:
};

extern "C"
{
  void TactileSensorInit(RTC::Manager* manager);
}

#endif // TactileSensor_H
