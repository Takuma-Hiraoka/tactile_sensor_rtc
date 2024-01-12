#ifndef TactileSensorROSBridge_H
#define TactileSensorROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>
#include <rtm/idl/BasicDataType.hh>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

class TactileSensorROSBridge : public RTC::DataFlowComponentBase{
protected:
  ros::NodeHandle nh;
  RTC::TimedDoubleSeq m_tactileSensorArray_;
  RTC::InPort<RTC::TimedDoubleSeq> m_tactileSensorArrayIn_;

  ros::Publisher tactile_sensor_pub;
public:
  TactileSensorROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};

extern "C"
{
  void TactileSensorROSBridgeInit(RTC::Manager* manager);
};

#endif // TactileSensorROSBridge_H
