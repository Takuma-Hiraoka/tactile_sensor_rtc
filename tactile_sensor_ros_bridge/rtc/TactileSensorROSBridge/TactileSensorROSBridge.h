#ifndef TactileSensorROSBridge_H
#define TactileSensorROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>
#include <rtm/idl/BasicDataType.hh>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <cnoid/EigenTypes>

class TactileSensorROSBridge : public RTC::DataFlowComponentBase{
protected:
  ros::NodeHandle nh;
  RTC::TimedDoubleSeq m_tactileSensorArray_;
  RTC::InPort<RTC::TimedDoubleSeq> m_tactileSensorArrayIn_;

  ros::Publisher tactile_sensor_array_pub;
public:
  class TactileSensor
  {
  public:
    std::string linkName;
    std::vector<cnoid::Vector3> positions; // リンク座標系でどこに取り付けられているか
  };
  TactileSensorROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  std::vector<TactileSensor> tactileSensorList;
  int num_sensor = 0;
  geometry_msgs::Vector3 arrow; // config arrow shape
};

extern "C"
{
  void TactileSensorROSBridgeInit(RTC::Manager* manager);
};

#endif // TactileSensorROSBridge_H
