#ifndef TactileSensorROSBridge_H
#define TactileSensorROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>
#include <rtm/idl/BasicDataType.hh>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <cnoid/Body>

class TactileSensorROSBridge : public RTC::DataFlowComponentBase{
protected:
  class TactileSensor
  {
  public:
    // from config file
    std::string name;
    std::string linkName; // 親リンク名 (!= ジョイント名)
    cnoid::LinkPtr link;
    cnoid::Vector3 translation = cnoid::Vector3::Zero(); // リンク座標系でどこに取り付けられているか
    cnoid::Matrix3 rotation = cnoid::Matrix3::Identity(); // リンク座標系でセンサの姿勢．zがリンク内側方向
  };


  ros::NodeHandle nh_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  RTC::TimedDoubleSeq m_tactileSensorArray_;
  RTC::InPort<RTC::TimedDoubleSeq> m_tactileSensorArrayIn_;

  ros::Publisher tactile_sensor_pub_;

  std::vector<TactileSensor> tactileSensorList_;

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
